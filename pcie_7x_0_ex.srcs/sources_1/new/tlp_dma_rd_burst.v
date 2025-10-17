`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/10/2025 02:07:29 PM
// Design Name: 
// Module Name: tlp_dma_rd_burst
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module tlp_dma_rd_burst #(
	parameter C_DATA_WIDTH     = 64,
	parameter C_MAX_BURST_SIZE = 256, // bytes per burst
	parameter C_RC_COUNT       = 15,
	parameter C_RC_CNT_WIDTH   = 4,

	// Do not override parameters below this line
	parameter KEEP_WIDTH = C_DATA_WIDTH / 8,
	parameter BURST_WIDTH = $clog2(C_MAX_BURST_SIZE + 1)
) (
	input clk,
	input rst_n,

	// RC FIFO TLP
	input [C_RC_COUNT*C_DATA_WIDTH-1:0] s_axis_fifo_tdata,
	input [C_RC_COUNT*KEEP_WIDTH-1:0]   s_axis_fifo_tkeep,
	input [C_RC_COUNT-1:0]              s_axis_fifo_tlast,
	input [C_RC_COUNT-1:0]              s_axis_fifo_tvalid,
	output reg [C_RC_COUNT-1:0]         s_axis_fifo_tready,

	// TX TLP
	output reg [C_DATA_WIDTH-1:0] m_axis_tdata,
	output reg [KEEP_WIDTH-1:0]   m_axis_tkeep,
	output reg                    m_axis_tlast,
	output reg                    m_axis_tvalid,
	input                         m_axis_tready,

	// ap args
	input [15:0]    s_axis_ap_tdata,
	input                      s_axis_ap_tvalid,
	output reg                 s_axis_ap_tready,
	output                     fifo_tx_empty
);
	localparam AP_STATE_IDLE  = 1'd0;
	localparam AP_STATE_RX    = 1'd1;
	localparam AP_STATE_WIDTH = 1;

	localparam TX_STATE_IDLE  = 1'd0;
	localparam TX_STATE_TX    = 1'd1;
	localparam TX_STATE_WIDTH = 1;

	genvar gen_i;
	integer i;

	// ap signals
	reg [AP_STATE_WIDTH-1:0] ap_state;
	wire                     m_axis_fire;
	wire                     s_axis_ap_fire;
	reg [TX_STATE_WIDTH-1:0] tx_state;
	reg [0:0]                tlp_hdr_idx;
	reg [C_RC_CNT_WIDTH-1:0] tx_burst_idx;
	reg [BURST_WIDTH-1:0]    tx_burst_bytes;
	wire [BURST_WIDTH-1:0]   s_axis_ap_burst_bytes;

	// fifo_tx_U signals
	reg                    fifo_tx_wr_en;
	reg [BURST_WIDTH-1:0]  fifo_tx_wr_data;
	reg                    fifo_tx_rd_en;
	wire [BURST_WIDTH-1:0] fifo_tx_rd_data;
	wire                   fifo_tx_data_valid;
	wire                   fifo_tx_full;
	wire [4:0]             fifo_tx_count;

	assign m_axis_fire = m_axis_tready & m_axis_tvalid;
	assign s_axis_ap_fire = s_axis_ap_tready & s_axis_ap_tvalid;
	assign s_axis_ap_burst_bytes = s_axis_ap_tdata[0 +: BURST_WIDTH];

	fifo_fwft2 #(
		.DATA_WIDTH(BURST_WIDTH),
		.DEPTH(16)
	) fifo_tx_U (
		.clk(clk),
		.rst_n(rst_n),

		.wr_en(fifo_tx_wr_en),
		.wr_data(fifo_tx_wr_data),

		.rd_en(fifo_tx_rd_en),
		.rd_data(fifo_tx_rd_data),

		.full(fifo_tx_full),
		.empty(fifo_tx_empty),

		.size(fifo_tx_count)
	);

	assign fifo_tx_data_valid = !fifo_tx_empty;

	// @FF ap_state
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_IDLE;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					ap_state <= AP_STATE_RX;
				end

				AP_STATE_RX: begin
				end
			endcase
		end
	end

	// @COMB s_axis_ap_tready
	always @(*) begin
		s_axis_ap_tready = 0;

		case(ap_state)
			AP_STATE_RX: begin
				if(fifo_tx_count < C_RC_COUNT) begin
					s_axis_ap_tready = 1;
				end
			end
		endcase
	end

	// @COMB fifo_tx_wr_en, @COMB fifo_tx_wr_data
	always @(*) begin
		fifo_tx_wr_en = 0;
		fifo_tx_wr_data = 0;

		case(ap_state)
			AP_STATE_RX: begin
				fifo_tx_wr_en = s_axis_ap_fire;
				fifo_tx_wr_data = s_axis_ap_burst_bytes;
			end
		endcase
	end

	// @COMB fifo_tx_rd_en
	always @(*) begin
		fifo_tx_rd_en = 0;

		case(tx_state)
			TX_STATE_TX: begin
				if(m_axis_fire) begin
					if(tlp_hdr_idx == 0) begin
					end else begin
						if(m_axis_tlast) begin
							if(tx_burst_bytes == 0) begin
								fifo_tx_rd_en = 1;
							end
						end
					end
				end
			end
		endcase
	end

	// @FF tx_state, @FF tx_burst_idx, @FF tx_burst_bytes, @FF tlp_hdr_idx
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			tx_state <= TX_STATE_IDLE;
			tx_burst_idx <= 0;
			tx_burst_bytes <= 0;
			tlp_hdr_idx <= 0;
		end else begin
			case(tx_state)
				TX_STATE_IDLE: begin
					if(fifo_tx_data_valid) begin
						tx_state <= TX_STATE_TX;
						tx_burst_bytes <= fifo_tx_rd_data;
						tlp_hdr_idx <= 0;
					end
				end

				TX_STATE_TX: begin
					if(m_axis_fire) begin
						if(tlp_hdr_idx == 0) begin
							tlp_hdr_idx <= 1;
							tx_burst_bytes <= tx_burst_bytes - (s_axis_fifo_tdata[(tx_burst_idx*C_DATA_WIDTH+0) +: 10] << 2);
						end else begin
							if(m_axis_tlast) begin
								if(tx_burst_bytes == 0) begin
									tx_state <= TX_STATE_IDLE;
									tx_burst_idx <= (tx_burst_idx == C_RC_COUNT - 1) ? 0 : tx_burst_idx + 1;
								end else begin
									tlp_hdr_idx <= 0;
								end
							end
						end
					end
				end
			endcase
		end
	end

	// @COMB s_axis_fifo_tready[i], @COMB m_axis_tdata, @COMB m_axis_tkeep, @COMB m_axis_tlast, @COMB m_axis_tvalid
	always @(*) begin
		for(i = 0;i < C_RC_COUNT;i = i + 1) begin
			s_axis_fifo_tready[i] = 0;
		end

		m_axis_tdata = 'hCAFE0001;
		m_axis_tkeep = 0;
		m_axis_tlast = 0;
		m_axis_tvalid = 0;

		case(tx_state)
			TX_STATE_TX: begin
				s_axis_fifo_tready[tx_burst_idx] = m_axis_tready;
				m_axis_tkeep = s_axis_fifo_tkeep[tx_burst_idx];
				m_axis_tlast = s_axis_fifo_tlast[tx_burst_idx];
				m_axis_tvalid = s_axis_fifo_tvalid[tx_burst_idx];
			end
		endcase
	end
endmodule