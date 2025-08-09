`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/29/2025 10:07:29 AM
// Design Name: 
// Module Name: tlp_dma_rd_rc_fifo
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


module tlp_dma_rd_rc_fifo #(
	parameter C_DATA_WIDTH     = 64,
	parameter C_RC_COUNT       = 15,
	parameter C_MAX_BURST_SIZE = 256,

	// Do not override parameters below this line
	parameter KEEP_WIDTH = C_DATA_WIDTH / 8
) (
	input clk,
	input rst_n,

	// RC TLP
	input [C_DATA_WIDTH-1:0] s_axis_rc_tdata,
	input [KEEP_WIDTH-1:0]   s_axis_rc_tkeep,
	input                    s_axis_rc_tlast,
	input                    s_axis_rc_tvalid,
	output reg               s_axis_rc_tready,

	// RC FIFO
	output [C_RC_COUNT*C_DATA_WIDTH-1:0] m_axis_fifo_tdata,
	output [C_RC_COUNT*KEEP_WIDTH-1:0]   m_axis_fifo_tkeep,
	output [C_RC_COUNT-1:0]              m_axis_fifo_tlast,
	output [C_RC_COUNT-1:0]              m_axis_fifo_tvalid,
	input [C_RC_COUNT-1:0]               m_axis_fifo_tready
);
	// `define USE_AXIS_FIFO

	localparam RC_CNT_WIDTH = $clog2(C_RC_COUNT + 1);

	localparam TLP_HDR_COUNT     = 2;
	localparam TLP_HDR_CNT_WIDTH = $clog2(TLP_HDR_COUNT);

	localparam AP_STATE_IDLE    = 2'd0;
	localparam AP_STATE_TLP_HDR = 2'd1;
	localparam AP_STATE_RC_FIFO = 2'd2;
	localparam AP_STATE_WIDTH   = 2;

	genvar gen_i;
	integer i;

	// ap signals
	reg [AP_STATE_WIDTH-1:0]    ap_state;
	reg [TLP_HDR_CNT_WIDTH-1:0] tlp_hdr_idx;
	reg [C_DATA_WIDTH-1:0]      fifo_tdata [TLP_HDR_COUNT-1:0];
	reg [KEEP_WIDTH-1:0]        fifo_tkeep [TLP_HDR_COUNT-1:0];
	reg                         fifo_tlast [TLP_HDR_COUNT-1:0];
	reg [RC_CNT_WIDTH-1:0]      rc_idx;
	reg                         demux_fifo;

	// fifo_dma_rd_U signals
	reg [C_DATA_WIDTH-1:0]  fifo_s_axis_tdata [C_RC_COUNT-1:0];
	reg [KEEP_WIDTH-1:0]    fifo_s_axis_tkeep [C_RC_COUNT-1:0];
	reg                     fifo_s_axis_tlast [C_RC_COUNT-1:0];
	reg                     fifo_s_axis_tvalid [C_RC_COUNT-1:0];
	wire                    fifo_s_axis_tready [C_RC_COUNT-1:0];
	wire [C_RC_COUNT-1:0]   fifo_s_axis_fire;

	assign s_axis_rc_fire = s_axis_rc_tready & s_axis_rc_tvalid;

	generate
		for(gen_i = 0;gen_i < C_RC_COUNT;gen_i = gen_i + 1) begin
`ifdef USE_AXIS_FIFO
			axis_fifo #(
				.DATA_WIDTH(C_DATA_WIDTH),
				.FIFO_DEPTH(C_MAX_BURST_SIZE >> 2)
			) fifo_dma_rd_U(
				.clk(clk),
				.rst_n(rst_n),

				.s_axis_tvalid(fifo_s_axis_tvalid[gen_i]),
				.s_axis_tready(fifo_s_axis_tready[gen_i]),
				.s_axis_tdata(fifo_s_axis_tdata[gen_i]),
				.s_axis_tkeep(fifo_s_axis_tkeep[gen_i]),
				.s_axis_tlast(fifo_s_axis_tlast[gen_i]),

				.m_axis_tvalid(m_axis_fifo_tvalid[gen_i]),
				.m_axis_tready(m_axis_fifo_tready[gen_i]),
				.m_axis_tdata(m_axis_fifo_tdata[gen_i*C_DATA_WIDTH +: C_DATA_WIDTH]),
				.m_axis_tkeep(m_axis_fifo_tkeep[gen_i*KEEP_WIDTH +: KEEP_WIDTH]),
				.m_axis_tlast(m_axis_fifo_tlast[gen_i]),

				.fifo_full(),
				.fifo_empty()
			);
`else // USE_AXIS_FIFO
			fifo_dma_rd fifo_dma_rd_U(
				.s_aclk(clk),
				.s_aresetn(rst_n),

				.s_axis_tvalid(fifo_s_axis_tvalid[gen_i]),
				.s_axis_tready(fifo_s_axis_tready[gen_i]),
				.s_axis_tdata(fifo_s_axis_tdata[gen_i]),
				.s_axis_tkeep(fifo_s_axis_tkeep[gen_i]),
				.s_axis_tlast(fifo_s_axis_tlast[gen_i]),

				.m_axis_tvalid(m_axis_fifo_tvalid[gen_i]),
				.m_axis_tready(m_axis_fifo_tready[gen_i]),
				.m_axis_tdata(m_axis_fifo_tdata[gen_i*C_DATA_WIDTH +: C_DATA_WIDTH]),
				.m_axis_tkeep(m_axis_fifo_tkeep[gen_i*KEEP_WIDTH +: KEEP_WIDTH]),
				.m_axis_tlast(m_axis_fifo_tlast[gen_i]),

				.wr_rst_busy(),
				.rd_rst_busy()
			);
`endif // USE_AXIS_FIFO

			assign fifo_s_axis_fire[gen_i] = fifo_s_axis_tready[gen_i] && fifo_s_axis_tvalid[gen_i];
		end
	endgenerate

	// @FF ap_state, @FF tlp_hdr_idx, @FF rc_idx, @FF demux_fifo
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_IDLE;
			tlp_hdr_idx <= 0;
			rc_idx <= 0;
			demux_fifo <= 0;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					ap_state <= AP_STATE_TLP_HDR;
					tlp_hdr_idx <= 0;
				end

				AP_STATE_TLP_HDR: begin
					if(s_axis_rc_fire) begin
						if(tlp_hdr_idx == 1) begin
							ap_state <= AP_STATE_RC_FIFO;
							rc_idx <= s_axis_rc_tdata[8 +: RC_CNT_WIDTH];
							demux_fifo <= 0;

							if(s_axis_rc_tlast) begin
								demux_fifo <= 1;
							end
						end else begin
							tlp_hdr_idx <= 1;
						end
					end
				end

				AP_STATE_RC_FIFO: begin
					if(demux_fifo) begin
						if(fifo_s_axis_fire[rc_idx]) begin
							if(fifo_s_axis_tlast[rc_idx]) begin
								ap_state <= AP_STATE_TLP_HDR;
								tlp_hdr_idx <= 0;
							end
						end
					end else begin
						if(s_axis_rc_fire) begin
							if(s_axis_rc_tlast) begin
								demux_fifo <= 1;
							end
						end
					end
				end
			endcase
		end
	end

	// @FF fifo_tdata[i], @FF fifo_tkeep[i], @FF fifo_tlast[i]
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			for(i = 0;i < TLP_HDR_COUNT;i = i + 1) begin
				fifo_tdata[i] <= 0;
				fifo_tkeep[i] <= 0;
				fifo_tlast[i] <= 0;
			end
		end else begin
			case(ap_state)
				AP_STATE_TLP_HDR: begin
					if(s_axis_rc_fire) begin
						fifo_tdata[tlp_hdr_idx] <= s_axis_rc_tdata;
						fifo_tkeep[tlp_hdr_idx] <= s_axis_rc_tkeep;
						fifo_tlast[tlp_hdr_idx] <= s_axis_rc_tlast;
					end
				end

				AP_STATE_RC_FIFO: begin
					if(demux_fifo) begin
						if(fifo_s_axis_fire[rc_idx]) begin
							fifo_tdata[0] <= fifo_tdata[1];
							fifo_tkeep[0] <= fifo_tkeep[1];
							fifo_tlast[0] <= fifo_tlast[1];
							fifo_tdata[1] <= 'hCAFE0001; // for debug purpose
							fifo_tkeep[1] <= 0;
							fifo_tlast[1] <= 0;
						end
					end else begin
						if(s_axis_rc_fire) begin
							fifo_tdata[0] <= fifo_tdata[1];
							fifo_tkeep[0] <= fifo_tkeep[1];
							fifo_tlast[0] <= fifo_tlast[1];
							fifo_tdata[1] <= s_axis_rc_tdata;
							fifo_tkeep[1] <= s_axis_rc_tkeep;
							fifo_tlast[1] <= s_axis_rc_tlast;
						end
					end
				end
			endcase
		end
	end

	// @COMB s_axis_rc_tready
	always @(*) begin
		s_axis_rc_tready = 0;

		case(ap_state)
			AP_STATE_TLP_HDR: begin
				s_axis_rc_tready = 1;
			end

			AP_STATE_RC_FIFO: begin
				if(! demux_fifo) begin
					s_axis_rc_tready = fifo_s_axis_tready[rc_idx];
				end
			end
		endcase
	end

	// @COMB fifo_s_axis_tdata[i], @COMB fifo_s_axis_tkeep[i],
	// @COMB fifo_s_axis_tlast[i], @COMB fifo_s_axis_tvalid[i]
	always @(*) begin
		for(i = 0;i < C_RC_COUNT;i = i + 1) begin
			fifo_s_axis_tdata[i] = 'hCAFE0002;
			fifo_s_axis_tkeep[i] = 0;
			fifo_s_axis_tlast[i] = 0;
			fifo_s_axis_tvalid[i] = 0;
		end

		case(ap_state)
			AP_STATE_RC_FIFO: begin
				fifo_s_axis_tdata[rc_idx] = fifo_tdata[0];
				fifo_s_axis_tkeep[rc_idx] = fifo_tkeep[0];
				fifo_s_axis_tlast[rc_idx] = fifo_tlast[0];

				if(demux_fifo) begin
					fifo_s_axis_tvalid[rc_idx] = 1;
				end else begin
					fifo_s_axis_tvalid[rc_idx] = s_axis_rc_tvalid;
				end
			end
		endcase
	end
endmodule
