`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/18/2025 17:30:23 PM
// Design Name: 
// Module Name: tlp_demuxer_rc_desc
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

module tlp_demuxer_rc_desc #(
	parameter C_DATA_WIDTH    = 64,
	parameter C_USER_WIDTH    = 22,
	parameter C_TLP_HDR_COUNT = 2,
	parameter C_RC_DESC_COUNT = 2,

	// Do not override parameters below this line
	parameter KEEP_WIDTH = C_DATA_WIDTH / 8,
	parameter RC_DESC_CNT_WIDTH = $clog2(C_RC_DESC_COUNT + 1)
)
(
	input clk,
	input rst_n,

	// RX
	input [C_DATA_WIDTH-1:0] s_axis_rx_tdata,
	input [KEEP_WIDTH-1:0]   s_axis_rx_tkeep,
	input                    s_axis_rx_tlast,
	input                    s_axis_rx_tvalid,
	output reg               s_axis_rx_tready,
	input [C_USER_WIDTH-1:0] s_axis_rx_tuser,

	// RC DESC
	output reg [C_DATA_WIDTH*C_RC_DESC_COUNT-1:0] m_axis_rc_desc_tdata,
	output reg [KEEP_WIDTH*C_RC_DESC_COUNT-1:0]   m_axis_rc_desc_tkeep,
	output reg [C_RC_DESC_COUNT-1:0]              m_axis_rc_desc_tlast,
	output reg [C_RC_DESC_COUNT-1:0]              m_axis_rc_desc_tvalid,
	input [C_RC_DESC_COUNT-1:0]                   m_axis_rc_desc_tready,

	// ap args
	input [C_DATA_WIDTH*C_TLP_HDR_COUNT-1:0] s_axis_ap_tlp_hdr_tdata,
	input [KEEP_WIDTH*C_TLP_HDR_COUNT-1:0]   s_axis_ap_tlp_hdr_tkeep,
	input [C_TLP_HDR_COUNT-1:0]              s_axis_ap_tlp_hdr_tlast,
	input                                    s_axis_ap_tvalid,
	output reg                               s_axis_ap_tready
);
	localparam AP_STATE_IDLE  = 1'd0;
	localparam AP_STATE_DEMUX = 1'd1;
	localparam AP_STATE_WIDTH = 1;

	integer i;
	genvar gen_i;

	reg [AP_STATE_WIDTH-1:0]   ap_state;
	wire                       s_axis_ap_fire;
	wire                       s_axis_rx_fire;
	wire [C_RC_DESC_COUNT-1:0] m_axis_rc_desc_fire;

	reg [RC_DESC_CNT_WIDTH-1:0] desc_idx;
	reg                         demux_fifo; // demux from FIFO
	reg [C_DATA_WIDTH-1:0]      fifo_tdata [C_TLP_HDR_COUNT-1:0];
	reg [KEEP_WIDTH-1:0]        fifo_tkeep [C_TLP_HDR_COUNT-1:0];
	reg                         fifo_tlast [C_TLP_HDR_COUNT-1:0];

	assign s_axis_ap_fire = s_axis_ap_tready && s_axis_ap_tvalid;
	assign s_axis_rx_fire = s_axis_rx_tready && s_axis_rx_tvalid;

	generate
		for(gen_i = 0;gen_i < C_RC_DESC_COUNT;gen_i = gen_i + 1) begin
			assign m_axis_rc_desc_fire[gen_i] = m_axis_rc_desc_tready[gen_i] && m_axis_rc_desc_tvalid[gen_i];
		end
	endgenerate

	// @FF ap_state, @FF demux_fifo
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_IDLE;
			demux_fifo <= 0;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					if(s_axis_ap_fire) begin
						ap_state <= AP_STATE_DEMUX;
						demux_fifo <= 0;
						desc_idx <= s_axis_ap_tlp_hdr_tdata[(1*C_DATA_WIDTH+8) +: RC_DESC_CNT_WIDTH];

						if(s_axis_ap_tlp_hdr_tlast[C_TLP_HDR_COUNT-1]) begin
							demux_fifo <= 1;
						end
					end
				end

				AP_STATE_DEMUX: begin
					if(demux_fifo) begin
						if(m_axis_rc_desc_fire[desc_idx]) begin
							if(m_axis_rc_desc_tlast[desc_idx]) begin
								ap_state <= AP_STATE_IDLE;
							end
						end
					end else begin
						if(s_axis_rx_fire) begin
							if(s_axis_rx_tlast) begin
								demux_fifo <= 1; // switch to fifo
							end
						end
					end
				end
			endcase
		end
	end

	// @COMB s_axis_ap_tready, @COMB s_axis_rx_tready
	always @(*) begin
		s_axis_ap_tready = 0;
		s_axis_rx_tready = 0;

		case(ap_state)
			AP_STATE_IDLE: begin
				s_axis_ap_tready = 1;
			end

			AP_STATE_DEMUX: begin
				if(! demux_fifo) begin
					s_axis_rx_tready = m_axis_rc_desc_tready[desc_idx];
				end
			end
		endcase
	end

	// @COMB m_axis_rc_desc_tdata[i], @COMB m_axis_rc_desc_tkeep[i],
	// @COMB m_axis_rc_desc_tlast, @COMB m_axis_rc_desc_tvalid
	always @(*) begin
		for(i = 0;i < C_RC_DESC_COUNT;i = i + 1) begin
			m_axis_rc_desc_tdata[i*C_DATA_WIDTH +: C_DATA_WIDTH] = 'hCAFE0002;
			m_axis_rc_desc_tkeep[i*KEEP_WIDTH +: KEEP_WIDTH] = 0;
			m_axis_rc_desc_tlast[i] = 0;
			m_axis_rc_desc_tvalid[i] = 0;
		end

		case(ap_state)
			AP_STATE_DEMUX: begin
				m_axis_rc_desc_tdata[desc_idx*C_DATA_WIDTH +: C_DATA_WIDTH] = fifo_tdata[0];
				m_axis_rc_desc_tkeep[desc_idx*KEEP_WIDTH +: KEEP_WIDTH] = fifo_tkeep[0];
				m_axis_rc_desc_tlast[desc_idx] = fifo_tlast[0];

				if(demux_fifo) begin
					m_axis_rc_desc_tvalid[desc_idx] = 1;
				end else begin
					m_axis_rc_desc_tvalid[desc_idx] = s_axis_rx_tvalid;
				end
			end
		endcase
	end

	// @FF fifo_tdata[i], @FF fifo_tkeep[i], @FF fifo_tlast[i]
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			for(i = 0;i < C_TLP_HDR_COUNT;i = i + 1) begin
				fifo_tdata[i] <= 'hCAFE003;
			end
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					if(s_axis_ap_fire) begin
						for(i = 0;i < C_TLP_HDR_COUNT;i = i + 1) begin
							fifo_tdata[i] <= s_axis_ap_tlp_hdr_tdata[i*C_DATA_WIDTH +: C_DATA_WIDTH];
							fifo_tkeep[i] <= s_axis_ap_tlp_hdr_tkeep[i*KEEP_WIDTH +: KEEP_WIDTH];
							fifo_tlast[i] <= s_axis_ap_tlp_hdr_tlast[i];
						end
					end
				end

				AP_STATE_DEMUX: begin
					if(demux_fifo) begin
						if(m_axis_rc_desc_fire[desc_idx]) begin
							fifo_tdata[0] <= fifo_tdata[1];
							fifo_tkeep[0] <= fifo_tkeep[1];
							fifo_tlast[0] <= fifo_tlast[1];
							fifo_tdata[1] <= 'hCAFE0001; // for debug purpose
							fifo_tkeep[1] <= 0;
							fifo_tlast[1] <= 0;
						end
					end else begin
						if(s_axis_rx_fire) begin
							fifo_tdata[0] <= fifo_tdata[1];
							fifo_tkeep[0] <= fifo_tkeep[1];
							fifo_tlast[0] <= fifo_tlast[1];
							fifo_tdata[1] <= s_axis_rx_tdata;
							fifo_tkeep[1] <= s_axis_rx_tkeep;
							fifo_tlast[1] <= s_axis_rx_tlast;
						end
					end
				end
			endcase
		end
	end
endmodule