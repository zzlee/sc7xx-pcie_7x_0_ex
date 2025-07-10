`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/21/2025 03:35:23 PM
// Design Name: 
// Module Name: tlp_demuxer
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


module tlp_demuxer #(
	parameter C_DATA_WIDTH = 64,
	parameter C_USER_WIDTH = 22,

	// Do not override parameters below this line
	parameter KEEP_WIDTH = C_DATA_WIDTH / 8
)
(
	input clk,
	input rst_n,

	// RX TLP
	input [C_DATA_WIDTH-1:0] m_axis_rx_tdata,
	input [KEEP_WIDTH-1:0]   m_axis_rx_tkeep,
	input                    m_axis_rx_tlast,
	input                    m_axis_rx_tvalid,
	output reg               m_axis_rx_tready,
	input [C_USER_WIDTH-1:0] m_axis_rx_tuser,

	// CQ TLP
	output [C_DATA_WIDTH-1:0] s_axis_cq_tdata,
	output [KEEP_WIDTH-1:0]   s_axis_cq_tkeep,
	output                    s_axis_cq_tlast,
	output                    s_axis_cq_tvalid,
	input                     s_axis_cq_tready,
	output [C_USER_WIDTH-1:0] s_axis_cq_tuser,

	// RC00 TLP
	output [C_DATA_WIDTH-1:0] s00_axis_rc_tdata,
	output [KEEP_WIDTH-1:0]   s00_axis_rc_tkeep,
	output                    s00_axis_rc_tlast,
	output                    s00_axis_rc_tvalid,
	input                     s00_axis_rc_tready,
	output [C_USER_WIDTH-1:0] s00_axis_rc_tuser,

	// RC01 TLP
	output [C_DATA_WIDTH-1:0] s01_axis_rc_tdata,
	output [KEEP_WIDTH-1:0]   s01_axis_rc_tkeep,
	output                    s01_axis_rc_tlast,
	output                    s01_axis_rc_tvalid,
	input                     s01_axis_rc_tready,
	output [C_USER_WIDTH-1:0] s01_axis_rc_tuser,

	input [15:0]                  cfg_completer_id,
	output reg [C_DATA_WIDTH-1:0] tlp_hdr_dw1_0
);
	localparam TLP_MEM_TYPE = 5'b00000;
	localparam TLP_CPL_TYPE = 5'b01010;

	localparam RC_COUNT     = 2;
	localparam RC_CNT_WIDTH = $clog2(RC_COUNT + 1);

	localparam STATE_BITS           = 4;
	localparam STATE_IDLE           = 4'd0;
	localparam STATE_DEMUX          = 4'd1;
	localparam STATE_DEMUX_CQ       = 4'd2;
	localparam STATE_DEMUX_RC_DW3_2 = 4'd3;
	localparam STATE_DEMUX_RC       = 4'd4;
	localparam STATE_DEMUX_RC_LAST  = 4'd5;

	genvar gen_i;
	integer i;

	reg [STATE_BITS-1:0]   ap_state;
	reg [C_DATA_WIDTH-1:0] rc_tdata;
	reg [KEEP_WIDTH-1:0]   rc_tkeep;
	reg                    rc_tlast;
	reg [C_USER_WIDTH-1:0] rc_tuser;
	reg [RC_CNT_WIDTH-1:0] rc_idx;

	wire m_axis_rx_fire;

	reg  s_axis_rc_tvalid [RC_COUNT-1:0];
	wire s_axis_rc_tready [RC_COUNT-1:0];
	wire s_axis_rc_fire [RC_COUNT-1:0];

	wire [4:0] tlp_hdr_type;

	assign tlp_hdr_type = m_axis_rx_tdata[28:24];

	assign s_axis_cq_tdata = m_axis_rx_tdata;
	assign s_axis_cq_tkeep = m_axis_rx_tkeep;
	assign s_axis_cq_tlast = m_axis_rx_tlast;
	assign s_axis_cq_tvalid = (ap_state == STATE_DEMUX_CQ ? m_axis_rx_tvalid : 1'b0);
	assign s_axis_cq_tuser = m_axis_rx_tuser;

	assign s00_axis_rc_tdata = rc_tdata;
	assign s00_axis_rc_tkeep = rc_tkeep;
	assign s00_axis_rc_tlast = rc_tlast;
	assign s00_axis_rc_tvalid = s_axis_rc_tvalid[0];
	assign s_axis_rc_tready[0] = s00_axis_rc_tready;
	assign s00_axis_rc_tuser = rc_tuser;

	assign s01_axis_rc_tdata = rc_tdata;
	assign s01_axis_rc_tkeep = rc_tkeep;
	assign s01_axis_rc_tlast = rc_tlast;
	assign s01_axis_rc_tvalid = s_axis_rc_tvalid[1];
	assign s_axis_rc_tready[1] = s01_axis_rc_tready;
	assign s01_axis_rc_tuser = rc_tuser;

	// @COMB s_axis_rc_tvalid[i]
	always @(*) begin
		for(i = 0; i < RC_COUNT; i = i + 1) begin
			s_axis_rc_tvalid[i] = 0;
		end

		case(ap_state)
			STATE_DEMUX_RC: begin
				s_axis_rc_tvalid[rc_idx] = m_axis_rx_tvalid;
			end

			STATE_DEMUX_RC_LAST: begin
				s_axis_rc_tvalid[rc_idx] = 1;
			end
		endcase
	end

	generate
		for(gen_i = 0; gen_i < RC_COUNT; gen_i = gen_i + 1) begin
			assign s_axis_rc_fire[gen_i] = s_axis_rc_tready[gen_i] && s_axis_rc_tvalid[gen_i];
		end
	endgenerate

	// @FF ap_state
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= STATE_IDLE;
		end else begin
			case(ap_state)
				STATE_IDLE: begin
					ap_state <= STATE_DEMUX;
				end

				STATE_DEMUX: begin
					if(m_axis_rx_fire) begin
						case(tlp_hdr_type)
							TLP_MEM_TYPE: ap_state <= STATE_DEMUX_CQ;
							TLP_CPL_TYPE: ap_state <= STATE_DEMUX_RC_DW3_2;
						endcase
					end
				end

				STATE_DEMUX_CQ: begin
					if(m_axis_rx_fire && m_axis_rx_tlast) begin
						ap_state <= STATE_DEMUX;
					end
				end

				STATE_DEMUX_RC_DW3_2: begin
					if(m_axis_rx_fire) begin
						if(m_axis_rx_tdata[31:16] == cfg_completer_id && m_axis_rx_tdata[(8+4) +: RC_CNT_WIDTH] < RC_COUNT) begin // cpl_req_id && cpl_req_tag
							ap_state <= STATE_DEMUX_RC;

							if(m_axis_rx_tlast) begin
								ap_state <= STATE_DEMUX_RC_LAST;
							end
						end else begin
							ap_state <= STATE_DEMUX;
						end
					end
				end

				STATE_DEMUX_RC: begin
					if(m_axis_rx_fire && m_axis_rx_tlast) begin
						ap_state <= STATE_DEMUX_RC_LAST;
					end
				end

				STATE_DEMUX_RC_LAST: begin
					if(s_axis_rc_fire[rc_idx]) begin
						ap_state <= STATE_DEMUX;
					end
				end
			endcase
		end
	end

	// @COMB m_axis_rx_tready
	always @(*) begin
		m_axis_rx_tready = 0;

		case(ap_state)
			STATE_DEMUX: begin
				m_axis_rx_tready = 1;
			end

			STATE_DEMUX_CQ: begin
				m_axis_rx_tready = s_axis_cq_tready;
			end

			STATE_DEMUX_RC_DW3_2: begin
				m_axis_rx_tready = 1;
			end

			STATE_DEMUX_RC: begin
				m_axis_rx_tready = s_axis_rc_tready[rc_idx];
			end
		endcase
	end

	assign m_axis_rx_fire = m_axis_rx_tready && m_axis_rx_tvalid;

	// @FF tlp_hdr_dw1_0, @FF rc_idx, @FF rc_tdata, @FF rc_tlast, @FF rc_tlast, @FF rc_tuser
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			tlp_hdr_dw1_0 <= 0;
			rc_idx <= 0;
			rc_tdata <= 0;
			rc_tkeep <= 0;
			rc_tlast <= 0;
			rc_tuser <= 0;
		end else begin
			case(ap_state)
				STATE_DEMUX:
					if(m_axis_rx_fire) begin
						tlp_hdr_dw1_0 <= m_axis_rx_tdata;
					end

				STATE_DEMUX_RC_DW3_2:
					if(m_axis_rx_fire) begin
						rc_idx <= m_axis_rx_tdata[(8+4) +: RC_CNT_WIDTH];
						rc_tdata <= m_axis_rx_tdata;
						rc_tkeep <= m_axis_rx_tkeep;
						rc_tlast <= m_axis_rx_tlast;
						rc_tuser <= m_axis_rx_tuser;
					end

				STATE_DEMUX_RC:
					if(m_axis_rx_fire) begin
						rc_tdata <= m_axis_rx_tdata;
						rc_tkeep <= m_axis_rx_tkeep;
						rc_tlast <= m_axis_rx_tlast;
						rc_tuser <= m_axis_rx_tuser;
					end
			endcase
		end
	end
endmodule
