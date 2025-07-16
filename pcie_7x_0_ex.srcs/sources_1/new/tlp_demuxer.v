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
	input [C_DATA_WIDTH-1:0] s_axis_rx_tdata,
	input [KEEP_WIDTH-1:0]   s_axis_rx_tkeep,
	input                    s_axis_rx_tlast,
	input                    s_axis_rx_tvalid,
	output reg               s_axis_rx_tready,
	input [C_USER_WIDTH-1:0] s_axis_rx_tuser,

	// CQ TLP
	output [C_DATA_WIDTH-1:0] m_axis_cq_tdata,
	output [KEEP_WIDTH-1:0]   m_axis_cq_tkeep,
	output                    m_axis_cq_tlast,
	output                    m_axis_cq_tvalid,
	input                     m_axis_cq_tready,
	output [C_USER_WIDTH-1:0] m_axis_cq_tuser,

	// RC00 TLP
	output [C_DATA_WIDTH-1:0] m00_axis_rc_tdata,
	output [KEEP_WIDTH-1:0]   m00_axis_rc_tkeep,
	output                    m00_axis_rc_tlast,
	output                    m00_axis_rc_tvalid,
	input                     m00_axis_rc_tready,
	output [C_USER_WIDTH-1:0] m00_axis_rc_tuser,

	// RC01 TLP
	output [C_DATA_WIDTH-1:0] m01_axis_rc_tdata,
	output [KEEP_WIDTH-1:0]   m01_axis_rc_tkeep,
	output                    m01_axis_rc_tlast,
	output                    m01_axis_rc_tvalid,
	input                     m01_axis_rc_tready,
	output [C_USER_WIDTH-1:0] m01_axis_rc_tuser,

	// RC00 DESC TLP
	output [C_DATA_WIDTH-1:0] m00_axis_rc_desc_tdata,
	output [KEEP_WIDTH-1:0]   m00_axis_rc_desc_tkeep,
	output                    m00_axis_rc_desc_tlast,
	output                    m00_axis_rc_desc_tvalid,
	input                     m00_axis_rc_desc_tready,
	output [C_USER_WIDTH-1:0] m00_axis_rc_desc_tuser,

	// RC01 DESC TLP
	output [C_DATA_WIDTH-1:0] m01_axis_rc_desc_tdata,
	output [KEEP_WIDTH-1:0]   m01_axis_rc_desc_tkeep,
	output                    m01_axis_rc_desc_tlast,
	output                    m01_axis_rc_desc_tvalid,
	input                     m01_axis_rc_desc_tready,
	output [C_USER_WIDTH-1:0] m01_axis_rc_desc_tuser,

	input [15:0]                  cfg_completer_id,
	output reg [C_DATA_WIDTH-1:0] tlp_hdr_dw1_0
);
	localparam TLP_MEM_TYPE = 5'b00000;
	localparam TLP_CPL_TYPE = 5'b01010;

	localparam RC_GRP_COUNT     = 2;
	localparam RC_GRP_CNT_WIDTH = $clog2(RC_GRP_COUNT + 1);

	localparam RC_DESC_GRP           = 4'b1111;
	localparam RC_DESC_GRP_COUNT     = 2;
	localparam RC_DESC_GRP_CNT_WIDTH = $clog2(RC_DESC_GRP_COUNT + 1);

	localparam STATE_IDLE               = 3'd0;
	localparam STATE_DEMUX              = 3'd1;
	localparam STATE_DEMUX_CQ           = 3'd2;
	localparam STATE_DEMUX_RC_DW3_2     = 3'd3;
	localparam STATE_DEMUX_RC           = 3'd4;
	localparam STATE_DEMUX_RC_LAST      = 3'd5;
	localparam STATE_DEMUX_RC_DESC      = 3'd6;
	localparam STATE_DEMUX_RC_DESC_LAST = 3'd7;
	localparam STATE_BITS               = 3;

	genvar gen_i;
	integer i;

	reg [STATE_BITS-1:0]            ap_state;
	reg [C_DATA_WIDTH-1:0]          rc_tdata;
	reg [KEEP_WIDTH-1:0]            rc_tkeep;
	reg                             rc_tlast;
	reg [C_USER_WIDTH-1:0]          rc_tuser;
	reg [RC_GRP_CNT_WIDTH-1:0]      rc_grp_idx;
	reg [RC_DESC_GRP_CNT_WIDTH-1:0] rc_desc_idx;

	wire s_axis_rx_fire;

	reg  m_axis_rc_tvalid [RC_GRP_COUNT-1:0];
	wire m_axis_rc_tready [RC_GRP_COUNT-1:0];
	wire m_axis_rc_fire [RC_GRP_COUNT-1:0];

	reg  m_axis_rc_desc_tvalid [RC_DESC_GRP_COUNT-1:0];
	wire m_axis_rc_desc_tready [RC_DESC_GRP_COUNT-1:0];
	wire m_axis_rc_desc_fire [RC_DESC_GRP_COUNT-1:0];

	wire [4:0] tlp_hdr_type;

	assign tlp_hdr_type = s_axis_rx_tdata[28:24];

	assign m_axis_cq_tdata = s_axis_rx_tdata;
	assign m_axis_cq_tkeep = s_axis_rx_tkeep;
	assign m_axis_cq_tlast = s_axis_rx_tlast;
	assign m_axis_cq_tvalid = (ap_state == STATE_DEMUX_CQ ? s_axis_rx_tvalid : 1'b0);
	assign m_axis_cq_tuser = s_axis_rx_tuser;

// ---------------------------------------------------
// ---- RC
	assign m00_axis_rc_tdata = rc_tdata;
	assign m00_axis_rc_tkeep = rc_tkeep;
	assign m00_axis_rc_tlast = rc_tlast;
	assign m00_axis_rc_tvalid = m_axis_rc_tvalid[0];
	assign m_axis_rc_tready[0] = m00_axis_rc_tready;
	assign m00_axis_rc_tuser = rc_tuser;

	assign m01_axis_rc_tdata = rc_tdata;
	assign m01_axis_rc_tkeep = rc_tkeep;
	assign m01_axis_rc_tlast = rc_tlast;
	assign m01_axis_rc_tvalid = m_axis_rc_tvalid[1];
	assign m_axis_rc_tready[1] = m01_axis_rc_tready;
	assign m01_axis_rc_tuser = rc_tuser;

	generate
		for(gen_i = 0; gen_i < RC_GRP_COUNT; gen_i = gen_i + 1) begin
			assign m_axis_rc_fire[gen_i] = m_axis_rc_tready[gen_i] && m_axis_rc_tvalid[gen_i];
		end
	endgenerate

	// @COMB m_axis_rc_tvalid[i]
	always @(*) begin
		for(i = 0; i < RC_GRP_COUNT; i = i + 1) begin
			m_axis_rc_tvalid[i] = 0;
		end

		case(ap_state)
			STATE_DEMUX_RC: begin
				m_axis_rc_tvalid[rc_grp_idx] = s_axis_rx_tvalid;
			end

			STATE_DEMUX_RC_LAST: begin
				m_axis_rc_tvalid[rc_grp_idx] = 1;
			end
		endcase
	end

// ---------------------------------------------------
// ---- RC DESC
	assign m00_axis_rc_desc_tdata = rc_tdata;
	assign m00_axis_rc_desc_tkeep = rc_tkeep;
	assign m00_axis_rc_desc_tlast = rc_tlast;
	assign m00_axis_rc_desc_tvalid = m_axis_rc_desc_tvalid[0];
	assign m_axis_rc_desc_tready[0] = m00_axis_rc_desc_tready;
	assign m00_axis_rc_desc_tuser = rc_tuser;

	assign m01_axis_rc_desc_tdata = rc_tdata;
	assign m01_axis_rc_desc_tkeep = rc_tkeep;
	assign m01_axis_rc_desc_tlast = rc_tlast;
	assign m01_axis_rc_desc_tvalid = m_axis_rc_desc_tvalid[1];
	assign m_axis_rc_desc_tready[1] = m01_axis_rc_desc_tready;
	assign m01_axis_rc_desc_tuser = rc_tuser;

	generate
		for(gen_i = 0; gen_i < RC_DESC_GRP_COUNT; gen_i = gen_i + 1) begin
			assign m_axis_rc_desc_fire[gen_i] = m_axis_rc_desc_tready[gen_i] && m_axis_rc_desc_tvalid[gen_i];
		end
	endgenerate

	// @COMB m_axis_rc_desc_tvalid[i]
	always @(*) begin
		for(i = 0; i < RC_DESC_GRP_COUNT; i = i + 1) begin
			m_axis_rc_desc_tvalid[i] = 0;
		end

		case(ap_state)
			STATE_DEMUX_RC_DESC: begin
				m_axis_rc_desc_tvalid[rc_desc_idx] = s_axis_rx_tvalid;
			end

			STATE_DEMUX_RC_DESC_LAST: begin
				m_axis_rc_desc_tvalid[rc_desc_idx] = 1;
			end
		endcase
	end
// ---------------------------------------------------

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
					if(s_axis_rx_fire) begin
						case(tlp_hdr_type)
							TLP_MEM_TYPE: ap_state <= STATE_DEMUX_CQ;
							TLP_CPL_TYPE: ap_state <= STATE_DEMUX_RC_DW3_2;
						endcase
					end
				end

				STATE_DEMUX_CQ: begin
					if(s_axis_rx_fire && s_axis_rx_tlast) begin
						ap_state <= STATE_DEMUX;
					end
				end

				STATE_DEMUX_RC_DW3_2: begin
					if(s_axis_rx_fire) begin
						if(s_axis_rx_tdata[31:16] == cfg_completer_id) begin
							if(s_axis_rx_tdata[(8+4) +: 4] == RC_DESC_GRP) begin
								ap_state <= STATE_DEMUX_RC_DESC;

								if(s_axis_rx_tlast) begin
									ap_state <= STATE_DEMUX_RC_DESC_LAST;
								end
							end else if(s_axis_rx_tdata[(8+4) +: RC_GRP_CNT_WIDTH] < RC_GRP_COUNT) begin
								ap_state <= STATE_DEMUX_RC;

								if(s_axis_rx_tlast) begin
									ap_state <= STATE_DEMUX_RC_LAST;
								end
							end else begin
								ap_state <= STATE_DEMUX;
							end
						end else begin
							ap_state <= STATE_DEMUX;
						end
					end
				end

				STATE_DEMUX_RC: begin
					if(s_axis_rx_fire && s_axis_rx_tlast) begin
						ap_state <= STATE_DEMUX_RC_LAST;
					end
				end

				STATE_DEMUX_RC_LAST: begin
					if(m_axis_rc_fire[rc_grp_idx]) begin
						ap_state <= STATE_DEMUX;
					end
				end

				STATE_DEMUX_RC_DESC: begin
					if(s_axis_rx_fire && s_axis_rx_tlast) begin
						ap_state <= STATE_DEMUX_RC_DESC_LAST;
					end
				end

				STATE_DEMUX_RC_DESC_LAST: begin
					if(m_axis_rc_desc_fire[rc_desc_idx]) begin
						ap_state <= STATE_DEMUX;
					end
				end
			endcase
		end
	end

	// @COMB s_axis_rx_tready
	always @(*) begin
		s_axis_rx_tready = 0;

		case(ap_state)
			STATE_DEMUX: begin
				s_axis_rx_tready = 1;
			end

			STATE_DEMUX_CQ: begin
				s_axis_rx_tready = m_axis_cq_tready;
			end

			STATE_DEMUX_RC_DW3_2: begin
				s_axis_rx_tready = 1;
			end

			STATE_DEMUX_RC: begin
				s_axis_rx_tready = m_axis_rc_tready[rc_grp_idx];
			end

			STATE_DEMUX_RC_DESC: begin
				s_axis_rx_tready = m_axis_rc_desc_tready[rc_desc_idx];
			end
		endcase
	end

	assign s_axis_rx_fire = s_axis_rx_tready && s_axis_rx_tvalid;

	// @FF tlp_hdr_dw1_0, @FF rc_grp, @FF rc_tdata, @FF rc_tlast, @FF rc_tlast, @FF rc_tuser
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			tlp_hdr_dw1_0 <= 0;
			rc_grp_idx <= 0;
			rc_desc_idx <= 0;
			rc_tdata <= 0;
			rc_tkeep <= 0;
			rc_tlast <= 0;
			rc_tuser <= 0;
		end else begin
			case(ap_state)
				STATE_DEMUX:
					if(s_axis_rx_fire) begin
						tlp_hdr_dw1_0 <= s_axis_rx_tdata;
					end

				STATE_DEMUX_RC_DW3_2:
					if(s_axis_rx_fire) begin
						rc_grp_idx <= s_axis_rx_tdata[(8+4) +: RC_GRP_CNT_WIDTH];
						rc_desc_idx <= s_axis_rx_tdata[8 +: RC_DESC_GRP_CNT_WIDTH];
						rc_tdata <= s_axis_rx_tdata;
						rc_tkeep <= s_axis_rx_tkeep;
						rc_tlast <= s_axis_rx_tlast;
						rc_tuser <= s_axis_rx_tuser;
					end

				STATE_DEMUX_RC, STATE_DEMUX_RC_DESC:
					if(s_axis_rx_fire) begin
						rc_tdata <= s_axis_rx_tdata;
						rc_tkeep <= s_axis_rx_tkeep;
						rc_tlast <= s_axis_rx_tlast;
						rc_tuser <= s_axis_rx_tuser;
					end
			endcase
		end
	end
endmodule
