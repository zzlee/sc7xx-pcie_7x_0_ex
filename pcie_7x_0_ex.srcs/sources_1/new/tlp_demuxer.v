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

	localparam SLAVE_COUNT     = 2;
	localparam SLAVE_CNT_WIDTH = $clog2(SLAVE_COUNT + 1);

	localparam STATE_BITS           = 4;
	localparam STATE_IDLE           = 4'd0;
	localparam STATE_DEMUX          = 4'd1;
	localparam STATE_DEMUX_CQ       = 4'd2;
	localparam STATE_DEMUX_RC_DW3_2 = 4'd3;
	localparam STATE_DEMUX_RC       = 4'd4;
	localparam STATE_DEMUX_RC_LAST  = 4'd5;

	reg [STATE_BITS-1:0]       state_reg;
	reg [C_DATA_WIDTH-1:0]     rx_tdata;
	reg [KEEP_WIDTH-1:0]       rx_tkeep;
	reg                        rx_tlast;
	reg [C_USER_WIDTH-1:0]     rx_tuser;
	reg [SLAVE_CNT_WIDTH-1:0]  s_idx;

	wire m_axis_rx_fire;

	wire s_axis_rc_tvalid [SLAVE_COUNT-1:0];
	wire s_axis_rc_tready [SLAVE_COUNT-1:0];
	wire s_axis_rc_fire [SLAVE_COUNT-1:0];

	wire [4:0] tlp_hdr_type;

	assign tlp_hdr_type = m_axis_rx_tdata[28:24];

	assign s_axis_cq_tdata = m_axis_rx_tdata;
	assign s_axis_cq_tkeep = m_axis_rx_tkeep;
	assign s_axis_cq_tlast = m_axis_rx_tlast;
	assign s_axis_cq_tvalid = (state_reg == STATE_DEMUX_CQ ? m_axis_rx_tvalid : 1'b0);
	assign s_axis_cq_tuser = m_axis_rx_tuser;

	assign s00_axis_rc_tdata = rx_tdata;
	assign s00_axis_rc_tkeep = rx_tkeep;
	assign s00_axis_rc_tlast = rx_tlast;
	assign s00_axis_rc_tvalid = s_axis_rc_tvalid[0];
	assign s_axis_rc_tready[0] = s00_axis_rc_tready;
	assign s00_axis_rc_tuser = rx_tuser;

	assign s01_axis_rc_tdata = rx_tdata;
	assign s01_axis_rc_tkeep = rx_tkeep;
	assign s01_axis_rc_tlast = rx_tlast;
	assign s01_axis_rc_tvalid = s_axis_rc_tvalid[1];
	assign s_axis_rc_tready[1] = s01_axis_rc_tready;
	assign s01_axis_rc_tuser = rx_tuser;

	generate
		genvar i;

		for (i = 0; i < SLAVE_COUNT; i = i + 1) begin
			assign s_axis_rc_tvalid[i] =
				(state_reg == STATE_DEMUX_RC && s_idx == i)      ? m_axis_rx_tvalid :
				(state_reg == STATE_DEMUX_RC_LAST && s_idx == i) ? 1'b1             : 1'b0;
			assign s_axis_rc_fire[i] = s_axis_rc_tready[i] && s_axis_rc_tvalid[i];
		end
	endgenerate

	// @FF state_reg
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			state_reg <= STATE_IDLE;
		end else begin
			case(state_reg)
				STATE_IDLE: begin
					state_reg <= STATE_DEMUX;
				end

				STATE_DEMUX: begin
					if(m_axis_rx_fire) begin
						case(tlp_hdr_type)
							TLP_MEM_TYPE: state_reg <= STATE_DEMUX_CQ;
							TLP_CPL_TYPE: state_reg <= STATE_DEMUX_RC_DW3_2;
						endcase
					end
				end

				STATE_DEMUX_CQ: begin
					if(m_axis_rx_fire && m_axis_rx_tlast) begin
						state_reg <= STATE_DEMUX;
					end
				end

				STATE_DEMUX_RC_DW3_2: begin
					if(m_axis_rx_fire) begin
						if(m_axis_rx_tdata[31:16] == cfg_completer_id && m_axis_rx_tdata[15:8+4] < SLAVE_COUNT) begin // cpl_req_id && cpl_req_tag
							state_reg <= STATE_DEMUX_RC;

							if(m_axis_rx_tlast) begin
								state_reg <= STATE_DEMUX_RC_LAST;
							end
						end else begin
							state_reg <= STATE_DEMUX;
						end
					end
				end

				STATE_DEMUX_RC: begin
					if(m_axis_rx_fire && m_axis_rx_tlast) begin
						state_reg <= STATE_DEMUX_RC_LAST;
					end
				end

				STATE_DEMUX_RC_LAST: begin
					if(s_axis_rc_fire[s_idx]) begin
						state_reg <= STATE_DEMUX;
					end
				end
			endcase
		end
	end

	// @COMB m_axis_rx_tready
	always @(*) begin
		m_axis_rx_tready = 0;

		case(state_reg)
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
				m_axis_rx_tready = s_axis_rc_tready[s_idx];
			end
		endcase
	end

	assign m_axis_rx_fire = m_axis_rx_tready && m_axis_rx_tvalid;

	// @FF tlp_hdr_dw1_0, @FF s_idx, @FF rx_tdata, @FF rx_tlast, @FF rx_tlast, @FF rx_tuser
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			tlp_hdr_dw1_0 <= 0;
			s_idx <= 0;
			rx_tdata <= 0;
			rx_tkeep <= 0;
			rx_tlast <= 0;
			rx_tuser <= 0;
		end else begin
			case(state_reg)
				STATE_DEMUX:
					if(m_axis_rx_fire) begin
						tlp_hdr_dw1_0 <= m_axis_rx_tdata;
					end

				STATE_DEMUX_RC_DW3_2:
					if(m_axis_rx_fire) begin
						s_idx <= m_axis_rx_tdata[15:8+4];
						rx_tdata <= m_axis_rx_tdata;
						rx_tkeep <= m_axis_rx_tkeep;
						rx_tlast <= m_axis_rx_tlast;
						rx_tuser <= m_axis_rx_tuser;
					end

				STATE_DEMUX_RC:
					if(m_axis_rx_fire) begin
						rx_tdata <= m_axis_rx_tdata;
						rx_tkeep <= m_axis_rx_tkeep;
						rx_tlast <= m_axis_rx_tlast;
						rx_tuser <= m_axis_rx_tuser;
					end
			endcase
		end
	end
endmodule
