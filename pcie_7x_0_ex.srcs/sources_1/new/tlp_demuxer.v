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
	output                   m_axis_rx_tready,
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

	// RC02 TLP
	output [C_DATA_WIDTH-1:0] s02_axis_rc_tdata,
	output [KEEP_WIDTH-1:0]   s02_axis_rc_tkeep,
	output                    s02_axis_rc_tlast,
	output                    s02_axis_rc_tvalid,
	input                     s02_axis_rc_tready,
	output [C_USER_WIDTH-1:0] s02_axis_rc_tuser,

	input [15:0]                  cfg_completer_id,
	output reg [C_DATA_WIDTH-1:0] tlp_hdr_dw1_0
);
	localparam SLAVE_COUNT = 3;
	localparam SLAVE_COUNT_WIDTH = $clog2(SLAVE_COUNT + 1);

	// TYPE for CQ
	localparam TLP_MEM_TYPE = 5'b00000;
	localparam TLP_IO_TYPE  = 5'b00010;

	// TYPE for RC
	localparam TLP_CPL_TYPE = 5'b01010;

	localparam STATE_BITS         = 4;
	localparam STATE_IDLE         = 4'd0;
	localparam STATE_DEMUX        = 4'd1;
	localparam STATE_DEMUX_CQ     = 4'd2;
	localparam STATE_DEMUX_RC     = 4'd3;

	reg [STATE_BITS-1:0]        state_reg, state_next;
	reg [SLAVE_COUNT_WIDTH-1:0] slave_index;

	wire m_axis_rx_fire;

	wire s_axis_rc_tvalid [SLAVE_COUNT-1:0];
	wire s_axis_rc_tready [SLAVE_COUNT-1:0];

	wire [4:0]  tlp_hdr_type;
	wire [15:0] tlp_hdr_rid;
	wire [7:0]  tlp_hdr_tag;

	assign m_axis_rx_tready =(
		state_reg == STATE_DEMUX ? 1 :
		state_reg == STATE_DEMUX_CQ ? s_axis_cq_tready :
		state_reg == STATE_DEMUX_RC ? s_axis_rc_tready[slave_index] : 0);
	assign m_axis_rx_fire = m_axis_rx_tready && m_axis_rx_tvalid;

	assign tlp_hdr_type = m_axis_rx_tdata[28:24];
	assign tlp_hdr_rid = m_axis_rx_tdata[63:48];
	assign tlp_hdr_tag = m_axis_rx_tdata[47:40];

	assign s_axis_cq_tdata = m_axis_rx_tdata;
	assign s_axis_cq_tkeep = m_axis_rx_tkeep;
	assign s_axis_cq_tlast = m_axis_rx_tlast;
	assign s_axis_cq_tvalid = (state_reg == STATE_DEMUX_CQ ? m_axis_rx_tvalid : 1'b0);
	assign s_axis_cq_tuser = m_axis_rx_tuser;

	assign s00_axis_rc_tdata = m_axis_rx_tdata;
	assign s00_axis_rc_tkeep = m_axis_rx_tkeep;
	assign s00_axis_rc_tlast = m_axis_rx_tlast;
	assign s00_axis_rc_tvalid = s_axis_rc_tvalid[0];
	assign s_axis_rc_tready[0] = s00_axis_rc_tready;
	assign s00_axis_rc_tuser = m_axis_rx_tuser;

	assign s01_axis_rc_tdata = m_axis_rx_tdata;
	assign s01_axis_rc_tkeep = m_axis_rx_tkeep;
	assign s01_axis_rc_tlast = m_axis_rx_tlast;
	assign s01_axis_rc_tvalid = s_axis_rc_tvalid[1];
	assign s_axis_rc_tready[1] = s01_axis_rc_tready;
	assign s01_axis_rc_tuser = m_axis_rx_tuser;

	assign s02_axis_rc_tdata = m_axis_rx_tdata;
	assign s02_axis_rc_tkeep = m_axis_rx_tkeep;
	assign s02_axis_rc_tlast = m_axis_rx_tlast;
	assign s02_axis_rc_tvalid = s_axis_rc_tvalid[2];
	assign s_axis_rc_tready[2] = s02_axis_rc_tready;
	assign s02_axis_rc_tuser = m_axis_rx_tuser;

	generate
		genvar i;

		for (i = 0; i < SLAVE_COUNT; i = i + 1) begin
			assign s_axis_rc_tvalid[i] = (state_reg == STATE_DEMUX_RC && slave_index == i) ? m_axis_rx_tvalid : 1'b0;
		end
	endgenerate

	always @(*) begin
		state_next = state_reg;

		case(state_reg)
			STATE_IDLE: state_next = STATE_DEMUX;

			STATE_DEMUX:
				if(m_axis_rx_fire) begin
					case(tlp_hdr_type)
						TLP_MEM_TYPE: state_next = STATE_DEMUX_CQ;

						TLP_CPL_TYPE:
							if(tlp_hdr_rid == cfg_completer_id && tlp_hdr_tag < SLAVE_COUNT) begin
								state_next = STATE_DEMUX_RC;
							end
					endcase
				end

			STATE_DEMUX_CQ:
				if(m_axis_rx_fire && m_axis_rx_tlast) begin
					state_next = STATE_DEMUX;
				end

			STATE_DEMUX_RC:
				if(m_axis_rx_fire && m_axis_rx_tlast) begin
					state_next = STATE_DEMUX;
				end
		endcase
	end

	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			state_reg <= STATE_IDLE;
			tlp_hdr_dw1_0 <= 0;
			slave_index <= 0;
		end else begin
			state_reg <= state_next;

			case(state_reg)
				STATE_DEMUX:
					if(m_axis_rx_fire) begin
						tlp_hdr_dw1_0 <= m_axis_rx_tdata;
						slave_index <= tlp_hdr_tag[SLAVE_COUNT_WIDTH-1:0];
					end
			endcase
		end
	end

endmodule
