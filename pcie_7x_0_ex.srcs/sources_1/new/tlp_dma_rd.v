`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/21/2025 04:36:29 PM
// Design Name: 
// Module Name: tlp_dma_rd
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


module tlp_dma_rd #(
	parameter C_DATA_WIDTH = 64,
	parameter C_RC_USER_WIDTH = 22,
	parameter C_RR_USER_WIDTH = 4,
	parameter C_SINK_DATA_WIDTH = 64,

	// Do not override parameters below this line
	parameter KEEP_WIDTH = C_DATA_WIDTH / 8,
	parameter SINK_KEEP_WIDTH = C_SINK_DATA_WIDTH / 8
) (
	input clk,
	input rst_n,

	// RC TLP
	input [C_DATA_WIDTH-1:0]    m_axis_rc_tdata,
	input [KEEP_WIDTH-1:0]      m_axis_rc_tkeep,
	input                       m_axis_rc_tlast,
	input                       m_axis_rc_tvalid,
	output                      m_axis_rc_tready,
	input [C_RC_USER_WIDTH-1:0] m_axis_rc_tuser,

	// RR TLP
	output [C_DATA_WIDTH-1:0]     s_axis_rr_tdata,
	output [KEEP_WIDTH-1:0]       s_axis_rr_tkeep,
	output                        s_axis_rr_tlast,
	output                        s_axis_rr_tvalid,
	input                         s_axis_rr_tready,
	output [C_RR_USER_WIDTH-1:0]  s_axis_rr_tuser,

	// S00 SINK
	output [C_SINK_DATA_WIDTH-1:0] s00_axis_tdata,
	output [SINK_KEEP_WIDTH-1:0]   s00_axis_tkeep,
	output                         s00_axis_tlast,
	output                         s00_axis_tvalid,
	input                          s00_axis_tready,
	output                         s00_axis_tuser,

	input [15:0]             cfg_completer_id,
	input [C_DATA_WIDTH-1:0] tlp_hdr_dw1_0,

	// ap params
	input [63:0] buf_addr,
	input [31:0] size,
	input [31:0] times,

	// ap ctrl
	input  ap_start,
	output ap_done,
	output ap_ready,
	output ap_idle
);

	assign m_axis_rc_tready = 0;
	assign s_axis_rr_tvalid = 0;
	assign s00_axis_tvalid = 0;
	assign ap_done = 0;
	assign ap_ready = 0;
	assign ap_idle = 1;

endmodule
