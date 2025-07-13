`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/11/2025 09:29:32 AM
// Design Name: 
// Module Name: axis_split_2
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

module axis_split_2 #(
	parameter C_SRC_DATA_WIDTH = 64,
	parameter C_DST_DATA_WIDTH = 32,

	// const parameters
	parameter SRC_KEEP_WIDTH = ((C_SRC_DATA_WIDTH+7)/8),
	parameter DST_KEEP_WIDTH = ((C_DST_DATA_WIDTH+7)/8)
) (
	input clk,
	input rst_n,

	// AXIS input
	input [C_SRC_DATA_WIDTH-1:0] s_axis_tdata,
	input [SRC_KEEP_WIDTH-1:0]   s_axis_tkeep,
	input                        s_axis_tlast,
	input                        s_axis_tuser,
	input                        s_axis_tvalid,
	output                       s_axis_tready,

	// AXIS output 0
	output [C_DST_DATA_WIDTH-1:0] m00_axis_tdata,
	output [DST_KEEP_WIDTH-1:0]   m00_axis_tkeep,
	output                        m00_axis_tlast,
	output                        m00_axis_tuser,
	output                        m00_axis_tvalid,
	input                         m00_axis_tready,

	// AXIS output 1
	output [C_DST_DATA_WIDTH-1:0] m01_axis_tdata,
	output [DST_KEEP_WIDTH-1:0]   m01_axis_tkeep,
	output                        m01_axis_tlast,
	output                        m01_axis_tuser,
	output                        m01_axis_tvalid,
	input                         m01_axis_tready
);
	localparam DST_COUNT = 2;

    wire [C_SRC_DATA_WIDTH-1:0] m_axis_tdata [DST_COUNT-1:0];
    wire [SRC_KEEP_WIDTH-1:0]   m_axis_tkeep [DST_COUNT-1:0];
    wire                        m_axis_tvalid [DST_COUNT-1:0];
    wire                        m_axis_tready [DST_COUNT-1:0];
    wire                        m_axis_tlast [DST_COUNT-1:0];
    wire                        m_axis_tuser [DST_COUNT-1:0];

	axis_broadcast #(
		.M_COUNT(2),
		.DATA_WIDTH(C_SRC_DATA_WIDTH),
		.LAST_ENABLE(1),
		.USER_ENABLE(1),
		.USER_WIDTH(1)
	) axis_broadcast_U(
    	.clk(clk),
    	.rst(~rst_n),

      	.s_axis_tdata(s_axis_tdata),
      	.s_axis_tkeep(s_axis_tkeep),
      	.s_axis_tvalid(s_axis_tvalid),
    	.s_axis_tready(s_axis_tready),
      	.s_axis_tlast(s_axis_tlast),
      	.s_axis_tuser(s_axis_tuser),

    	.m_axis_tdata({m_axis_tdata[1], m_axis_tdata[0]}),
    	.m_axis_tkeep({m_axis_tkeep[1], m_axis_tkeep[0]}),
    	.m_axis_tvalid({m_axis_tvalid[1], m_axis_tvalid[0]}),
    	.m_axis_tready({m_axis_tready[1], m_axis_tready[0]}),
    	.m_axis_tlast({m_axis_tlast[1], m_axis_tlast[0]}),
    	.m_axis_tuser({m_axis_tuser[1], m_axis_tuser[0]})
	);

	assign m00_axis_tdata = m_axis_tdata[0][0*C_DST_DATA_WIDTH +: C_DST_DATA_WIDTH];
	assign m00_axis_tkeep = m_axis_tkeep[0][0*DST_KEEP_WIDTH +: DST_KEEP_WIDTH];
	assign m00_axis_tlast = m_axis_tlast[0];
	assign m00_axis_tuser = m_axis_tuser[0];
	assign m00_axis_tvalid = m_axis_tvalid[0];
	assign m_axis_tready[0] = m00_axis_tready;

	assign m01_axis_tdata = m_axis_tdata[1][1*C_DST_DATA_WIDTH +: C_DST_DATA_WIDTH];
	assign m01_axis_tkeep = m_axis_tkeep[1][1*DST_KEEP_WIDTH +: DST_KEEP_WIDTH];
	assign m01_axis_tlast = m_axis_tlast[1];
	assign m01_axis_tuser = m_axis_tuser[1];
	assign m01_axis_tvalid = m_axis_tvalid[1];
	assign m_axis_tready[1] = m01_axis_tready;

endmodule
