`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/22/2025 03:05:59 PM
// Design Name: 
// Module Name: tlp_muxer
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

module tlp_muxer #(
	parameter C_DATA_WIDTH = 64,
	parameter C_USER_WIDTH = 4,

	// Do not override parameters below this line
	parameter KEEP_WIDTH = C_DATA_WIDTH / 8
) (
	input clk,
	input rst_n,

	// Input 0
	input [C_DATA_WIDTH-1:0] s00_axis_tdata,
	input [KEEP_WIDTH-1:0]   s00_axis_tkeep,
	input                    s00_axis_tlast,
	input                    s00_axis_tvalid,
	output                   s00_axis_tready,

	// Input 1
	input [C_DATA_WIDTH-1:0] s01_axis_tdata,
	input [KEEP_WIDTH-1:0]   s01_axis_tkeep,
	input                    s01_axis_tlast,
	input                    s01_axis_tvalid,
	output                   s01_axis_tready,

	// Input 2
	input [C_DATA_WIDTH-1:0] s02_axis_tdata,
	input [KEEP_WIDTH-1:0]   s02_axis_tkeep,
	input                    s02_axis_tlast,
	input                    s02_axis_tvalid,
	output                   s02_axis_tready,

	// Input 3
	input [C_DATA_WIDTH-1:0] s03_axis_tdata,
	input [KEEP_WIDTH-1:0]   s03_axis_tkeep,
	input                    s03_axis_tlast,
	input                    s03_axis_tvalid,
	output                   s03_axis_tready,

	// Input 4
	input [C_DATA_WIDTH-1:0] s04_axis_tdata,
	input [KEEP_WIDTH-1:0]   s04_axis_tkeep,
	input                    s04_axis_tlast,
	input                    s04_axis_tvalid,
	output                   s04_axis_tready,

	// Output
	output reg [C_DATA_WIDTH-1:0] m_axis_tdata,
	output reg [KEEP_WIDTH-1:0]   m_axis_tkeep,
	output reg                    m_axis_tlast,
	output reg                    m_axis_tvalid,
	input                         m_axis_tready,
	output [C_USER_WIDTH-1:0]     m_axis_tuser
);
	localparam SRC_COUNT = 5;
	integer i;

	reg [SRC_COUNT-1:0]          slaves_arb_input_unencoded_reg;
	wire                         slaves_arb_output_valid;
	wire [$clog2(SRC_COUNT)-1:0] slaves_arb_output_encoded;
	wire [SRC_COUNT-1:0]         slaves_arb_output_unencoded;

	wire [C_DATA_WIDTH-1:0] s_axis_tdata [SRC_COUNT-1:0];
	wire [KEEP_WIDTH-1:0]   s_axis_tkeep [SRC_COUNT-1:0];
	wire                    s_axis_tlast [SRC_COUNT-1:0];
	wire [SRC_COUNT-1:0]    s_axis_tvalid;
	reg                     s_axis_tready [SRC_COUNT-1:0];
	wire [C_USER_WIDTH-1:0] s_axis_tuser [SRC_COUNT-1:0];

	wire m_axis_fire;

	priority_encoder #(
		.WIDTH(SRC_COUNT),
		.LSB_HIGH_PRIORITY(1)
	) slaves_arb_U(
		.input_unencoded(slaves_arb_input_unencoded_reg),
		.output_valid(slaves_arb_output_valid),
		.output_encoded(slaves_arb_output_encoded),
		.output_unencoded(slaves_arb_output_unencoded)
	);

// ----------------------------------------------------------
// RR group
	assign s_axis_tdata[0] = s00_axis_tdata;
	assign s_axis_tkeep[0] = s00_axis_tkeep;
	assign s_axis_tlast[0] = s00_axis_tlast;
	assign s_axis_tvalid[0] = s00_axis_tvalid;
	assign s00_axis_tready = s_axis_tready[0];

	assign s_axis_tdata[1] = s01_axis_tdata;
	assign s_axis_tkeep[1] = s01_axis_tkeep;
	assign s_axis_tlast[1] = s01_axis_tlast;
	assign s_axis_tvalid[1] = s01_axis_tvalid;
	assign s01_axis_tready = s_axis_tready[1];

	assign s_axis_tdata[2] = s02_axis_tdata;
	assign s_axis_tkeep[2] = s02_axis_tkeep;
	assign s_axis_tlast[2] = s02_axis_tlast;
	assign s_axis_tvalid[2] = s02_axis_tvalid;
	assign s02_axis_tready = s_axis_tready[2];

	assign s_axis_tdata[3] = s03_axis_tdata;
	assign s_axis_tkeep[3] = s03_axis_tkeep;
	assign s_axis_tlast[3] = s03_axis_tlast;
	assign s_axis_tvalid[3] = s03_axis_tvalid;
	assign s03_axis_tready = s_axis_tready[3];

	assign s_axis_tdata[4] = s04_axis_tdata;
	assign s_axis_tkeep[4] = s04_axis_tkeep;
	assign s_axis_tlast[4] = s04_axis_tlast;
	assign s_axis_tvalid[4] = s04_axis_tvalid;
	assign s04_axis_tready = s_axis_tready[4];
// ----------------------------------------------------------

	assign m_axis_tuser[0] = 1'b0; // Unused for V6
	assign m_axis_tuser[1] = 1'b0; // Error forward packet
	assign m_axis_tuser[2] = 1'b0; // Stream packet
	assign m_axis_tuser[3] = 1'b0; // Unused discontinue
	assign m_axis_fire = m_axis_tvalid && m_axis_tready;

	// @COMB s_axis_tready[i]
	always @(*) begin
		for (i = 0; i < SRC_COUNT; i = i + 1) begin
			s_axis_tready[i] = 0;
		end

		if(slaves_arb_output_valid) begin
			s_axis_tready[slaves_arb_output_encoded] = m_axis_tready;
		end
	end

	// @FF slaves_arb_input_unencoded_reg
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			slaves_arb_input_unencoded_reg <= 0;
		end else begin
			if(slaves_arb_output_valid) begin
				if(m_axis_fire && m_axis_tlast) begin
					slaves_arb_input_unencoded_reg <= slaves_arb_input_unencoded_reg & ~slaves_arb_output_unencoded;
				end
			end else begin
				slaves_arb_input_unencoded_reg <= s_axis_tvalid;
			end
		end
	end

	// @COMB m_axis_tdata, @COMB m_axis_tkeep, @COMB m_axis_tlast, @COMB m_axis_tvalid
	always @(*) begin
		if(slaves_arb_output_valid) begin
			m_axis_tdata = s_axis_tdata[slaves_arb_output_encoded];
			m_axis_tkeep = s_axis_tkeep[slaves_arb_output_encoded];
			m_axis_tlast = s_axis_tlast[slaves_arb_output_encoded];
			m_axis_tvalid = s_axis_tvalid[slaves_arb_output_encoded];
		end else begin
			m_axis_tdata = 'hCAFECAFEABCDABCD; // for debug purpose
			m_axis_tkeep = 0;
			m_axis_tlast = 0;
			m_axis_tvalid = 0;
		end
	end
endmodule
