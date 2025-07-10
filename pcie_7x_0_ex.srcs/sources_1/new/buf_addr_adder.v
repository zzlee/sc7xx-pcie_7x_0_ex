`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/10/2025 08:50:32 AM
// Design Name: 
// Module Name: buf_addr_adder
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

module buf_addr_adder #(
	parameter C_ADDR_WIDTH = 64,
	parameter C_INC_WIDTH = 32
) (
	input clk,
	input rst_n,

	input [C_ADDR_WIDTH-1:0]      buf_addr,
	input [C_INC_WIDTH-1:0]       inc,
	output reg [C_ADDR_WIDTH-1:0] buf_addr_next
);
	reg carry;

	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			buf_addr_next <= 0;
			carry <= 0;
		end else begin
			{carry, buf_addr_next[31:0]} <= buf_addr[31:0] + inc;
			buf_addr_next[C_ADDR_WIDTH-1:32] <= buf_addr[C_ADDR_WIDTH-1:32] + carry;
		end
	end
endmodule