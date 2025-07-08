`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/23/2025 03:08:21 PM
// Design Name: 
// Module Name: pcie_turnoff
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


module pcie_turnoff(
	input clk,
	input rst_n,

	input      cfg_to_turnoff,
	output reg cfg_turnoff_ok
);
	reg cfg_to_turnoff_q;

	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			cfg_turnoff_ok <= 0;
			cfg_to_turnoff_q <= 0;
		end else begin
			cfg_to_turnoff_q <= cfg_to_turnoff;
			cfg_turnoff_ok <= cfg_to_turnoff_q;
		end
	end
endmodule
