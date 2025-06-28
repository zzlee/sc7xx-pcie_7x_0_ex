`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/24/2025 09:39:04 AM
// Design Name: 
// Module Name: pcie_intr_ctrl
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


module pcie_intr_ctrl #(
	parameter INTR_WIDTH = 4
) (
	input clk,
	input rst_n,

	input [INTR_WIDTH-1:0] intr,

	output        cfg_interrupt,
	input         cfg_interrupt_rdy,
	output [7:0]  cfg_interrupt_di
);
	localparam STATE_BITS = 2;
	localparam STATE_IDLE = 2'd0;
	localparam STATE_INTR = 2'd1;

	reg [STATE_BITS-1:0] state_reg;
	reg [INTR_WIDTH-1:0] last_intr;

	reg [INTR_WIDTH-1:0]          intr_arb_input_unencoded_reg;
	wire                          intr_arb_output_valid;
	wire [$clog2(INTR_WIDTH)-1:0] intr_arb_output_encoded;
	wire [INTR_WIDTH-1:0]         intr_arb_output_unencoded;

	wire cfg_interrupt_fire;

	assign cfg_interrupt = (state_reg == STATE_INTR && intr_arb_output_valid);
	assign cfg_interrupt_di = 8'd0; // TODO, intr_arb_output_encoded?!
	assign cfg_interrupt_fire = cfg_interrupt && cfg_interrupt_rdy;

	priority_encoder #(
		.WIDTH(INTR_WIDTH)
	) intr_arb_U(
		.input_unencoded(intr_arb_input_unencoded_reg),
		.output_valid(intr_arb_output_valid),
		.output_encoded(intr_arb_output_encoded),
		.output_unencoded(intr_arb_output_unencoded)
	);

	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			state_reg <= STATE_IDLE;
			intr_arb_input_unencoded_reg <= 0;
			last_intr <= 0;
		end else begin
			case(state_reg)
				STATE_IDLE:
					if(last_intr != intr) begin
						last_intr <= intr;
						intr_arb_input_unencoded_reg <= intr;
						state_reg <= STATE_INTR;
					end

				STATE_INTR:
					if(intr_arb_output_valid) begin
						if(cfg_interrupt_fire)
							intr_arb_input_unencoded_reg <= intr_arb_input_unencoded_reg & ~intr_arb_output_unencoded;
					end else begin
						state_reg <= STATE_IDLE;
					end
			endcase
		end
	end
endmodule
