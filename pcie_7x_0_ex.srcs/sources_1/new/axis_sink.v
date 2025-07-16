`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/16/2025 09:45:59 AM
// Design Name: 
// Module Name: axis_sink
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


module axis_sink # (
	parameter C_DATA_WIDTH = 32,

	// const parameters
	parameter KEEP_WIDTH = ((C_DATA_WIDTH+7)/8)
) (
	input clk,
	input rst_n,

	// Flow Control
	input  ap_start,
	output ap_idle,
	output ap_ready,
	output ap_done,

	// AXI Stream Interface (Input)
	input [C_DATA_WIDTH-1:0] s_axis_tdata,
	input [KEEP_WIDTH-1:0]   s_axis_tkeep,
	input                    s_axis_tlast,
	input                    s_axis_tuser,
	input                    s_axis_tvalid,
	output                   s_axis_tready
);

	localparam STATE_IDLE   = 2'd0;
	localparam STATE_START  = 2'd1;
	localparam STATE_FINISH = 2'd2;
	localparam STATE_BITS   = 2;

	wire t_fire;

	reg [STATE_BITS-1:0] state_reg;

	assign ap_idle = (state_reg == STATE_IDLE);
	assign ap_done = (state_reg == STATE_FINISH);
	assign ap_ready = ap_done;

	assign t_fire = s_axis_tvalid && s_axis_tready;
	assign s_axis_tready = (state_reg == STATE_START);

	always @(posedge clk or negedge rst_n) begin
		if(! rst_n) begin
			state_reg <= STATE_IDLE;
		end else begin
			case(state_reg)
				STATE_IDLE: begin
					if(ap_start) begin
						state_reg <= STATE_START;
					end
				end
			endcase
		end
	end

	always @(*) begin
		case(state_reg)
			STATE_START: begin
				if(t_fire) begin
					$display("t_fire: data='h%X keep='h%X last=%d user=%d", s_axis_tdata, s_axis_tkeep, s_axis_tlast, s_axis_tuser);
				end
			end
		endcase
	end
endmodule
