`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/08/2025 07:55:32 AM
// Design Name: 
// Module Name: axis_src
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

module axis_src #(
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

	// AXI Stream Interface (Output)
	output [C_DATA_WIDTH-1:0] m_axis_tdata,
	output [KEEP_WIDTH-1:0]   m_axis_tkeep,
	output                    m_axis_tlast,
	output reg                m_axis_tuser,
	output                    m_axis_tvalid,
	input                     m_axis_tready
);

	localparam STATE_IDLE   = 2'd0;
	localparam STATE_START  = 2'd1;
	localparam STATE_FINISH = 2'd2;
	localparam STATE_BITS   = 2;

	wire t_fire;

	reg [STATE_BITS-1:0]    state_reg;
	reg [7:0]               data_seed_reg;
	wire [C_DATA_WIDTH-1:0] generated_data;

	assign ap_idle = (state_reg == STATE_IDLE);
	assign ap_done = (state_reg == STATE_FINISH);
	assign ap_ready = 0;

	// generated data
	generate
		genvar i;

		for (i = 0; i < C_DATA_WIDTH / 8; i = i + 1) begin
			assign generated_data[8*(i+1)-1:8*i] = data_seed_reg + i;
		end
	endgenerate

	assign t_fire = m_axis_tvalid && m_axis_tready;

	assign m_axis_tkeep = {(KEEP_WIDTH){1'b1}};
	assign m_axis_tdata = generated_data;
	assign m_axis_tlast = 0;
	assign m_axis_tvalid = (state_reg == STATE_START);

	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			state_reg <= STATE_IDLE;
		end else begin
			case(state_reg)
				STATE_IDLE: begin
					if(ap_start) begin
						state_reg <= STATE_START;
					end
				end

				STATE_FINISH: begin
					state_reg <= STATE_IDLE;
				end
			endcase
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			data_seed_reg <= 0;
			m_axis_tuser <= 1;
		end else begin
			case(state_reg)
				STATE_START: begin
					if(t_fire) begin
						data_seed_reg <= data_seed_reg + 1;
						m_axis_tuser <= 0;
					end
				end
			endcase
		end
	end

	always @(*) begin
		case(state_reg)
			STATE_START: begin
				if(t_fire) begin
					$display("t_fire: data='h%X keep='h%X last=%d user=%d",
						m_axis_tdata, m_axis_tkeep, m_axis_tlast, m_axis_tuser);
				end
			end
		endcase
	end
endmodule
