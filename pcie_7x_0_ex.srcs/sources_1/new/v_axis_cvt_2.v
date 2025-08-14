`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 08/13/2025 09:30:49 AM
// Design Name:
// Module Name: v_axis_cvt_2
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

module v_axis_cvt_2 #(
	parameter C_COMP_PER_PIXEL = 3,
	parameter C_BITS_PER_COMP = 8,
	parameter C_PIXELS_PER_BEAT = 4,

	// Do not override parameters below this line
	parameter BITS_PER_PIXEL = C_BITS_PER_COMP * C_COMP_PER_PIXEL,
	parameter TDATA_IN_WIDTH = $floor(((BITS_PER_PIXEL * C_PIXELS_PER_BEAT) + 7) / 8) * 8,
	parameter TDATA_OUT_WIDTH = $floor(((C_BITS_PER_COMP * C_PIXELS_PER_BEAT) + 7) / 8) * 8
) (
	input clk,
	input rst_n,

	// Flow Control
	input  ap_start,
	output ap_idle,
	output ap_ready,
	output ap_done,

	// Configuration
	input [31:0]      WIDTH,
	input [31:0]      HEIGHT,
	input [3:0]       COLOR_FORMAT_IN,
	input [3:0]       COLOR_FORMAT_OUT,
	output reg [31:0] ERR_MASK,

	// AXI Stream Interface (Input)
	input [TDATA_IN_WIDTH-1:0] s_axis_tdata,
	input                      s_axis_tlast,
	input                      s_axis_tuser,
	input                      s_axis_tvalid,
	output reg                 s_axis_tready,

	// AXI Stream Interface (Output)
	output reg [TDATA_OUT_WIDTH-1:0] m00_axis_tdata,
	output reg                       m00_axis_tlast,
	output reg                       m00_axis_tuser,
	output reg                       m00_axis_tvalid,
	input                            m00_axis_tready,

	output reg [TDATA_OUT_WIDTH-1:0] m01_axis_tdata,
	output reg                       m01_axis_tlast,
	output reg                       m01_axis_tuser,
	output reg                       m01_axis_tvalid,
	input                            m01_axis_tready
);
	`include "err_mask.vh"
	`include "color_format.vh"

	localparam AP_STATE_IDLE   = 2'd0;
	localparam AP_STATE_FINISH = 2'd1;
	localparam AP_STATE_RUN    = 2'd2;
	localparam AP_STATE_ERROR  = 2'd3;
	localparam AP_STATE_WIDTH  = 2;

	integer i, j;

	reg [AP_STATE_WIDTH-1:0] ap_state;
	reg [31:0]               ERR_MASK_next;

	assign ap_idle = (ap_state == AP_STATE_IDLE);
	assign ap_done = (ap_state == AP_STATE_FINISH);
	assign ap_ready = ap_done;

	// @FF ap_state
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_IDLE;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					if(ap_start) begin
						ap_state <= AP_STATE_RUN;
					end
				end

				AP_STATE_RUN: begin
				end

				AP_STATE_ERROR: begin
				end
			endcase

			if(|ERR_MASK) begin
				ap_state <= AP_STATE_ERROR;
			end
		end
	end

	// @COMB s_axis_tready, @COMB m00_axis_tvalid, @COMB m01_axis_tvalid
	always @(*) begin
		s_axis_tready = 0;
		m00_axis_tvalid = 0;
		m01_axis_tvalid = 0;

		case(ap_state)
			AP_STATE_RUN: begin
				if(COLOR_FORMAT_IN == COLOR_FORMAT_YUV444) begin
					s_axis_tready = m00_axis_tready && m01_axis_tready;
					m00_axis_tvalid = s_axis_tready;
					m01_axis_tvalid = s_axis_tready;
				end
			end
		endcase
	end

	// @COMB m00_axis_tdata, @COMB m00_axis_tlast, @COMB m00_axis_tuser,
	// @COMB m01_axis_tdata, @COMB m01_axis_tlast, @COMB m01_axis_tuser
	always @(*) begin
		m00_axis_tdata = 0;
		m01_axis_tdata = 0;
		m00_axis_tlast = 0;
		m01_axis_tlast = 0;
		m00_axis_tuser = 0;
		m01_axis_tuser = 0;

		case(ap_state)
			AP_STATE_RUN: begin
				m00_axis_tlast = s_axis_tlast;
				m01_axis_tlast = s_axis_tlast;
				m00_axis_tuser = s_axis_tuser;
				m01_axis_tuser = s_axis_tuser;

				if(COLOR_FORMAT_IN == COLOR_FORMAT_YUV444 && COLOR_FORMAT_OUT == COLOR_FORMAT_YUV422) begin
					// Y component
					for(i = 0;i < C_PIXELS_PER_BEAT;i = i + 1) begin
						m00_axis_tdata[C_BITS_PER_COMP*i +: C_BITS_PER_COMP] =
							s_axis_tdata[BITS_PER_PIXEL*i + C_BITS_PER_COMP*0 +: C_BITS_PER_COMP];
					end

					// U/V component
					for(i = 0;i < C_PIXELS_PER_BEAT;i = i + 2) begin
						m01_axis_tdata[C_BITS_PER_COMP*(i+0) +: C_BITS_PER_COMP] =
							(s_axis_tdata[BITS_PER_PIXEL*(i+0) + C_BITS_PER_COMP*1 +: C_BITS_PER_COMP] +
							s_axis_tdata[BITS_PER_PIXEL*(i+1) + C_BITS_PER_COMP*1 +: C_BITS_PER_COMP]) >> 1;
						m01_axis_tdata[C_BITS_PER_COMP*(i+1) +: C_BITS_PER_COMP] =
							(s_axis_tdata[BITS_PER_PIXEL*(i+0) + C_BITS_PER_COMP*2 +: C_BITS_PER_COMP] +
							s_axis_tdata[BITS_PER_PIXEL*(i+1) + C_BITS_PER_COMP*2 +: C_BITS_PER_COMP]) >> 1;
					end
				end
			end
		endcase
	end

	// @FF ERR_MASK
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ERR_MASK <= ERR_MASK_SUCCESS;
		end else begin
			ERR_MASK <= ERR_MASK_next;
		end
	end

	// @COMB ERR_MASK_next
	always @(*) begin
		ERR_MASK_next = ERR_MASK;

		case(ap_state)
			AP_STATE_RUN: begin
			end
		endcase
	end
endmodule
