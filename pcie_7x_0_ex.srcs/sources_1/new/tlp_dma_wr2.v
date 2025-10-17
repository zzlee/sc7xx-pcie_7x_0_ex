`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 10/07/2025 10:20:29 PM
// Design Name:
// Module Name: tlp_dma_wr2
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

module tlp_dma_wr2 #(
	parameter C_DATA_WIDTH      = 64,
	parameter C_MAX_WIDTH       = 4096,
	parameter C_MAX_HEIGHT      = 2160,
	parameter C_MAX_BURST_SIZE  = 256, // bytes per burst (FIFO depth C_MAX_BURST_SIZE >> 2)

	// Do not override parameters below this line
	parameter KEEP_WIDTH     = C_DATA_WIDTH / 8,
	parameter S_DATA_WIDTH   = 3 * 32, // 3DWs
	parameter S_KEEP_WIDTH   = ((S_DATA_WIDTH + 7) / 8)
) (
	input clk,
	input rst_n,

	// desc_rd_U signals
	output reg   desc_rd_fifo0_rd_en,
	input [95:0] desc_rd_fifo0_dout,
	input        desc_rd_fifo0_valid,
	output reg   desc_rd_ap_start,
	input        desc_rd_ap_ready,
	input [31:0] desc_rd_ERR_MASK,

	// axis_to_tlp_wr_U signals
	output [79:0] m_axis_req_tdata,
	output reg    m_axis_req_tvalid,
	input         m_axis_req_tready,

	// ap params
	input [31:0]      SIZE,
	output reg [31:0] ERR_MASK,

	// ap ctrl
	input  ap_start,
	output ap_done,
	output ap_ready,
	output ap_idle
);
	`include "err_mask.vh"

	localparam BURST_WIDTH  = $clog2(C_MAX_BURST_SIZE + 1);
	localparam MAX_SIZE     = C_MAX_WIDTH * C_MAX_HEIGHT * 4;
	localparam SIZE_WIDTH   = $clog2(MAX_SIZE + 1);

	localparam AP_STATE_IDLE        = 3'd0;
	localparam AP_STATE_FINISH      = 3'd1;
	localparam AP_STATE_WAIT_DESC   = 3'd2;
	localparam AP_STATE_DMA_WR      = 3'd3;
	localparam AP_STATE_DMA_WR_NEXT = 3'd4;
	localparam AP_STATE_ERROR       = 3'd7;
	localparam AP_STATE_WIDTH       = 3;

	genvar gen_i;
	integer i;

	reg [AP_STATE_WIDTH-1:0] ap_state;
	reg [31:0]               ERR_MASK_next;
	reg [SIZE_WIDTH-1:0]     size_int;
	reg [SIZE_WIDTH-1:0]     size_next_int;
	reg [63:0]               addr_int;
	reg [63:0]               addr_next_int;
	reg [31:0]               bytes_int;
	reg [31:0]               bytes_next_int;
	reg [BURST_WIDTH-1:0]    burst_bytes;
	wire [63:0]              desc_rd_fifo0_addr;
	wire [31:0]              desc_rd_fifo0_bytes;
	reg [63:0]               m_axis_req_tdata_addr;
	reg [9:0]                m_axis_req_tdata_len; // in DWs
	wire                     m_axis_req_fire;

	assign ap_idle = (ap_state == AP_STATE_IDLE);
	assign ap_done = (ap_state == AP_STATE_FINISH);
	assign ap_ready = ap_done;

	assign desc_rd_fifo0_addr = desc_rd_fifo0_dout[0 +: 64];
	assign desc_rd_fifo0_bytes = desc_rd_fifo0_dout[64 +: 32];
	assign m_axis_req_fire = m_axis_req_tvalid && m_axis_req_tready;
	assign m_axis_req_tdata[0 +: 64] = m_axis_req_tdata_addr;
	assign m_axis_req_tdata[64 +: 10] = m_axis_req_tdata_len;

	// @FF ap_state, @FF size_int, @FF size_next_int,
	// @FF addr_int, @FF addr_next_int, @FF bytes_int, @FF bytes_next_int
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_IDLE;
			size_int <= 0;
			size_next_int <= 0;
			addr_int <= 0;
			addr_next_int <= 0;
			bytes_int <= 0;
			bytes_next_int <= 0;

			m_axis_req_tdata_addr <= 0;
			m_axis_req_tdata_len <= 0;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					if(ap_start) begin
						size_int <= (SIZE & ~32'b11);
						ap_state <= AP_STATE_WAIT_DESC;
					end
				end

				AP_STATE_WAIT_DESC: begin
					if(desc_rd_fifo0_valid) begin
						addr_int <= desc_rd_fifo0_addr;
						bytes_int <= desc_rd_fifo0_bytes;

						ap_state <= AP_STATE_DMA_WR_NEXT;
					end
				end

				AP_STATE_DMA_WR_NEXT: begin
					ap_state <= AP_STATE_DMA_WR;

					m_axis_req_tdata_addr <= addr_int;
					m_axis_req_tdata_len <= burst_bytes[2 +: BURST_WIDTH-2]; // in DWs

					addr_next_int <= addr_int + burst_bytes;
					bytes_next_int <= bytes_int - burst_bytes;
					size_next_int <= size_int - burst_bytes;
				end

				AP_STATE_DMA_WR: begin
					if(m_axis_req_fire) begin
						addr_int <= addr_next_int;
						bytes_int <= bytes_next_int;
						size_int <= size_next_int;

						if(size_next_int == 0) begin
							ap_state <= AP_STATE_FINISH;
						end else if(bytes_next_int == 0) begin
							ap_state <= AP_STATE_WAIT_DESC;
						end else begin
							ap_state <= AP_STATE_DMA_WR_NEXT;
						end
					end
				end

				AP_STATE_FINISH: begin
					ap_state <= AP_STATE_IDLE;
				end

				AP_STATE_ERROR: begin
				end
			endcase

			if(|ERR_MASK) begin
				ap_state <= AP_STATE_ERROR;
			end
		end
	end

	// @COMB burst_bytes
	always @(*) begin
		burst_bytes = 0;

		case(ap_state)
			AP_STATE_DMA_WR_NEXT: begin
				if(bytes_int > size_int) begin
					burst_bytes = (size_int > C_MAX_BURST_SIZE ? C_MAX_BURST_SIZE : size_int);
				end else begin
					burst_bytes = (bytes_int > C_MAX_BURST_SIZE ? C_MAX_BURST_SIZE : bytes_int);
				end
			end
		endcase
	end

	// @FF desc_rd_ap_start
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			desc_rd_ap_start <= 0;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					if(ap_start) begin
						desc_rd_ap_start <= 1;
					end
				end

				default: begin
					if(desc_rd_ap_ready) begin
						desc_rd_ap_start <= 0;
					end
				end
			endcase
		end
	end

	// @COMB desc_rd_fifo0_rd_en
	always @(*) begin
		desc_rd_fifo0_rd_en = 0;

		case(ap_state)
			AP_STATE_WAIT_DESC: begin
				if(desc_rd_fifo0_valid) begin
					desc_rd_fifo0_rd_en = 1;
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
		ERR_MASK_next = ERR_MASK | desc_rd_ERR_MASK;

		// TODO: add error handling
		// case(ap_state)
		// 	AP_STATE_XXX: begin
		// 		ERR_MASK_next = ERR_MASK_next | ERR_MASK_XXX;
		// 	end
		// endcase
	end

	// @COMB m_axis_req_tvalid
	always @(*) begin
		m_axis_req_tvalid = 0;

		case(ap_state)
			AP_STATE_DMA_WR: begin
				m_axis_req_tvalid = 1;
			end
		endcase

	end
endmodule
