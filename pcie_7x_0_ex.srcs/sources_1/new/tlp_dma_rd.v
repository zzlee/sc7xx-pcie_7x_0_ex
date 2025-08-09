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
	parameter C_DATA_WIDTH      = 64,
	parameter C_GRP_IDX         = 4'd0, // grp index base for tlp_demuxer/m01_axis_rc_grp
	parameter C_MAX_WIDTH       = 4096,
	parameter C_MAX_HEIGHT      = 2160,
	parameter C_MAX_BURST_SIZE  = 256, // bytes per burst
	parameter C_RC_COUNT        = 15,
	parameter C_RC_CNT_WIDTH    = 4,

	// Do not override parameters below this line
	parameter KEEP_WIDTH  = C_DATA_WIDTH / 8,
	parameter BURST_WIDTH = $clog2(C_MAX_BURST_SIZE + 1),
	parameter DESC_WIDTH  = 32+32+32 // {bytes, addr_hi, addr_lo}
) (
	input clk,
	input rst_n,

	// desc_rd_U signals
	output reg             desc_rd_rd_en,
	input [DESC_WIDTH-1:0] desc_rd_rd_data,
	input                  desc_rd_data_valid,
	output reg             desc_rd_ap_start,
	input                  desc_rd_ap_ready,
	input [31:0]           desc_rd_ERR_MASK,

	// RR TLP
	output reg [C_DATA_WIDTH-1:0] m_axis_rr_tdata,
	output reg [KEEP_WIDTH-1:0]   m_axis_rr_tkeep,
	output reg                    m_axis_rr_tlast,
	output reg                    m_axis_rr_tvalid,
	input                         m_axis_rr_tready,

	// dma_rd_burst_U
	output reg [BURST_WIDTH-1:0] dma_rd_burst_s_axis_ap_burst_bytes,
	output reg                   dma_rd_burst_s_axis_ap_tvalid,
	input                        dma_rd_burst_s_axis_ap_tready,
	input                        dma_rd_burst_fifo_empty,

	input [15:0] cfg_completer_id,

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

	localparam TLP_MEM_RD32_FMT_TYPE = 7'b00_00000;
	localparam TLP_MEM_RD64_FMT_TYPE = 7'b01_00000;

	localparam MAX_SIZE   = C_MAX_WIDTH * C_MAX_HEIGHT * 4;
	localparam SIZE_WIDTH = $clog2(MAX_SIZE + 1);

	localparam AP_STATE_IDLE        = 3'd0;
	localparam AP_STATE_FINISH      = 3'd1;
	localparam AP_STATE_WAIT_DESC   = 3'd2;
	localparam AP_STATE_DMA_RD      = 3'd3;
	localparam AP_STATE_DMA_RD_NEXT = 3'd4;
	localparam AP_STATE_WAIT_TX     = 3'd5;
	localparam AP_STATE_ERROR       = 3'd7;
	localparam AP_STATE_WIDTH       = 3;

	genvar gen_i;
	integer i;

	reg [AP_STATE_WIDTH-1:0] ap_state;
	reg [31:0]               ERR_MASK_next;
	reg [1:0]                tlp_hdr_idx;
	reg [63:0]               buf_addr_int;
	wire [63:0]              buf_addr_next;
	wire [31:0]              buf_addr_lo;
	wire [31:0]              buf_addr_hi;
	reg                      buf_addr_32bit;
	reg [SIZE_WIDTH-1:0]     addr_adder_inc;
	reg [31:0]               desc_buf_size_int, desc_buf_size_next;
	reg [SIZE_WIDTH-1:0]     size_int, size_next;
	reg [BURST_WIDTH-1:0]    burst_bytes_int;

	wire m_axis_rr_fire;

	// TX TLP DW1_0
	wire [6:0] tlp_hdr_fmt_type;
	wire [2:0] tlp_hdr_tc;
	wire       tlp_hdr_td;
	wire       tlp_hdr_ep;
	wire [1:0] tlp_hdr_attr;
	wire [9:0] tlp_hdr_len;
	wire [7:0] tlp_hdr_tag [C_RC_COUNT-1:0];
	wire [3:0] tlp_hdr_last_be;
	wire [3:0] tlp_hdr_first_be;

	// dma_rd_burst_U signals
	reg [C_RC_CNT_WIDTH-1:0] dma_rd_burst_idx;
	wire                     dma_rd_burst_s_axis_ap_fire;

	assign ap_idle = (ap_state == AP_STATE_IDLE);
	assign ap_done = (ap_state == AP_STATE_FINISH);
	assign ap_ready = ap_done;

	assign buf_addr_lo = {buf_addr_int[31:2], 2'b00};
	assign buf_addr_hi = buf_addr_int[63:32];

	addr_adder #(
		.C_INC_WIDTH(SIZE_WIDTH)
	) addr_adder_U (
		.clk(clk),
		.rst_n(rst_n),
		.addr(buf_addr_int),
		.inc(addr_adder_inc),
		.addr_next(buf_addr_next)
	);

	assign tlp_hdr_fmt_type = (buf_addr_32bit ? TLP_MEM_RD32_FMT_TYPE : TLP_MEM_RD64_FMT_TYPE);
	assign tlp_hdr_tc = 3'b0;
	assign tlp_hdr_td = 1'b0;
	assign tlp_hdr_ep = 1'b0;
	assign tlp_hdr_attr = 2'b0;
	assign tlp_hdr_len = (burst_bytes_int >> 2); // measured by DW unit

	generate
		for(gen_i = 0;gen_i < C_RC_COUNT;gen_i = gen_i + 1) begin
			assign tlp_hdr_tag[gen_i] = {C_GRP_IDX, gen_i[3:0]};
		end
	endgenerate

	assign tlp_hdr_last_be = 4'b1111;
	assign tlp_hdr_first_be = 4'b1111;

	// @FF ap_state, @FF tlp_hdr_idx
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_IDLE;
			tlp_hdr_idx <= 0;
		end else begin
			case(ap_state)
				AP_STATE_IDLE:
					if(ap_start) begin
						ap_state <= AP_STATE_WAIT_DESC;
					end

				AP_STATE_WAIT_DESC: begin
					if(desc_rd_data_valid) begin
						ap_state <= AP_STATE_DMA_RD_NEXT;
					end
				end

				AP_STATE_DMA_RD: begin
					case(tlp_hdr_idx)
						0: begin
							if(m_axis_rr_fire) begin
								tlp_hdr_idx <= 1;
							end
						end

						1: begin
							if(m_axis_rr_fire) begin
								tlp_hdr_idx <= 2;
							end
						end

						2: begin
							ap_state <= AP_STATE_DMA_RD_NEXT;

							if(size_next == 0) begin
								ap_state <= AP_STATE_WAIT_TX;
							end else if(desc_buf_size_next == 0) begin
								ap_state <= AP_STATE_WAIT_DESC;
							end
						end
					endcase
				end


				AP_STATE_DMA_RD_NEXT: begin
					if(dma_rd_burst_s_axis_ap_fire) begin
						ap_state <= AP_STATE_DMA_RD;
						tlp_hdr_idx <= 0;
					end
				end

				AP_STATE_WAIT_TX: begin
					if(dma_rd_burst_fifo_empty) begin
						ap_state <= AP_STATE_FINISH;
					end
				end

				AP_STATE_FINISH: begin
					ap_state <= AP_STATE_IDLE;
				end
			endcase

			if(|ERR_MASK) begin
				ap_state <= AP_STATE_ERROR;
			end
		end
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

	// @COMB desc_rd_rd_en
	always @(*) begin
		desc_rd_rd_en = 0;

		case(ap_state)
			AP_STATE_WAIT_DESC: begin
				if(desc_rd_data_valid) begin
					desc_rd_rd_en = 1;
				end
			end
		endcase
	end

	// @COMB m_axis_rr_tdata, @COMB m_axis_rr_tkeep, @COMB m_axis_rr_tlast, @COMB m_axis_rr_tvalid
	always @(*) begin
		m_axis_rr_tdata = 'hEEFFAABBCAFECAFE; // for debug purpose
		m_axis_rr_tkeep = 0;
		m_axis_rr_tlast = 0;
		m_axis_rr_tvalid = 0;

		case(ap_state)
			AP_STATE_DMA_RD: begin
				case(tlp_hdr_idx)
					0: begin
						m_axis_rr_tlast = 0;
						m_axis_rr_tdata = {                // Bits
							// DW1
							cfg_completer_id,              // 16
							tlp_hdr_tag[dma_rd_burst_idx], // 8
							tlp_hdr_last_be,               // 4
							tlp_hdr_first_be,              // 4
							// DW0
							{1'b0},                        // 1
							tlp_hdr_fmt_type,              // 7
							{1'b0},                        // 1
							tlp_hdr_tc,                    // 3
							{4'b0},                        // 4
							tlp_hdr_td,                    // 1
							tlp_hdr_ep,                    // 1
							tlp_hdr_attr,                  // 2
							{2'b0},                        // 2
							tlp_hdr_len                    // 10
						};
						m_axis_rr_tkeep = 8'hFF;
						m_axis_rr_tvalid = 1;
					end

					1: begin
						m_axis_rr_tlast = 1;
						m_axis_rr_tvalid = 1;

						if(buf_addr_32bit) begin
							m_axis_rr_tdata = {
								// DW3
								32'b0,
								// DW2 Addr
								buf_addr_lo
							};
							m_axis_rr_tkeep = 8'h0F;
						end else begin
							m_axis_rr_tdata = {
								// DW3 Addr LO
								buf_addr_lo,
								// DW2 Addr HI
								buf_addr_hi
							};
							m_axis_rr_tkeep = 8'hFF;
						end
					end
				endcase
			end
		endcase
	end

	assign m_axis_rr_fire = m_axis_rr_tready && m_axis_rr_tvalid;

	// @FF buf_addr_int, @FF buf_addr_32bit, @FF addr_adder_inc, @FF size_int, @FF size_next,
	// @FF desc_buf_size_int, @FF desc_buf_size_next, @FF burst_bytes_int
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			buf_addr_int <= 'hCAFE0001; // for debug purpose
			buf_addr_32bit <= 0;
			addr_adder_inc <= 0;
			size_int <= 0;
			size_next <= 0;
			desc_buf_size_int <= 'hCAFE0002; // for debug purpose
			desc_buf_size_next <= 0;
			burst_bytes_int <= 0;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					if(ap_start) begin
						size_int <= (SIZE & ~32'b11);
					end
				end

				AP_STATE_WAIT_DESC: begin
					if(desc_rd_data_valid) begin
						buf_addr_int <= desc_rd_rd_data[63:0];
						buf_addr_32bit <= (desc_rd_rd_data[63:32] == 32'b0);
						desc_buf_size_int <= desc_rd_rd_data[64 +: 32];
					end
				end

				AP_STATE_DMA_RD: begin
					case(tlp_hdr_idx)
						2: begin
							buf_addr_int <= buf_addr_next;
							size_int <= size_next;
							desc_buf_size_int <= desc_buf_size_next;
						end
					endcase
				end

				AP_STATE_DMA_RD_NEXT: begin
					if(desc_buf_size_int > C_MAX_BURST_SIZE) begin
						addr_adder_inc <= C_MAX_BURST_SIZE;
						size_next <= size_int - C_MAX_BURST_SIZE;
						burst_bytes_int <= C_MAX_BURST_SIZE;
						desc_buf_size_next <= desc_buf_size_int - C_MAX_BURST_SIZE;
					end else begin
						addr_adder_inc <= desc_buf_size_int;
						size_next <= size_int - desc_buf_size_int;
						burst_bytes_int <= desc_buf_size_int;
						desc_buf_size_next <= 0;
					end
				end
			endcase
		end
	end

	assign dma_rd_burst_s_axis_ap_fire = dma_rd_burst_s_axis_ap_tready & dma_rd_burst_s_axis_ap_tvalid;

	// @FF dma_rd_burst_idx
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			dma_rd_burst_idx <= 0;
		end else begin
			case(ap_state)
				AP_STATE_DMA_RD: begin
					case(tlp_hdr_idx)
						2: begin
							dma_rd_burst_idx <= (dma_rd_burst_idx == C_RC_COUNT - 1) ? 0 : dma_rd_burst_idx + 1;
						end
					endcase
				end
			endcase
		end
	end

	// @COMB dma_rd_burst_s_axis_ap_tvalid, @COMB dma_rd_burst_s_axis_ap_burst_bytes
	always @(*) begin
		dma_rd_burst_s_axis_ap_tvalid = 0;

		case(ap_state)
			AP_STATE_DMA_RD_NEXT: begin
				if(size_int > C_MAX_BURST_SIZE) begin
					dma_rd_burst_s_axis_ap_burst_bytes = C_MAX_BURST_SIZE;
				end else begin
					dma_rd_burst_s_axis_ap_burst_bytes = size_int;
				end

				dma_rd_burst_s_axis_ap_tvalid = 1;
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
endmodule
