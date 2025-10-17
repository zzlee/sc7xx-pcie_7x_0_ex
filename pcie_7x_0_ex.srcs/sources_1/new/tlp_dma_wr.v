`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 06/21/2025 04:36:29 PM
// Design Name:
// Module Name: tlp_dma_wr
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

module tlp_dma_wr #(
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

	// RR TLP
	output reg [C_DATA_WIDTH-1:0] m_axis_rr_tdata,
	output reg [KEEP_WIDTH-1:0]   m_axis_rr_tkeep,
	output reg                    m_axis_rr_tlast,
	output reg                    m_axis_rr_tvalid,
	input                         m_axis_rr_tready,

	// SRC data
	input [S_DATA_WIDTH-1:0] s_axis_tdata,
	input [S_KEEP_WIDTH-1:0] s_axis_tkeep,
	input                    s_axis_tlast,
	input                    s_axis_tvalid,
	output reg               s_axis_tready,
	input                    s_axis_tuser,

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

	localparam BURST_WIDTH  = $clog2(C_MAX_BURST_SIZE + 1);
	localparam MAX_SIZE     = C_MAX_WIDTH * C_MAX_HEIGHT * 4;
	localparam SIZE_WIDTH   = $clog2(MAX_SIZE + 1);
	localparam S_FIFO_WIDTH = 6 * 32; // 6DWs

	localparam TLP_MEM_WR32_FMT_TYPE = 7'b10_00000;
	localparam TLP_MEM_WR64_FMT_TYPE = 7'b11_00000;

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
	reg [SIZE_WIDTH-1:0]     size_int;
	reg [31:0]               ERR_MASK_next;
	reg [1:0]                tlp_hdr_idx;
	reg [63:0]               desc_addr_int;
	reg [31:0]               desc_bytes_int;
	reg                      desc_addr_32bit;
	reg [63:0]               desc_addr_next_int;
	reg [BURST_WIDTH-1:0]    burst_bytes;
	reg [1:0]                tlp_payload_dws;
	wire [63:0]              desc_rd_fifo0_addr;
	wire [31:0]              desc_rd_fifo0_bytes;

	wire m_axis_rr_fire;
	wire s_axis_fire;

	wire [7:0] tlp_hdr_tag;
	wire [3:0] tlp_hdr_last_be;
	wire [3:0] tlp_hdr_first_be;
	wire [6:0] tlp_hdr_fmt_type;
	wire [2:0] tlp_hdr_tc;
	wire       tlp_hdr_td;
	wire       tlp_hdr_ep;
	wire [1:0] tlp_hdr_attr;
	reg [9:0]  tlp_hdr_len;

	reg [S_FIFO_WIDTH-1:0] s_fifo;
	reg [2:0]              s_fifo_dws;
	wire [31:0]            s_fifo_pci_dw0;
	wire [31:0]            s_fifo_pci_dw1;

	assign ap_idle = (ap_state == AP_STATE_IDLE);
	assign ap_done = (ap_state == AP_STATE_FINISH);
	assign ap_ready = ap_done;

	assign tlp_hdr_fmt_type = (desc_addr_32bit ? TLP_MEM_WR32_FMT_TYPE : TLP_MEM_WR64_FMT_TYPE);
	assign tlp_hdr_tc = 3'b0;
	assign tlp_hdr_td = 1'b0;
	assign tlp_hdr_ep = 1'b0;
	assign tlp_hdr_attr = 2'b0;
	assign tlp_hdr_tag = 8'd0; // Don't care
	assign tlp_hdr_last_be = 4'b1111;
	assign tlp_hdr_first_be = 4'b1111;

	assign desc_rd_fifo0_addr = desc_rd_fifo0_dout[0 +: 64];
	assign desc_rd_fifo0_bytes = desc_rd_fifo0_dout[64 +: 32];
	assign m_axis_rr_fire = m_axis_rr_tready && m_axis_rr_tvalid;
	assign s_axis_fire = s_axis_tready && s_axis_tvalid;

	// @FF ap_state, @FF size_int, @FF tlp_hdr_idx, @FF tlp_hdr_len,
	// @FF desc_addr_int, @FF desc_bytes_int, @FF desc_addr_32bit, @FF desc_addr_next_int
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_IDLE;
			size_int <= 0;
			tlp_hdr_idx <= 0;
			tlp_hdr_len <= 0;
			desc_addr_int <= 0;
			desc_bytes_int <= 0;
			desc_addr_32bit <= 0;
			desc_addr_next_int <= 0;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					if(ap_start) begin
						ap_state <= AP_STATE_WAIT_DESC;
						size_int <= (SIZE & ~32'b11);
					end
				end

				AP_STATE_WAIT_DESC: begin
					if(desc_rd_fifo0_valid) begin
						ap_state <= AP_STATE_DMA_WR_NEXT;
						desc_addr_int <= desc_rd_fifo0_addr;
						desc_bytes_int <= desc_rd_fifo0_bytes;
						desc_addr_32bit <= (desc_rd_fifo0_addr[63:32] == 32'b0);
					end
				end

				AP_STATE_DMA_WR_NEXT: begin
					ap_state <= AP_STATE_DMA_WR;
					tlp_hdr_idx <= 0;
					tlp_hdr_len <= burst_bytes[2 +: BURST_WIDTH-2]; // in DWs
					desc_addr_next_int <= desc_addr_int + burst_bytes;
					desc_bytes_int <= desc_bytes_int - burst_bytes;
					size_int <= size_int - burst_bytes;
				end

				AP_STATE_DMA_WR: begin
					if(m_axis_rr_fire) begin
						case(tlp_hdr_idx)
							0: begin
								tlp_hdr_idx <= 1;
							end

							1, 2: begin
								tlp_hdr_idx <= 2;
								tlp_hdr_len <= tlp_hdr_len - tlp_payload_dws;

								if(m_axis_rr_tlast) begin
									ap_state <= AP_STATE_DMA_WR_NEXT;
									desc_addr_int <= desc_addr_next_int;

									if(size_int == 0) begin
										ap_state <= AP_STATE_FINISH;
									end else if(desc_bytes_int == 0) begin
										ap_state <= AP_STATE_WAIT_DESC;
									end
								end
							end
						endcase
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

	// @COMB burst_bytes, @COMB tlp_payload_dws
	always @(*) begin
		burst_bytes = 0;
		tlp_payload_dws = 0;

		case(ap_state)
			AP_STATE_DMA_WR_NEXT: begin
				if(desc_bytes_int > size_int) begin
					burst_bytes = (size_int > C_MAX_BURST_SIZE ? C_MAX_BURST_SIZE : size_int);
				end else begin
					burst_bytes = (desc_bytes_int > C_MAX_BURST_SIZE ? C_MAX_BURST_SIZE : desc_bytes_int);
				end
			end

			AP_STATE_DMA_WR: begin
				case(tlp_hdr_idx)
					1: begin
						tlp_payload_dws = desc_addr_32bit ? 1 : 0;
					end

					2: begin
						tlp_payload_dws = (m_axis_rr_tkeep == 'hFF ? 2 : 1);
					end
				endcase
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

	assign s_fifo_pci_dw0 = {s_fifo[0*8 +: 8], s_fifo[1*8 +: 8], s_fifo[2*8 +: 8], s_fifo[3*8 +: 8]};
	assign s_fifo_pci_dw1 = {s_fifo[4*8 +: 8], s_fifo[5*8 +: 8], s_fifo[6*8 +: 8], s_fifo[7*8 +: 8]};

	// @COMB m_axis_rr_tdata, @COMB m_axis_rr_tkeep, @COMB m_axis_rr_tlast, @COMB m_axis_rr_tvalid
	always @(*) begin
		m_axis_rr_tdata = 'hAABBCCDDCAFECAFE; // for debug purpose
		m_axis_rr_tkeep = 0;
		m_axis_rr_tlast = 0;
		m_axis_rr_tvalid = 0;

		case(ap_state)
			AP_STATE_DMA_WR: begin
				case(tlp_hdr_idx)
					0: begin
						m_axis_rr_tlast = 0;
						m_axis_rr_tdata = {           // Bits
							// DW1
							cfg_completer_id,         // 16
							tlp_hdr_tag,              // 8
							tlp_hdr_last_be,          // 4
							tlp_hdr_first_be,         // 4
							// DW0
							{1'b0},                   // 1
							tlp_hdr_fmt_type,         // 7
							{1'b0},                   // 1
							tlp_hdr_tc,               // 3
							{4'b0},                   // 4
							tlp_hdr_td,               // 1
							tlp_hdr_ep,               // 1
							tlp_hdr_attr,             // 2
							{2'b0},                   // 2
							tlp_hdr_len               // 10
						};
						m_axis_rr_tkeep = 8'hFF;
						m_axis_rr_tvalid = 1;
					end

					1: begin
						m_axis_rr_tkeep = 8'hFF;

						if(desc_addr_32bit) begin
							m_axis_rr_tlast = (tlp_hdr_len == 1);
							m_axis_rr_tdata = {
								// DW3 Data
								s_fifo_pci_dw0,
								// DW2 Addr LO
								{desc_addr_int[31:2], 2'b00}
							};
							m_axis_rr_tvalid = (s_fifo_dws >= tlp_payload_dws);
						end else begin
							m_axis_rr_tlast = 0;
							m_axis_rr_tdata = {
								// DW3 Addr LO
								{desc_addr_int[31:2], 2'b00},
								// DW2 Addr HI
								desc_addr_int[63:32]
							};
							m_axis_rr_tvalid = 1;
						end
					end

					2: begin
						m_axis_rr_tlast = (tlp_hdr_len <= 2);
						m_axis_rr_tkeep = (tlp_hdr_len == 1 ? 8'h0F : 8'hFF);

						if(m_axis_rr_tkeep == 8'hFF) begin
							m_axis_rr_tdata = {
								s_fifo_pci_dw1,
								s_fifo_pci_dw0
							};
							m_axis_rr_tvalid = (s_fifo_dws >= tlp_payload_dws);
						end else begin
							m_axis_rr_tdata = {
								32'd0,
								s_fifo_pci_dw0
							};
							m_axis_rr_tvalid = (s_fifo_dws >= tlp_payload_dws);
						end
					end
				endcase
			end
		endcase
	end

	// @FF s_fifo, @FF s_fifo_dws
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			s_fifo <= 0;
			s_fifo_dws <= 0;
		end else begin
			case(ap_state)
				AP_STATE_DMA_WR: begin
					case(tlp_hdr_idx)
						1, 2: begin
							case({s_axis_fire, m_axis_rr_fire})
								2'b01: begin
									// s_fifo[0*32 +: s_fifo_dws*32] <= s_fifo[tlp_payload_dws*32 +: s_fifo_dws*32];
									case(tlp_payload_dws)
										1:
											case(s_fifo_dws)
												1: s_fifo[0*32 +: 1*32] <= s_fifo[1*32 +: 1*32];
												2: s_fifo[0*32 +: 2*32] <= s_fifo[1*32 +: 2*32];
												3: s_fifo[0*32 +: 3*32] <= s_fifo[1*32 +: 3*32];
												default: ;
											endcase
										2:
											case(s_fifo_dws)
												1: s_fifo[0*32 +: 1*32] <= s_fifo[2*32 +: 1*32];
												2: s_fifo[0*32 +: 2*32] <= s_fifo[2*32 +: 2*32];
												3: s_fifo[0*32 +: 3*32] <= s_fifo[2*32 +: 3*32];
												default: ;
											endcase
										default: ;
									endcase
									s_fifo_dws <= s_fifo_dws - tlp_payload_dws;
								end

								2'b10: begin
									// s_fifo[s_fifo_dws*32 +: S_DATA_WIDTH] <= s_axis_tdata;
									case(s_fifo_dws)
										0: s_fifo[0*32 +: S_DATA_WIDTH] <= s_axis_tdata;
										1: s_fifo[1*32 +: S_DATA_WIDTH] <= s_axis_tdata;
										2: s_fifo[2*32 +: S_DATA_WIDTH] <= s_axis_tdata;
										3: s_fifo[3*32 +: S_DATA_WIDTH] <= s_axis_tdata;
										default: ;
									endcase
									s_fifo_dws <= s_fifo_dws + (S_DATA_WIDTH / 32);
								end

								2'b11: begin
									// s_fifo[0*32 +: (s_fifo_dws-tlp_payload_dws)*32] <= s_fifo[tlp_payload_dws*32 +: (s_fifo_dws-tlp_payload_dws)*32];
									case(tlp_payload_dws)
										1:
											case(s_fifo_dws)
												2: s_fifo[0*32 +: 1*32] <= s_fifo[1*32 +: 1*32];
												3: s_fifo[0*32 +: 2*32] <= s_fifo[1*32 +: 2*32];
												default: ;
											endcase
										2:
											case(s_fifo_dws)
												3: s_fifo[0*32 +: 1*32] <= s_fifo[2*32 +: 1*32];
												default: ;
											endcase
										default: ;
									endcase

									// s_fifo[(s_fifo_dws-tlp_payload_dws)*32 +: S_DATA_WIDTH] <= s_axis_tdata;
									case(tlp_payload_dws)
										0:
											case(s_fifo_dws)
												0: s_fifo[0*32 +: S_DATA_WIDTH] <= s_axis_tdata;
												1: s_fifo[1*32 +: S_DATA_WIDTH] <= s_axis_tdata;
												2: s_fifo[2*32 +: S_DATA_WIDTH] <= s_axis_tdata;
												3: s_fifo[3*32 +: S_DATA_WIDTH] <= s_axis_tdata;
												default: ;
											endcase
										1:
											case(s_fifo_dws)
												1: s_fifo[0*32 +: S_DATA_WIDTH] <= s_axis_tdata;
												2: s_fifo[1*32 +: S_DATA_WIDTH] <= s_axis_tdata;
												3: s_fifo[2*32 +: S_DATA_WIDTH] <= s_axis_tdata;
												default: ;
											endcase
										2:
											case(s_fifo_dws)
												2: s_fifo[0*32 +: S_DATA_WIDTH] <= s_axis_tdata;
												3: s_fifo[1*32 +: S_DATA_WIDTH] <= s_axis_tdata;
												default: ;
											endcase
										default: ;
									endcase
									s_fifo_dws <= s_fifo_dws + (S_DATA_WIDTH / 32) - tlp_payload_dws;
								end
							endcase
						end
					endcase
				end
			endcase
		end
	end

	// @COMB s_axis_tready
	always @(*) begin
		s_axis_tready = 0;

		case(ap_state)
			AP_STATE_DMA_WR: begin
				case(tlp_hdr_idx)
					1, 2: begin
						if(s_fifo_dws <= ((S_FIFO_WIDTH - S_DATA_WIDTH) / 32)) begin
							s_axis_tready = 1;
						end
					end
				endcase
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
