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
	parameter C_DATA_WIDTH     = 64,
	parameter C_SRC_DATA_WIDTH = 32,
	parameter C_MAX_WIDTH      = 4096,
	parameter C_MAX_HEIGHT     = 2160,
	parameter C_MAX_BURST_SIZE = 256, // bytes per burst (FIFO depth C_MAX_BURST_SIZE >> 2)

	// Do not override parameters below this line
	parameter KEEP_WIDTH     = C_DATA_WIDTH / 8,
	parameter SRC_KEEP_WIDTH = C_SRC_DATA_WIDTH / 8,
	parameter DESC_WIDTH     = 32+32+32 // {bytes, addr_hi, addr_lo}
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

	// S00 SRC
	input [C_SRC_DATA_WIDTH-1:0] s00_axis_tdata,
	input [SRC_KEEP_WIDTH-1:0]   s00_axis_tkeep,
	input                        s00_axis_tlast,
	input                        s00_axis_tvalid,
	output                       s00_axis_tready,
	input                        s00_axis_tuser,

	// S01 SRC
	input [C_SRC_DATA_WIDTH-1:0] s01_axis_tdata,
	input [SRC_KEEP_WIDTH-1:0]   s01_axis_tkeep,
	input                        s01_axis_tlast,
	input                        s01_axis_tvalid,
	output                       s01_axis_tready,
	input                        s01_axis_tuser,

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
	// `define USE_FIFO_FWFT

	localparam MAX_BURST_DW = C_MAX_BURST_SIZE >> 2;
	localparam BURST_WIDTH  = $clog2(C_MAX_BURST_SIZE + 1);
	localparam MAX_SIZE     = C_MAX_WIDTH * C_MAX_HEIGHT * 4;
	localparam SIZE_WIDTH   = $clog2(MAX_SIZE + 1);

	localparam TLP_MEM_WR32_FMT_TYPE = 7'b10_00000;
	localparam TLP_MEM_WR64_FMT_TYPE = 7'b11_00000;

	localparam AP_STATE_IDLE        = 3'd0;
	localparam AP_STATE_FINISH      = 3'd1;
	localparam AP_STATE_WAIT_DESC   = 3'd2;
	localparam AP_STATE_DMA_WR      = 3'd3;
	localparam AP_STATE_DMA_WR_NEXT = 3'd4;
	localparam AP_STATE_ERROR       = 3'd7;
	localparam AP_STATE_WIDTH       = 3;

	localparam SRC_COUNT     = 2;
	localparam SRC_CNT_WIDTH = $clog2(SRC_COUNT);

	localparam SRC_STATE_IDLE   = 3'd0;
	localparam SRC_STATE_FINISH = 3'd1;
	localparam SRC_STATE_RX_DW0 = 3'd2;
	localparam SRC_STATE_RX_DW1 = 3'd3;
	localparam SRC_STATE_WIDTH  = 3;

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
	reg [1:0]                tlp_payload_dw_count;
	reg [SRC_CNT_WIDTH-1:0]  src_idx_int;

	wire m_axis_rr_fire;

	wire [7:0] tlp_hdr_tag;
	wire [3:0] tlp_hdr_last_be;
	wire [3:0] tlp_hdr_first_be;
	wire [6:0] tlp_hdr_fmt_type;
	wire [2:0] tlp_hdr_tc;
	wire       tlp_hdr_td;
	wire       tlp_hdr_ep;
	wire [1:0] tlp_hdr_attr;
	reg [9:0]  tlp_hdr_len;

	wire [C_SRC_DATA_WIDTH-1:0] s_axis_tdata [SRC_COUNT-1:0];
	reg                         s_axis_tready [SRC_COUNT-1:0];
	wire                        s_axis_tvalid [SRC_COUNT-1:0];
	wire                        s_axis_fire [SRC_COUNT-1:0];

	// src_fifo_U[SRC_COUNT]
	reg [SRC_STATE_WIDTH-1:0]  src_fifo_state [SRC_COUNT-1:0];
	reg [C_SRC_DATA_WIDTH-1:0] src_fifo_dw0 [SRC_COUNT-1:0];
	reg                        src_fifo_wr_en [SRC_COUNT-1:0];
	wire [C_DATA_WIDTH-1:0]    src_fifo_wr_data [SRC_COUNT-1:0];
	reg                        src_fifo_rd_en [SRC_COUNT-1:0];
	wire [C_DATA_WIDTH-1:0]    src_fifo_rd_data [SRC_COUNT-1:0];
	wire                       src_fifo_data_valid [SRC_COUNT-1:0];
	wire                       src_fifo_full [SRC_COUNT-1:0];
	wire [31:0]                src_fifo_pcie_dw0;
	wire [31:0]                src_fifo_pcie_dw1;
	reg [31:0]                 src_fifo_last_dw [SRC_COUNT-1:0];
	reg                        src_fifo_has_last_dw [SRC_COUNT-1:0];

	assign ap_idle = (ap_state == AP_STATE_IDLE);
	assign ap_done = (ap_state == AP_STATE_FINISH);
	assign ap_ready = ap_done;

	assign s_axis_tdata[0] = s00_axis_tdata;
	assign s00_axis_tready = s_axis_tready[0];
	assign s_axis_tvalid[0] = s00_axis_tvalid;

	assign s_axis_tdata[1] = s01_axis_tdata;
	assign s01_axis_tready = s_axis_tready[1];
	assign s_axis_tvalid[1] = s01_axis_tvalid;

	assign tlp_hdr_fmt_type = (desc_addr_32bit ? TLP_MEM_WR32_FMT_TYPE : TLP_MEM_WR64_FMT_TYPE);
	assign tlp_hdr_tc = 3'b0;
	assign tlp_hdr_td = 1'b0;
	assign tlp_hdr_ep = 1'b0;
	assign tlp_hdr_attr = 2'b0;
	assign tlp_hdr_tag = 8'd0; // Don't care
	assign tlp_hdr_last_be = 4'b1111;
	assign tlp_hdr_first_be = 4'b1111;

	generate
		for(gen_i = 0;gen_i < SRC_COUNT;gen_i = gen_i + 1) begin
`ifdef USE_FIFO_FWFT
			fifo_fwft #(
				.DATA_WIDTH(C_DATA_WIDTH),
				.DEPTH(MAX_BURST_DW)
            ) src_fifo_U (
				.clk(clk),
				.rst_n(rst_n),
				.wr_en(src_fifo_wr_en[gen_i]),
				.wr_data(src_fifo_wr_data[gen_i]),
				.rd_en(src_fifo_rd_en[gen_i]),
				.rd_data(src_fifo_rd_data[gen_i]),
				.full(src_fifo_full[gen_i]),
				.empty(),
				.data_valid(src_fifo_data_valid[gen_i])
			);
`else // USE_FIFO_FWFT
			fifo_dma_wr src_fifo_U (
				.clk(clk),
				.srst(~rst_n),
				.wr_en(src_fifo_wr_en[gen_i]),
				.din(src_fifo_wr_data[gen_i]),
				.rd_en(src_fifo_rd_en[gen_i]),
				.dout(src_fifo_rd_data[gen_i]),
				.full(src_fifo_full[gen_i]),
				.empty(),
				.valid(src_fifo_data_valid[gen_i])
			);
`endif // USE_FIFO_FWFT

			// @FF src_fifo_state[gen_i], src_fifo_dw0[gen_i]
			always @(posedge clk or negedge rst_n) begin
				if(~rst_n) begin
					src_fifo_state[gen_i] <= SRC_STATE_IDLE;
					src_fifo_dw0[gen_i] <= 0;
				end else begin
					case(src_fifo_state[gen_i])
						SRC_STATE_IDLE:
							src_fifo_state[gen_i] <= SRC_STATE_RX_DW0;

						SRC_STATE_RX_DW0:
							if(s_axis_fire[gen_i]) begin
								src_fifo_dw0[gen_i] <= s_axis_tdata[gen_i];
								src_fifo_state[gen_i] <= SRC_STATE_RX_DW1;
							end

						SRC_STATE_RX_DW1:
							if(s_axis_fire[gen_i]) begin
								src_fifo_state[gen_i] <= SRC_STATE_RX_DW0;
							end
					endcase
				end
			end

			// @COMB s_axis_tready[gen_i], @COMB src_fifo_wr_en[gen_i]
			always @(*) begin
				s_axis_tready[gen_i] = 0;
				src_fifo_wr_en[gen_i] = 0;

				case(src_fifo_state[gen_i])
					SRC_STATE_RX_DW0: begin
						s_axis_tready[gen_i] = 1;
					end

					SRC_STATE_RX_DW1: begin
						s_axis_tready[gen_i] = !src_fifo_full[gen_i];
						src_fifo_wr_en[gen_i] = s_axis_fire[gen_i];
					end
				endcase
			end

			assign src_fifo_wr_data[gen_i] = { s_axis_tdata[gen_i], src_fifo_dw0[gen_i] };
			assign s_axis_fire[gen_i] = s_axis_tready[gen_i] && s_axis_tvalid[gen_i];
		end
	endgenerate

	// @COMB src_fifo_rd_en[i]
	always @(*) begin
		for(i = 0;i < SRC_COUNT;i = i + 1) begin
			src_fifo_rd_en[i] = 0;
		end

		case(ap_state)
			AP_STATE_DMA_WR: begin
				case(tlp_hdr_idx)
					1: begin
						if(desc_addr_32bit) begin
							if(! src_fifo_has_last_dw[src_idx_int]) begin
								src_fifo_rd_en[src_idx_int] = m_axis_rr_fire;
							end
						end
					end

					2: begin
						if(m_axis_rr_tkeep == 8'hFF) begin
							src_fifo_rd_en[src_idx_int] = m_axis_rr_fire;
						end else begin
							if(! src_fifo_has_last_dw[src_idx_int]) begin
								src_fifo_rd_en[src_idx_int] = m_axis_rr_fire;
							end
						end
					end
				endcase
			end
		endcase
	end

	// @FF src_fifo_last_dw[i], @FF src_fifo_has_last_dw[i]
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			for(i = 0;i < SRC_COUNT;i = i + 1) begin
				src_fifo_last_dw[i] <= 0;
				src_fifo_has_last_dw[i] <= 0;
			end
		end else begin
			case(ap_state)
				AP_STATE_DMA_WR: begin
					case(tlp_hdr_idx)
						1: begin
							if(desc_addr_32bit) begin
								if(m_axis_rr_fire) begin
									if(src_fifo_has_last_dw[src_idx_int]) begin
										src_fifo_has_last_dw[src_idx_int] <= 0;
									end else begin
										src_fifo_last_dw[src_idx_int] <= src_fifo_pcie_dw1;
										src_fifo_has_last_dw[src_idx_int] <= 1;
									end
								end
							end
						end

						2: begin
							if(m_axis_rr_fire) begin
								if(m_axis_rr_tkeep == 8'hFF) begin
									if(src_fifo_has_last_dw[src_idx_int]) begin
										src_fifo_last_dw[src_idx_int] <= src_fifo_pcie_dw1;
									end
								end else begin
									if(src_fifo_has_last_dw[src_idx_int]) begin
										src_fifo_has_last_dw[src_idx_int] <= 0;
									end else begin
										src_fifo_last_dw[src_idx_int] <= src_fifo_pcie_dw1;
										src_fifo_has_last_dw[src_idx_int] <= 1;
									end
								end
							end
						end
					endcase
				end
			endcase
		end
	end

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
					if(desc_rd_data_valid) begin
						ap_state <= AP_STATE_DMA_WR_NEXT;
						desc_addr_int <= desc_rd_rd_data[63:0];
						desc_bytes_int <= desc_rd_rd_data[64 +: 32];
						desc_addr_32bit <= (desc_rd_rd_data[32 +: 32] == 32'b0);
					end
				end

				AP_STATE_DMA_WR_NEXT: begin
					ap_state <= AP_STATE_DMA_WR;
					tlp_hdr_idx <= 0;
					tlp_hdr_len <= (burst_bytes >> 2);
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
								tlp_hdr_len <= tlp_hdr_len - tlp_payload_dw_count;

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

	// @FF src_idx_int
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			src_idx_int <= 0;
		end else begin
			case(ap_state)
				AP_STATE_DMA_WR_NEXT: begin
					src_idx_int <= ((src_idx_int == 0) ? 1 : 0);
				end
			endcase
		end
	end

	// @COMB burst_bytes, @COMB tlp_payload_dw_count
	always @(*) begin
		burst_bytes = 0;
		tlp_payload_dw_count = 0;

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
						tlp_payload_dw_count = desc_addr_32bit ? 1 : 0;
					end

					2: begin
						tlp_payload_dw_count = (m_axis_rr_tkeep == 'hFF ? 2 : 1);
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

	assign src_fifo_pcie_dw0 = {src_fifo_rd_data[src_idx_int][0*8 +: 8], src_fifo_rd_data[src_idx_int][1*8 +: 8], src_fifo_rd_data[src_idx_int][2*8 +: 8], src_fifo_rd_data[src_idx_int][3*8 +: 8]};
	assign src_fifo_pcie_dw1 = {src_fifo_rd_data[src_idx_int][4*8 +: 8], src_fifo_rd_data[src_idx_int][5*8 +: 8], src_fifo_rd_data[src_idx_int][6*8 +: 8], src_fifo_rd_data[src_idx_int][7*8 +: 8]};

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
								src_fifo_has_last_dw[src_idx_int] ? src_fifo_last_dw[src_idx_int] : src_fifo_pcie_dw0,
								// DW2 Addr LO
								{desc_addr_int[31:2], 2'b00}
							};
							m_axis_rr_tvalid = src_fifo_has_last_dw[src_idx_int] ? 1 : src_fifo_data_valid[src_idx_int];
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
							if(src_fifo_has_last_dw[src_idx_int]) begin
								m_axis_rr_tdata = {
									src_fifo_pcie_dw0,
									src_fifo_last_dw[src_idx_int]
								};
							end else begin
								m_axis_rr_tdata = {
									src_fifo_pcie_dw1,
									src_fifo_pcie_dw0
								};
							end
							m_axis_rr_tvalid = src_fifo_data_valid[src_idx_int];
						end else begin
							if(src_fifo_has_last_dw[src_idx_int]) begin
								m_axis_rr_tdata = {
									32'd0,
									src_fifo_last_dw[src_idx_int]
								};
								m_axis_rr_tvalid = 1;
							end else begin
								m_axis_rr_tdata = {
									src_fifo_pcie_dw1,
									src_fifo_pcie_dw0
								};
								m_axis_rr_tvalid = src_fifo_data_valid[src_idx_int];
							end
						end
					end
				endcase
			end
		endcase
	end

	assign m_axis_rr_fire = m_axis_rr_tready && m_axis_rr_tvalid;

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
