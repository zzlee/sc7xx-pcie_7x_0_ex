`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 07/11/2025 13:36:29 PM
// Design Name:
// Module Name: tlp_xdma_desc_rd
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

module tlp_xdma_desc_rd #(
	parameter C_DATA_WIDTH     = 64,
	parameter C_DESC_IDX       = 4'd0, // desc index for tlp_demuxer/m00_axis_rc_desc
	parameter C_DMA_DIR        = 0, // 0 => DMA_FROM_DEVICE, 1 => DMA_TO_DEVICE
	parameter C_MAX_BURST_SIZE = 256, // bytes per burst

	// Do not override parameters below this line
	parameter KEEP_WIDTH   = C_DATA_WIDTH / 8,
	parameter DESC_WIDTH   = 32+32+32 // {bytes, addr_hi, addr_lo}
) (
	input clk,
	input rst_n,

	// RC DESC TLP
	input [C_DATA_WIDTH-1:0] s_axis_rc_desc_tdata,
	input [KEEP_WIDTH-1:0]   s_axis_rc_desc_tkeep,
	input                    s_axis_rc_desc_tlast,
	input                    s_axis_rc_desc_tvalid,
	output reg               s_axis_rc_desc_tready,

	// RR DESC TLP
	output reg [C_DATA_WIDTH-1:0] m_axis_rr_desc_tdata,
	output reg [KEEP_WIDTH-1:0]   m_axis_rr_desc_tkeep,
	output reg                    m_axis_rr_desc_tlast,
	output reg                    m_axis_rr_desc_tvalid,
	input                         m_axis_rr_desc_tready,

	input [15:0] cfg_completer_id,

	// FIFO RD
	input                   rd_en,
	output [DESC_WIDTH-1:0] rd_data,
	output                  data_valid,

	// ap params
	input [63:0]      DESC_ADDR,
	input [31:0]      DESC_ADJ,
	output reg [31:0] ERR_MASK,

	// ap ctrl
	input  ap_start,
	output ap_done,
	output ap_ready,
	output ap_idle
);
	`include "err_mask.vh"
	// `define USE_FIFO_FWFT

	localparam MAX_BURST_DW   = C_MAX_BURST_SIZE >> 2;
	localparam BYTES_PER_DESC = 32;

	localparam TLP_MEM_RD32_FMT_TYPE = 7'b00_00000;
	localparam TLP_MEM_RD64_FMT_TYPE = 7'b01_00000;

	localparam AP_STATE_IDLE        = 3'd0;
	localparam AP_STATE_FINISH      = 3'd1;
	localparam AP_STATE_DMA_RD      = 3'd2;
	localparam AP_STATE_DMA_RD_NEXT = 3'd3;
	localparam AP_STATE_RX          = 3'd4;
	localparam AP_STATE_WR_FIFO     = 3'd5;
	localparam AP_STATE_FIFO_EMPTY  = 3'd6;
	localparam AP_STATE_ERROR       = 3'd7;
	localparam AP_STATE_WIDTH       = 3;

	localparam DESC_MAGIC      = 16'hAD4B;
	localparam DMA_FROM_DEVICE = 0;
	localparam DMA_TO_DEVICE   = 1;

	reg [AP_STATE_WIDTH-1:0] ap_state;
	reg [31:0]               ERR_MASK_next;
	reg [0:0]                tlp_hdr_idx;
	reg [1:0]                rx_tlp_hdr_idx;
	reg [63:0]               addr_int;
	reg                      addr_32bit;
	reg [9:0]                desc_adj_dw_int;

	// TX TLP DW1_0
	wire [6:0] tlp_hdr_fmt_type;
	wire [2:0] tlp_hdr_tc;
	wire       tlp_hdr_td;
	wire       tlp_hdr_ep;
	wire [1:0] tlp_hdr_attr;
	reg [9:0]  tlp_hdr_len;
	wire [7:0] tlp_hdr_tag;
	wire [3:0] tlp_hdr_last_be;
	wire [3:0] tlp_hdr_first_be;

	wire m_axis_rr_desc_fire;
	wire s_axis_rc_desc_fire;

	reg [2:0]            desc_payload_idx;
	reg [31:0]           desc_payload [7+1:0];
	wire [31:0]          desc_control;
	wire [31:0]          desc_bytes;
	wire [31:0]          desc_src_addr_lo;
	wire [31:0]          desc_src_addr_hi;
	wire [31:0]          desc_dst_addr_lo;
	wire [31:0]          desc_dst_addr_hi;
	wire [31:0]          desc_next_lo;
	wire [31:0]          desc_next_hi;
	wire [31:0]          tlp_pcie_dw0;
	wire [31:0]          tlp_pcie_dw1;
	reg                  desc_tlast;
	reg [KEEP_WIDTH-1:0] desc_tkeep;

	// fifo_xdma_desc_U signals
	reg                  fifo_xdma_desc_wr_en;
	reg [DESC_WIDTH-1:0] fifo_xdma_desc_wr_data;
	wire                 fifo_xdma_desc_full;
	wire                 fifo_xdma_desc_empty;

	assign ap_idle = (ap_state == AP_STATE_IDLE);
	assign ap_done = (ap_state == AP_STATE_FINISH);
	assign ap_ready = ap_done;

	assign tlp_hdr_fmt_type = (addr_32bit ? TLP_MEM_RD32_FMT_TYPE : TLP_MEM_RD64_FMT_TYPE);
	assign tlp_hdr_tc = 3'b0;
	assign tlp_hdr_td = 1'b0;
	assign tlp_hdr_ep = 1'b0;
	assign tlp_hdr_attr = 2'b0;
	assign tlp_hdr_tag = {4'hF, C_DESC_IDX};
	assign tlp_hdr_last_be = 4'b1111;
	assign tlp_hdr_first_be = 4'b1111;

	assign desc_control = desc_payload[0];
	assign desc_bytes = desc_payload[1];
	assign desc_src_addr_lo = desc_payload[2];
	assign desc_src_addr_hi = desc_payload[3];
	assign desc_dst_addr_lo = desc_payload[4];
	assign desc_dst_addr_hi = desc_payload[5];
	assign desc_next_lo = desc_payload[6];
	assign desc_next_hi = desc_payload[7];
	assign tlp_pcie_dw0 = {s_axis_rc_desc_tdata[0*8 +: 8], s_axis_rc_desc_tdata[1*8 +: 8], s_axis_rc_desc_tdata[2*8 +: 8], s_axis_rc_desc_tdata[3*8 +: 8]};
	assign tlp_pcie_dw1 = {s_axis_rc_desc_tdata[4*8 +: 8], s_axis_rc_desc_tdata[5*8 +: 8], s_axis_rc_desc_tdata[6*8 +: 8], s_axis_rc_desc_tdata[7*8 +: 8]};

	// @FF ap_state, @FF tlp_hdr_len, @FF desc_payload_idx, @FF desc_control, @FF desc_bytes, @FF desc_src_addr_lo,
	// @FF desc_src_addr_hi, @FF desc_dst_addr_lo, @FF desc_dst_addr_hi, @FF desc_next_lo, @FF desc_next_hi,
	// @FF addr_int, @FF addr_32bit
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_IDLE;
			tlp_hdr_idx <= 0;
			tlp_hdr_len <= 0;
			rx_tlp_hdr_idx <= 0;
			desc_payload_idx <= 0;
			desc_tlast <= 0;
			desc_adj_dw_int <= 0;
			addr_int <= 0;
			addr_32bit <= 0;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					if(ap_start) begin
						ap_state <= AP_STATE_DMA_RD_NEXT;
						addr_int <= DESC_ADDR;
						desc_adj_dw_int <= (DESC_ADJ[0 +: 6] + 1) << 3; // in DWs
					end
				end

				AP_STATE_DMA_RD_NEXT: begin
					ap_state <= AP_STATE_DMA_RD;
					addr_32bit <= (addr_int[63:32] == 32'b0);
					tlp_hdr_idx <= 0;

					if(desc_adj_dw_int > MAX_BURST_DW) begin
						tlp_hdr_len <= MAX_BURST_DW;
						desc_adj_dw_int <= desc_adj_dw_int - MAX_BURST_DW;
					end else begin
						tlp_hdr_len <= desc_adj_dw_int;
						desc_adj_dw_int <= 0;
					end
				end

				AP_STATE_DMA_RD: begin
					if(m_axis_rr_desc_fire) begin
						if(tlp_hdr_idx == 1) begin
							ap_state <= AP_STATE_RX;
							rx_tlp_hdr_idx <= 0;
							desc_payload_idx <= 0;
						end else begin
							tlp_hdr_idx <= 1;
						end
					end
				end

				AP_STATE_RX: begin
					if(s_axis_rc_desc_fire) begin
						case(rx_tlp_hdr_idx)
							0: begin
								rx_tlp_hdr_idx <= 1;
								tlp_hdr_len <= tlp_hdr_len - s_axis_rc_desc_tdata[9:0];
								addr_int <= addr_int + (s_axis_rc_desc_tdata[9:0] << 2);
							end

							1: begin
								rx_tlp_hdr_idx <= 2;
								desc_payload[desc_payload_idx] <= tlp_pcie_dw1;
								desc_tlast <= s_axis_rc_desc_tlast;
								desc_tkeep <= s_axis_rc_desc_tkeep;

								if(desc_payload_idx == 7) begin
									ap_state <= AP_STATE_WR_FIFO;
								end else begin
									desc_payload_idx <= desc_payload_idx + 1;

									if(s_axis_rc_desc_tlast) begin
										if(tlp_hdr_len == 0) begin
											ap_state <= AP_STATE_FIFO_EMPTY;
										end
									end
								end
							end

							2: begin
								desc_payload[desc_payload_idx + 0] <= tlp_pcie_dw0;
								desc_tlast <= s_axis_rc_desc_tlast;
								desc_tkeep <= s_axis_rc_desc_tkeep;

								case(s_axis_rc_desc_tkeep)
									'hFF: begin
										desc_payload[desc_payload_idx + 1] <= tlp_pcie_dw1;

										if(desc_payload_idx == 6 || desc_payload_idx == 7) begin
											ap_state <= AP_STATE_WR_FIFO;
										end else begin
											desc_payload_idx <= desc_payload_idx + 2;

											if(s_axis_rc_desc_tlast) begin
												if(tlp_hdr_len == 0) begin
													ap_state <= AP_STATE_FIFO_EMPTY;
												end
											end
										end
									end

									'h0F: begin
										if(desc_payload_idx == 7) begin
											ap_state <= AP_STATE_WR_FIFO;
										end else begin
											desc_payload_idx <= desc_payload_idx + 1;

											if(s_axis_rc_desc_tlast) begin
												if(tlp_hdr_len == 0) begin
													ap_state <= AP_STATE_FIFO_EMPTY;
												end
											end
										end
									end
								endcase
							end
						endcase
					end
				end

				AP_STATE_WR_FIFO: begin
					if(! fifo_xdma_desc_full) begin
						ap_state <= AP_STATE_RX;

						if(desc_payload_idx == 7 && desc_tkeep == 'hFF) begin
							desc_payload[0] <= desc_payload[8];
							desc_payload_idx <= 1;
						end else begin
							desc_payload_idx <= 0;
						end

						if(desc_tlast) begin
							if(tlp_hdr_len == 0) begin
								ap_state <= AP_STATE_FIFO_EMPTY;
							end
						end
					end
				end

				AP_STATE_FIFO_EMPTY: begin
					if(fifo_xdma_desc_empty) begin
						if(desc_control[0] == 1) begin
							ap_state <= AP_STATE_FINISH;
						end else begin
							ap_state <= AP_STATE_DMA_RD_NEXT;

							if(desc_adj_dw_int == 0) begin
								addr_int <= {desc_next_hi, desc_next_lo};
								desc_adj_dw_int <= (desc_control[8 +: 6] + 1) << 3; // in DWs
							end
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

	// @COMB m_axis_rr_desc_tdata, @COMB m_axis_rr_desc_tkeep, @COMB m_axis_rr_desc_tlast, @COMB m_axis_rr_desc_tvalid
	always @(*) begin
		m_axis_rr_desc_tdata = 'hEEFFAABBCAFECAFE; // for debug purpose
		m_axis_rr_desc_tkeep = 0;
		m_axis_rr_desc_tlast = 0;
		m_axis_rr_desc_tvalid = 0;

		case(ap_state)
			AP_STATE_DMA_RD: begin
				m_axis_rr_desc_tvalid = 1;

				case(tlp_hdr_idx)
					0: begin
						m_axis_rr_desc_tlast = 0;
						m_axis_rr_desc_tdata = {      // Bits
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
						m_axis_rr_desc_tkeep = 8'hFF;
					end

					1: begin
						m_axis_rr_desc_tlast = 1;

						if(addr_32bit) begin
							m_axis_rr_desc_tdata = {
								// DW3
								32'b0,
								// DW2 Addr LO
								{addr_int[31:2], 2'b00}
							};
							m_axis_rr_desc_tkeep = 8'h0F;
						end else begin
							m_axis_rr_desc_tdata = {
								// DW3 Addr LO
								{addr_int[31:2], 2'b00},
								// DW2 Addr HI
								addr_int[63:32]
							};
							m_axis_rr_desc_tkeep = 8'hFF;
						end
					end
				endcase
			end
		endcase
	end

	assign m_axis_rr_desc_fire = m_axis_rr_desc_tready && m_axis_rr_desc_tvalid;

	// @COMB s_axis_rc_desc_tready
	always @(*) begin
		s_axis_rc_desc_tready = 0;

		case(ap_state)
			AP_STATE_RX: begin
				s_axis_rc_desc_tready = 1;
			end
		endcase
	end

	assign s_axis_rc_desc_fire = s_axis_rc_desc_tready && s_axis_rc_desc_tvalid;

`ifdef USE_FIFO_FWFT
	fifo_fwft #(
		.DATA_WIDTH(DESC_WIDTH),
		.DEPTH((C_MAX_BURST_SIZE / BYTES_PER_DESC) * 2)
	) fifo_xdma_desc_U(
		.clk(clk),
		.rst_n(rst_n),
		.wr_en(fifo_xdma_desc_wr_en),
		.wr_data(fifo_xdma_desc_wr_data),
		.rd_en(rd_en),
		.rd_data(rd_data),
		.full(fifo_xdma_desc_full),
		.empty(fifo_xdma_desc_empty),
		.data_valid(data_valid)
	);
`else // USE_FIFO_FWFT
	fifo_xdma_desc fifo_xdma_desc_U(
		.clk(clk),
		.srst(~rst_n),

		.wr_en(fifo_xdma_desc_wr_en),
		.din(fifo_xdma_desc_wr_data),
		.full(fifo_xdma_desc_full),

		.rd_en(rd_en),
		.dout(rd_data),
		.valid(data_valid),
		.empty(fifo_xdma_desc_empty)
	);
`endif // USE_FIFO_FWFT

	// @COMB fifo_xdma_desc_wr_en, @COMB fifo_xdma_desc_wr_data
	always @(*) begin
		fifo_xdma_desc_wr_en = 0;
		fifo_xdma_desc_wr_data = 'hEEFFAABBCAFECAFE;

		case(ap_state)
			AP_STATE_WR_FIFO: begin
				if(! fifo_xdma_desc_full) begin
					fifo_xdma_desc_wr_en = 1;
					fifo_xdma_desc_wr_data[64 +: 32] = desc_bytes;

					if(C_DMA_DIR == DMA_FROM_DEVICE) begin
						fifo_xdma_desc_wr_data[63:0] = {desc_dst_addr_hi, desc_dst_addr_lo};
					end else if(C_DMA_DIR == DMA_TO_DEVICE) begin
						fifo_xdma_desc_wr_data[63:0] = {desc_src_addr_hi, desc_src_addr_lo};
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
			AP_STATE_WR_FIFO: begin
				if(! fifo_xdma_desc_full) begin
					if((desc_control[16 +: 16] != DESC_MAGIC)) begin
						ERR_MASK_next = ERR_MASK_next | ERR_MASK_DESC_MAGIC;
					end

					if(desc_bytes == 0) begin
						ERR_MASK_next = ERR_MASK_next | ERR_MASK_DESC_BYTES;
					end
				end
			end
		endcase
	end
endmodule
