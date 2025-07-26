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
	parameter C_RC_USER_WIDTH  = 22,
	parameter C_RR_USER_WIDTH  = 4,
	parameter C_RC_TAG         = 8'hF0, // tag base for tlp_demuxer/m00_axis_desc_rc
	parameter C_DMA_DIR        = 0, // 0 => DMA_FROM_DEVICE, 1 => DMA_TO_DEVICE

	// Do not override parameters below this line
	parameter KEEP_WIDTH = C_DATA_WIDTH / 8,
	parameter DESC_WIDTH = 32+32+32 // {bytes, addr_hi, addr_lo}
) (
	input clk,
	input rst_n,

	// RC DESC TLP
	input [C_DATA_WIDTH-1:0]    s_axis_rc_desc_tdata,
	input [KEEP_WIDTH-1:0]      s_axis_rc_desc_tkeep,
	input                       s_axis_rc_desc_tlast,
	input                       s_axis_rc_desc_tvalid,
	output reg                  s_axis_rc_desc_tready,
	input [C_RC_USER_WIDTH-1:0] s_axis_rc_desc_tuser,

	// RR DESC TLP
	output reg [C_DATA_WIDTH-1:0] m_axis_rr_desc_tdata,
	output reg [KEEP_WIDTH-1:0]   m_axis_rr_desc_tkeep,
	output reg                    m_axis_rr_desc_tlast,
	output reg                    m_axis_rr_desc_tvalid,
	input                         m_axis_rr_desc_tready,
	output [C_RR_USER_WIDTH-1:0]  m_axis_rr_desc_tuser,

	input [15:0] cfg_completer_id,

	// FIFO RD
	input                    rd_en,
	output [DESC_WIDTH-1:0]  rd_data,
	output                   data_valid,

	// ap params
	input [63:0]      DESC_ADDR,
	input [15:0]      DESC_ADJ,
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

	localparam AP_STATE_IDLE         = 3'd0;
	localparam AP_STATE_FINISH       = 3'd1;
	localparam AP_STATE_DMA_RD_DW1_0 = 3'd2;
	localparam AP_STATE_DMA_RD_DW3_2 = 3'd3;
	localparam AP_STATE_RX_DW1_0     = 3'd4;
	localparam AP_STATE_RX_DW3_2     = 3'd5;
	localparam AP_STATE_WR_FIFO      = 3'd6;
	localparam AP_STATE_ERROR        = 3'b111;
	localparam AP_STATE_WIDTH        = 3;

	localparam DESC_BYTES = 8 << 2; // 8 DWs
	localparam DESC_MAGIC = 16'hAD4B;
	localparam DMA_FROM_DEVICE = 0;
	localparam DMA_TO_DEVICE = 1;

	reg [AP_STATE_WIDTH-1:0] ap_state;
	reg [31:0]               ERR_MASK_next;
	reg [63:0]               desc_addr_int;
	wire [31:0]              desc_addr_lo;
	wire [31:0]              desc_addr_hi;
	reg                      desc_addr_32bit;
	reg [15:0]               desc_adj_int;

	// RX TLP DW1_0
	reg [C_DATA_WIDTH-1:0] rx_tlp_hdr_dw1_0;
	wire [6:0]             rx_tlp_hdr_fmt_type;
	wire [2:0]             rx_tlp_hdr_tc;
	wire                   rx_tlp_hdr_td;
	wire                   rx_tlp_hdr_ep;
	wire [1:0]             rx_tlp_hdr_attr;
	wire [9:0]             rx_tlp_hdr_len;
	wire [15:0]            rx_tlp_hdr_rid;
	wire [7:0]             rx_tlp_hdr_tag;
	wire [7:0]             rx_tlp_hdr_be;

	// TX TLP DW1_0
	wire [6:0] tlp_hdr_fmt_type;
	wire [2:0] tlp_hdr_tc;
	wire       tlp_hdr_td;
	wire       tlp_hdr_ep;
	wire [1:0] tlp_hdr_attr;
	wire [9:0] tlp_hdr_len;
	wire [7:0] tlp_hdr_tag;
	wire [3:0] tlp_hdr_last_be;
	wire [3:0] tlp_hdr_first_be;

	wire m_axis_rr_desc_fire;
	wire s_axis_rc_desc_fire;

	reg [2:0]  desc_inc;
	reg [31:0] desc_control;
	reg [31:0] desc_bytes;
	reg [31:0] desc_src_addr_lo;
	reg [31:0] desc_src_addr_hi;
	reg [31:0] desc_dst_addr_lo;
	reg [31:0] desc_dst_addr_hi;
	reg [31:0] desc_next_lo;
	reg [31:0] desc_next_hi;
	wire [63:0] desc_dw0;
	wire [63:0] desc_dw1;
	wire [63:0] desc_addr;

	// fifo_xdma_desc_U signals
	reg                  fifo_xdma_desc_wr_en;
	reg [DESC_WIDTH-1:0] fifo_xdma_desc_wr_data;
	wire                 fifo_xdma_desc_full;

	assign ap_idle = (ap_state == AP_STATE_IDLE);
	assign ap_done = (ap_state == AP_STATE_FINISH);
	assign ap_ready = ap_done;

	assign desc_addr_lo = {desc_addr_int[31:2], 2'b00};
	assign desc_addr_hi = desc_addr_int[63:32];

	assign rx_tlp_hdr_fmt_type = rx_tlp_hdr_dw1_0[30:24];
	assign rx_tlp_hdr_tc = rx_tlp_hdr_dw1_0[22:20];
	assign rx_tlp_hdr_td = rx_tlp_hdr_dw1_0[15];
	assign rx_tlp_hdr_ep = rx_tlp_hdr_dw1_0[14];
	assign rx_tlp_hdr_attr = rx_tlp_hdr_dw1_0[13:12];
	assign rx_tlp_hdr_len = rx_tlp_hdr_dw1_0[9:0];
	assign rx_tlp_hdr_rid = rx_tlp_hdr_dw1_0[63:48];
	assign rx_tlp_hdr_tag = rx_tlp_hdr_dw1_0[47:40];
	assign rx_tlp_hdr_be = rx_tlp_hdr_dw1_0[39:32];

	assign tlp_hdr_fmt_type = (desc_addr_32bit ? TLP_MEM_RD32_FMT_TYPE : TLP_MEM_RD64_FMT_TYPE);
	assign tlp_hdr_tc = 3'b0;
	assign tlp_hdr_td = 1'b0;
	assign tlp_hdr_ep = 1'b0;
	assign tlp_hdr_attr = 2'b0;
	assign tlp_hdr_len = (DESC_BYTES >> 2); // measured by DW unit
	assign tlp_hdr_tag = C_RC_TAG;
	assign tlp_hdr_last_be = 4'b1111;
	assign tlp_hdr_first_be = 4'b1111;

	assign desc_dw0 = {s_axis_rc_desc_tdata[0*8 +: 8], s_axis_rc_desc_tdata[1*8 +: 8], s_axis_rc_desc_tdata[2*8 +: 8], s_axis_rc_desc_tdata[3*8 +: 8]};
	assign desc_dw1 = {s_axis_rc_desc_tdata[4*8 +: 8], s_axis_rc_desc_tdata[5*8 +: 8], s_axis_rc_desc_tdata[6*8 +: 8], s_axis_rc_desc_tdata[7*8 +: 8]};

	generate
		if(C_DMA_DIR == DMA_FROM_DEVICE) begin
			assign desc_addr = {desc_dst_addr_hi, desc_dst_addr_lo};
		end else if(C_DMA_DIR == DMA_TO_DEVICE) begin
			assign desc_addr = {desc_src_addr_hi, desc_src_addr_lo};
		end
	endgenerate

	// @FF ap_state, @FF desc_inc, @FF desc_control, @FF desc_bytes, @FF desc_src_addr_lo, @FF desc_src_addr_hi,
	// @FF desc_dst_addr_lo, @FF desc_dst_addr_hi, @FF desc_next_lo, @FF desc_next_hi
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_IDLE;
			desc_inc <= 0;
			desc_control <= 0;
			desc_bytes <= 0;
			desc_src_addr_lo <= 0;
			desc_src_addr_hi <= 0;
			desc_dst_addr_lo <= 0;
			desc_dst_addr_hi <= 0;
			desc_next_lo <= 0;
			desc_next_hi <= 0;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					if(ap_start) begin
						ap_state <= AP_STATE_DMA_RD_DW1_0;
					end
				end

				AP_STATE_DMA_RD_DW1_0: begin
					if(m_axis_rr_desc_fire) begin
						ap_state <= AP_STATE_DMA_RD_DW3_2;
					end
				end

				AP_STATE_DMA_RD_DW3_2: begin
					if(m_axis_rr_desc_fire) begin
						ap_state <= AP_STATE_RX_DW1_0;
						desc_inc <= 0;
					end
				end

				AP_STATE_RX_DW1_0: begin
					if(s_axis_rc_desc_fire) begin
						ap_state <= AP_STATE_RX_DW3_2;
					end
				end

				AP_STATE_RX_DW3_2: begin
					if(s_axis_rc_desc_fire) begin
						case(desc_inc)
							0: begin
								desc_control <= desc_dw1;
							end
							1: begin
								desc_bytes <= desc_dw0;
								desc_src_addr_lo <= desc_dw1;
							end
							2: begin
								desc_src_addr_hi <= desc_dw0;
								desc_dst_addr_lo <= desc_dw1;
							end
							3: begin
								desc_dst_addr_hi <= desc_dw0;
								desc_next_lo <= desc_dw1;
							end
							4: begin
								desc_next_hi <= desc_dw0;
								ap_state <= AP_STATE_WR_FIFO;
							end
						endcase

						desc_inc <= desc_inc + 1;
					end
				end

				AP_STATE_WR_FIFO: begin
					if(! fifo_xdma_desc_full) begin
						if(desc_adj_int == 0) begin
							ap_state <= AP_STATE_FINISH;
						end else begin
							ap_state <= AP_STATE_DMA_RD_DW1_0;
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

	// rx_tlp_hdr_dw1_0
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			rx_tlp_hdr_dw1_0 <= 0;
		end else begin
			case(ap_state)
				AP_STATE_RX_DW1_0: begin
					if(s_axis_rc_desc_fire) begin
						rx_tlp_hdr_dw1_0 <= s_axis_rc_desc_tdata;
					end
				end
			endcase
		end
	end

	// @FF desc_addr_int, @FF desc_addr_32bit, @FF desc_adj_int
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			desc_addr_int <= 0;
			desc_addr_32bit <= 0;
			desc_adj_int <= 0;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					if(ap_start) begin
						desc_addr_int <= DESC_ADDR;
						desc_addr_32bit <= (DESC_ADDR[63:32] == 32'b0);
						desc_adj_int <= DESC_ADJ;
					end
				end

				AP_STATE_WR_FIFO: begin
					if(! fifo_xdma_desc_full) begin
						desc_addr_int <= {desc_next_hi, desc_next_lo};
						desc_adj_int <= desc_adj_int - 1;
					end
				end
			endcase
		end
	end

	// @COMB m_axis_rr_desc_tdata, @COMB m_axis_rr_desc_tkeep, @COMB m_axis_rr_desc_tlast, @COMB m_axis_rr_desc_tvalid
	always @(*) begin
		m_axis_rr_desc_tdata = 'hEEFFAABBCAFECAFE; // for debug purpose
		m_axis_rr_desc_tkeep = 0;
		m_axis_rr_desc_tlast = 0;
		m_axis_rr_desc_tvalid = 0;

		case(ap_state)
			AP_STATE_DMA_RD_DW1_0: begin
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
				m_axis_rr_desc_tvalid = 1;
			end

			AP_STATE_DMA_RD_DW3_2: begin
				m_axis_rr_desc_tlast = 1;
				m_axis_rr_desc_tvalid = 1;

				if(desc_addr_32bit) begin
					m_axis_rr_desc_tdata = {
						// DW3
						32'b0,
						// DW2 Addr
						desc_addr_lo
					};
					m_axis_rr_desc_tkeep = 8'h0F;
				end else begin
					m_axis_rr_desc_tdata = {
						// DW3 Addr LO
						desc_addr_lo,
						// DW2 Addr HI
						desc_addr_hi
					};
					m_axis_rr_desc_tkeep = 8'hFF;
				end
			end
		endcase
	end

	assign m_axis_rr_desc_tuser[0] = 1'b0; // Unused for V6
	assign m_axis_rr_desc_tuser[1] = 1'b0; // Error forward packet
	assign m_axis_rr_desc_tuser[2] = 1'b0; // Stream packet
	assign m_axis_rr_desc_tuser[3] = 1'b0; // Unused discontinue
	assign m_axis_rr_desc_fire = m_axis_rr_desc_tready && m_axis_rr_desc_tvalid;

	// @COMB s_axis_rc_desc_tready
	always @(*) begin
		s_axis_rc_desc_tready = 0;

		case(ap_state)
			AP_STATE_RX_DW1_0, AP_STATE_RX_DW3_2: begin
				s_axis_rc_desc_tready = 1;
			end
		endcase
	end

	assign s_axis_rc_desc_fire = s_axis_rc_desc_tready && s_axis_rc_desc_tvalid;

	fifo_xdma_desc fifo_xdma_desc_U(
		.clk(clk),
		.srst(~rst_n),

		.wr_en(fifo_xdma_desc_wr_en),
		.din(fifo_xdma_desc_wr_data),
		.full(fifo_xdma_desc_full),

		.rd_en(rd_en),
		.dout(rd_data),
		.valid(data_valid)
	);

	// @COMB fifo_xdma_desc_wr_en, @COMB fifo_xdma_desc_wr_data
	always @(*) begin
		fifo_xdma_desc_wr_en = 0;
		fifo_xdma_desc_wr_data = 'hEEFFAABBCAFECAFE;

		case(ap_state)
			AP_STATE_WR_FIFO: begin
				if(! fifo_xdma_desc_full) begin
					fifo_xdma_desc_wr_en = 1;
					fifo_xdma_desc_wr_data = {desc_bytes, desc_addr};
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
			AP_STATE_RX_DW3_2: begin
				if(s_axis_rc_desc_fire) begin
					case(desc_inc)
						4: begin
							if((desc_control[31:16] != DESC_MAGIC)) begin
								ERR_MASK_next = ERR_MASK_next | ERR_MASK_DESC_MAGIC;
							end

							if((desc_control[13:8] != desc_adj_int)) begin
								ERR_MASK_next = ERR_MASK_next | ERR_MASK_DESC_ADJ;
							end
						end
					endcase
				end
			end

			AP_STATE_WR_FIFO: begin
				if(! fifo_xdma_desc_full) begin
					if(desc_bytes == 0) begin
						ERR_MASK_next = ERR_MASK_next | ERR_MASK_DESC_BYTES;
					end

					if(desc_next_hi == 0 && desc_next_lo == 0 && desc_adj_int != 0) begin
						ERR_MASK_next = ERR_MASK_next | ERR_MASK_DESC_NEXT;
					end

					if(desc_next_hi != 0 && desc_next_lo != 0 && desc_adj_int == 0) begin
						ERR_MASK_next = ERR_MASK_next | ERR_MASK_DESC_END;
					end
				end
			end
		endcase
	end
endmodule
