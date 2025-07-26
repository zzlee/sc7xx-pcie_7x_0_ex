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
	parameter C_RC_USER_WIDTH  = 22,
	parameter C_RR_USER_WIDTH  = 4,
	parameter C_SRC_DATA_WIDTH = 32,

	// Do not override parameters below this line
	parameter KEEP_WIDTH     = C_DATA_WIDTH / 8,
	parameter SRC_KEEP_WIDTH = C_SRC_DATA_WIDTH / 8
) (
	input clk,
	input rst_n,

	// RC DESC TLP
	input [C_DATA_WIDTH-1:0]    s_axis_rc_desc_tdata,
	input [KEEP_WIDTH-1:0]      s_axis_rc_desc_tkeep,
	input                       s_axis_rc_desc_tlast,
	input                       s_axis_rc_desc_tvalid,
	output                      s_axis_rc_desc_tready,
	input [C_RC_USER_WIDTH-1:0] s_axis_rc_desc_tuser,

	// RR DESC TLP
	output [C_DATA_WIDTH-1:0]     m_axis_rr_desc_tdata,
	output [KEEP_WIDTH-1:0]       m_axis_rr_desc_tkeep,
	output                        m_axis_rr_desc_tlast,
	output                        m_axis_rr_desc_tvalid,
	input                         m_axis_rr_desc_tready,
	output [C_RR_USER_WIDTH-1:0]  m_axis_rr_desc_tuser,

	// RR TLP
	output reg [C_DATA_WIDTH-1:0] m_axis_rr_tdata,
	output reg [KEEP_WIDTH-1:0]   m_axis_rr_tkeep,
	output reg                    m_axis_rr_tlast,
	output reg                    m_axis_rr_tvalid,
	input                         m_axis_rr_tready,
	output [C_RR_USER_WIDTH-1:0]  m_axis_rr_tuser,

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
	input [63:0]      DESC_ADDR,
	input [31:0]      DESC_ADJ,
	input [31:0]      SIZE,
	output reg [31:0] ERR_MASK,

	// ap ctrl
	input  ap_start,
	output ap_done,
	output ap_ready,
	output ap_idle
);
	`include "err_mask.vh"

	localparam TLP_MEM_WR32_FMT_TYPE = 7'b10_00000;
	localparam TLP_MEM_WR64_FMT_TYPE = 7'b11_00000;

	localparam BURST_SIZE  = 256; // bytes per burst (FIFO depth 256 >> 2 = 64)
	localparam BURST_WIDTH = $clog2(BURST_SIZE + 1);

	localparam MAX_SIZE   = 4096 * 2160 * 4;
	localparam SIZE_WIDTH = $clog2(MAX_SIZE + 1);

	localparam DESC_WIDTH = 32+32+32; // {bytes, addr_hi, addr_lo}

	localparam AP_STATE_IDLE         = 3'd0;
	localparam AP_STATE_FINISH       = 3'd1;
	localparam AP_STATE_WAIT_DESC    = 3'd2;
	localparam AP_STATE_DMA_WR_DW1_0 = 3'd3;
	localparam AP_STATE_DMA_WR_DW3_2 = 3'd4;
	localparam AP_STATE_DMA_WR       = 3'd5;
	localparam AP_STATE_DMA_WR_NEXT  = 3'd6;
	localparam AP_STATE_ERROR        = 3'b111;
	localparam AP_STATE_WIDTH        = 3;

	localparam SRC_COUNT     = 2;
	localparam SRC_CNT_WIDTH = $clog2(SRC_COUNT);

	localparam SRC_STATE_IDLE   = 3'd0;
	localparam SRC_STATE_FINISH = 3'd1;
	localparam SRC_STATE_RX_DW0 = 3'd2;
	localparam SRC_STATE_RX_DW1 = 3'd3;
	localparam SRC_STATE_WIDTH  = 3;

	genvar gen_i;
	integer i;

	reg [AP_STATE_WIDTH-1:0]    ap_state;
	reg [31:0]              ERR_MASK_next;
	reg [63:0]              buf_addr_int;
	wire [63:0]             buf_addr_next;
	wire [31:0]             buf_addr_lo;
	wire [31:0]             buf_addr_hi;
	reg                     buf_addr_32bit;
	reg [SIZE_WIDTH-1:0]    addr_adder_inc;
	reg [31:0]              buf_size_int, buf_size_next;
	reg [SIZE_WIDTH-1:0]    size_int, size_next;
	reg [BURST_WIDTH-1:0]   burst_bytes_int;
	reg [SRC_CNT_WIDTH-1:0] src_sel;

	wire m_axis_rr_fire;

	wire [7:0] tlp_hdr_tag;
	wire [3:0] tlp_hdr_last_be;
	wire [3:0] tlp_hdr_first_be;
	wire [6:0] tlp_hdr_fmt_type;
	wire [2:0] tlp_hdr_tc;
	wire       tlp_hdr_td;
	wire       tlp_hdr_ep;
	wire [1:0] tlp_hdr_attr;
	wire [9:0] tlp_hdr_len;

	wire [C_SRC_DATA_WIDTH-1:0] s_axis_tdata [SRC_COUNT-1:0];
	reg                         s_axis_tready [SRC_COUNT-1:0];
	wire                        s_axis_tvalid [SRC_COUNT-1:0];
	wire                        s_axis_fire [SRC_COUNT-1:0];

	// m_fifo_U[SRC_COUNT]
	reg [SRC_STATE_WIDTH-1:0]  m_state [SRC_COUNT-1:0];
	reg [C_SRC_DATA_WIDTH-1:0] m_dw0 [SRC_COUNT-1:0];
	reg                        m_fifo_wr_en [SRC_COUNT-1:0];
	wire [C_DATA_WIDTH-1:0]    m_fifo_wr_data [SRC_COUNT-1:0];
	reg                        m_fifo_rd_en [SRC_COUNT-1:0];
	wire [C_DATA_WIDTH-1:0]    m_fifo_rd_data [SRC_COUNT-1:0];
	wire                       m_fifo_data_valid [SRC_COUNT-1:0];
	wire                       m_fifo_full [SRC_COUNT-1:0];

	// desc_rd_U signals
	reg                   desc_rd_rd_en;
	wire [DESC_WIDTH-1:0] desc_rd_rd_data;
	wire                  desc_rd_data_valid;
	reg                   desc_rd_ap_start;
	wire                  desc_rd_ap_ready;
	wire [31:0]           desc_rd_ERR_MASK;

	assign ap_idle = (ap_state == AP_STATE_IDLE);
	assign ap_done = (ap_state == AP_STATE_FINISH);
	assign ap_ready = ap_done;

	tlp_xdma_desc_rd #(
		.C_DATA_WIDTH(C_DATA_WIDTH),
		.C_RC_USER_WIDTH(C_RC_USER_WIDTH),
		.C_RR_USER_WIDTH(C_RR_USER_WIDTH),
		.C_RC_TAG(8'hF0), // tag base for tlp_demuxer/m00_axis_desc_rc
		.C_DMA_DIR(0) // DMA_FROM_DEVICE
	) desc_rd_U(
		.clk(clk),
		.rst_n(rst_n),

		.s_axis_rc_desc_tdata(s_axis_rc_desc_tdata),
		.s_axis_rc_desc_tkeep(s_axis_rc_desc_tkeep),
		.s_axis_rc_desc_tlast(s_axis_rc_desc_tlast),
		.s_axis_rc_desc_tvalid(s_axis_rc_desc_tvalid),
		.s_axis_rc_desc_tready(s_axis_rc_desc_tready),
		.s_axis_rc_desc_tuser(s_axis_rc_desc_tuser),

		.m_axis_rr_desc_tdata(m_axis_rr_desc_tdata),
		.m_axis_rr_desc_tkeep(m_axis_rr_desc_tkeep),
		.m_axis_rr_desc_tlast(m_axis_rr_desc_tlast),
		.m_axis_rr_desc_tvalid(m_axis_rr_desc_tvalid),
		.m_axis_rr_desc_tready(m_axis_rr_desc_tready),
		.m_axis_rr_desc_tuser(m_axis_rr_desc_tuser),

		.cfg_completer_id(cfg_completer_id),

		.rd_en(desc_rd_rd_en),
		.rd_data(desc_rd_rd_data),
		.data_valid(desc_rd_data_valid),

		.DESC_ADDR(DESC_ADDR),
		.DESC_ADJ(DESC_ADJ),
		.ERR_MASK(desc_rd_ERR_MASK),

		.ap_start(desc_rd_ap_start),
		.ap_done(),
		.ap_ready(desc_rd_ap_ready),
		.ap_idle()
	);

	assign s_axis_tdata[0] = s00_axis_tdata;
	assign s00_axis_tready = s_axis_tready[0];
	assign s_axis_tvalid[0] = s00_axis_tvalid;

	assign s_axis_tdata[1] = s01_axis_tdata;
	assign s01_axis_tready = s_axis_tready[1];
	assign s_axis_tvalid[1] = s01_axis_tvalid;

	assign tlp_hdr_tag = 8'd0; // Don't care
	assign tlp_hdr_last_be = 4'b1111;
	assign tlp_hdr_first_be = 4'b1111;
	assign tlp_hdr_fmt_type = (buf_addr_32bit ? TLP_MEM_WR32_FMT_TYPE : TLP_MEM_WR64_FMT_TYPE);
	assign tlp_hdr_tc = 3'b0;
	assign tlp_hdr_td = 1'b0;
	assign tlp_hdr_ep = 1'b0;
	assign tlp_hdr_attr = 2'b0;
	assign tlp_hdr_len = (burst_bytes_int >> 2); // measured by DW unit

	assign buf_addr_lo = {buf_addr_int[31:2], 2'b00};
	assign buf_addr_hi = buf_addr_int[63:32];

	generate
		for(gen_i = 0;gen_i < SRC_COUNT;gen_i = gen_i + 1) begin
			tlp_dma_wr_m_fifo m_fifo_U (
				.clk(clk),
				.srst(~rst_n),
				.wr_en(m_fifo_wr_en[gen_i]),
				.din(m_fifo_wr_data[gen_i]),
				.rd_en(m_fifo_rd_en[gen_i]),
				.dout(m_fifo_rd_data[gen_i]),
				.full(m_fifo_full[gen_i]),
				.empty(),
				.valid(m_fifo_data_valid[gen_i])
			);

			// @FF m_state[gen_i], m_dw0[gen_i]
			always @(posedge clk or negedge rst_n) begin
				if(~rst_n) begin
					m_state[gen_i] <= SRC_STATE_IDLE;
					m_dw0[gen_i] <= 0;
				end else begin
					case(m_state[gen_i])
						SRC_STATE_IDLE:
							m_state[gen_i] <= SRC_STATE_RX_DW0;

						SRC_STATE_RX_DW0:
							if(s_axis_fire[gen_i]) begin
								m_dw0[gen_i] <= s_axis_tdata[gen_i];
								m_state[gen_i] <= SRC_STATE_RX_DW1;
							end

						SRC_STATE_RX_DW1:
							if(s_axis_fire[gen_i]) begin
								m_state[gen_i] <= SRC_STATE_RX_DW0;
							end
					endcase
				end
			end

			// @COMB s_axis_tready[gen_i], @COMB m_fifo_wr_en[gen_i]
			always @(*) begin
				s_axis_tready[gen_i] = 0;
				m_fifo_wr_en[gen_i] = 0;

				case(m_state[gen_i])
					SRC_STATE_RX_DW0: begin
						s_axis_tready[gen_i] = 1;
					end

					SRC_STATE_RX_DW1: begin
						s_axis_tready[gen_i] = !m_fifo_full[gen_i];
						m_fifo_wr_en[gen_i] = s_axis_fire[gen_i];
					end
				endcase
			end

			assign m_fifo_wr_data[gen_i] = { s_axis_tdata[gen_i], m_dw0[gen_i] };
			assign s_axis_fire[gen_i] = s_axis_tready[gen_i] && s_axis_tvalid[gen_i];
		end
	endgenerate

	// @FF ap_state
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_IDLE;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					if(ap_start) begin
						ap_state <= AP_STATE_WAIT_DESC;
					end
				end

				AP_STATE_WAIT_DESC: begin
					if(desc_rd_data_valid) begin
						ap_state <= AP_STATE_DMA_WR_NEXT;
					end
				end

				AP_STATE_DMA_WR_DW1_0:
					if(m_axis_rr_fire) begin
						ap_state <= AP_STATE_DMA_WR_DW3_2;
					end

				AP_STATE_DMA_WR_DW3_2: begin
					if(m_axis_rr_fire) begin
						ap_state <= AP_STATE_DMA_WR;

						if(buf_addr_32bit) begin
							if(m_axis_rr_tlast) begin
								ap_state <= AP_STATE_DMA_WR_NEXT;

								if(size_next == 0) begin
									ap_state <= AP_STATE_FINISH;
								end else if(buf_size_next == 0) begin
									ap_state <= AP_STATE_WAIT_DESC;
								end
							end
						end
					end
				end

				AP_STATE_DMA_WR: begin
					if(m_axis_rr_fire) begin
						if(m_axis_rr_tlast) begin
							ap_state <= AP_STATE_DMA_WR_NEXT;

							if(size_next == 0) begin
								ap_state <= AP_STATE_FINISH;
							end else if(buf_size_next == 0) begin
								ap_state <= AP_STATE_WAIT_DESC;
							end
						end
					end
				end

				AP_STATE_DMA_WR_NEXT: begin
					ap_state <= AP_STATE_DMA_WR_DW1_0;
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

	addr_adder #(
		.C_INC_WIDTH(SIZE_WIDTH)
	) addr_adder_U (
		.clk(clk),
		.rst_n(rst_n),
		.addr(buf_addr_int),
		.inc(addr_adder_inc),
		.addr_next(buf_addr_next)
	);

	// @FF buf_addr_int, @FF addr_adder_inc, @FF size_int, @FF size_next, @FF burst_bytes_int
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			buf_addr_int <= 'hCAFE0001; // for debug purpose
			buf_addr_32bit <= 0;
			addr_adder_inc <= 0;
			size_int <= 0;
			size_next <= 0;
			burst_bytes_int <= 0;
			buf_size_int <= 'hCAFE0002; // for debug purpose
			buf_size_next <= 0;
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
						buf_size_int <= desc_rd_rd_data[64 +: 32];
					end
				end

				AP_STATE_DMA_WR_DW3_2: begin
					if(buf_addr_32bit) begin
						if(m_axis_rr_fire) begin
							burst_bytes_int <= burst_bytes_int - 4;

							if(m_axis_rr_tlast) begin
								buf_addr_int <= buf_addr_next;
								size_int <= size_next;
								buf_size_int <= buf_size_next;
							end
						end
					end
				end

				AP_STATE_DMA_WR: begin
					if(m_axis_rr_fire) begin
						burst_bytes_int <= ((burst_bytes_int == 4) ? 0 : (burst_bytes_int - 8));

						if(m_axis_rr_tlast) begin
							buf_addr_int <= buf_addr_next;
							size_int <= size_next;
							buf_size_int <= buf_size_next;
						end
					end
				end

				AP_STATE_DMA_WR_NEXT: begin
					if(buf_size_int > BURST_SIZE) begin
						addr_adder_inc <= BURST_SIZE;
						size_next <= size_int - BURST_SIZE;
						burst_bytes_int <= BURST_SIZE;
						buf_size_next <= buf_size_int - BURST_SIZE;
					end else begin
						addr_adder_inc <= buf_size_int;
						size_next <= size_int - buf_size_int;
						burst_bytes_int <= buf_size_int;
						buf_size_next <= 0;
					end
				end
			endcase
		end
	end

	// @FF src_sel
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			src_sel <= 0;
		end else begin
			case(ap_state)
				AP_STATE_DMA_WR_NEXT: begin
					src_sel <= ((src_sel == 0) ? 1 : 0);
				end
			endcase
		end
	end

	// @COMB m_axis_rr_tdata, @COMB m_axis_rr_tkeep, @COMB m_axis_rr_tlast, @COMB m_axis_rr_tvalid, @COMB m_fifo_rd_en[SRC_COUNT]
	always @(*) begin
		m_axis_rr_tdata = 'hAABBCCDDCAFECAFE; // for debug purpose
		m_axis_rr_tkeep = 0;
		m_axis_rr_tlast = 0;
		m_axis_rr_tvalid = 0;

		for(i = 0;i < SRC_COUNT;i = i + 1) begin
			m_fifo_rd_en[i] = 0;
		end

		case(ap_state)
			AP_STATE_DMA_WR_DW1_0: begin
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

			AP_STATE_DMA_WR_DW3_2: begin
				if(buf_addr_32bit) begin
					m_axis_rr_tlast = (burst_bytes_int == 4);
					m_axis_rr_tkeep = 8'hFF;

					m_axis_rr_tdata = {
						// DW3 Data
						{m_fifo_rd_data[src_sel][7:0], m_fifo_rd_data[src_sel][15:8], m_fifo_rd_data[src_sel][23:16], m_fifo_rd_data[src_sel][31:24]},
						// DW2 Addr
						buf_addr_lo
					};
					m_axis_rr_tvalid = m_fifo_data_valid[src_sel];
					m_fifo_rd_en[src_sel] = m_axis_rr_fire;
				end else begin
					m_axis_rr_tlast = 0;
					m_axis_rr_tdata = {
						// DW3 Addr LO
						buf_addr_lo,
						// DW2 Addr HI
						buf_addr_hi
					};
					m_axis_rr_tkeep = 8'hFF;
					m_axis_rr_tvalid = 1;
				end
			end

			AP_STATE_DMA_WR: begin
				m_axis_rr_tlast = (burst_bytes_int == 4 || burst_bytes_int == 8);
				m_axis_rr_tkeep = (burst_bytes_int == 4 ? 8'h0F : 8'hFF);

				m_axis_rr_tdata = m_fifo_rd_data[src_sel];
				m_axis_rr_tvalid = m_fifo_data_valid[src_sel];
				m_fifo_rd_en[src_sel] = m_axis_rr_fire;
			end
		endcase
	end

	assign m_axis_rr_tuser[0] = 1'b0; // Unused for V6
	assign m_axis_rr_tuser[1] = 1'b0; // Error forward packet
	assign m_axis_rr_tuser[2] = 1'b0; // Stream packet
	assign m_axis_rr_tuser[3] = 1'b0; // Unused discontinue
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

		case(ap_state)
			AP_STATE_DMA_WR_NEXT: begin
				if(buf_size_int > size_int) begin
					ERR_MASK_next = ERR_MASK_next | ERR_MASK_BUF_SIZE;
				end
			end
		endcase
	end
endmodule
