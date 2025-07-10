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

	// RC TLP
	input [C_DATA_WIDTH-1:0]    m_axis_rc_tdata,
	input [KEEP_WIDTH-1:0]      m_axis_rc_tkeep,
	input                       m_axis_rc_tlast,
	input                       m_axis_rc_tvalid,
	output                      m_axis_rc_tready,
	input [C_RC_USER_WIDTH-1:0] m_axis_rc_tuser,

	// RR TLP
	output reg [C_DATA_WIDTH-1:0] s_axis_rr_tdata,
	output reg [KEEP_WIDTH-1:0]   s_axis_rr_tkeep,
	output reg                    s_axis_rr_tlast,
	output reg                    s_axis_rr_tvalid,
	input                         s_axis_rr_tready,
	output [C_RR_USER_WIDTH-1:0]  s_axis_rr_tuser,

	// M00 SRC
	input [C_SRC_DATA_WIDTH-1:0] m00_axis_tdata,
	input [SRC_KEEP_WIDTH-1:0]   m00_axis_tkeep,
	input                        m00_axis_tlast,
	input                        m00_axis_tvalid,
	output                       m00_axis_tready,
	input                        m00_axis_tuser,

	// M01 SRC
	input [C_SRC_DATA_WIDTH-1:0] m01_axis_tdata,
	input [SRC_KEEP_WIDTH-1:0]   m01_axis_tkeep,
	input                        m01_axis_tlast,
	input                        m01_axis_tvalid,
	output                       m01_axis_tready,
	input                        m01_axis_tuser,

	input [15:0]             cfg_completer_id,
	input [C_DATA_WIDTH-1:0] tlp_hdr_dw1_0,

	// ap params
	input [63:0] buf_addr,
	input [31:0] size,
	input [31:0] times,

	// ap ctrl
	input  ap_start,
	output ap_done,
	output ap_ready,
	output ap_idle
);
	localparam TLP_MEM_WR32_FMT_TYPE = 7'b10_00000;
	localparam TLP_MEM_WR64_FMT_TYPE = 7'b11_00000;

	localparam BURST_SIZE  = 256; // bytes per burst (FIFO depth 256 >> 2 = 64)
	localparam BURST_WIDTH = $clog2(BURST_SIZE + 1);

	localparam MAX_SIZE   = 4096;
	localparam SIZE_WIDTH = $clog2(MAX_SIZE + 1);

	localparam MAX_TIMES   = 4320;
	localparam TIMES_WIDTH = $clog2(MAX_TIMES + 1);

	localparam STATE_IDLE            = 3'd0;
	localparam STATE_FINISH          = 3'd1;
	localparam STATE_DMA_WR_DW1_0    = 3'd2;
	localparam STATE_DMA_WR_DW3_2    = 3'd3;
	localparam STATE_DMA_WR          = 3'd4;
	localparam STATE_DMA_WR_DONE     = 3'd5;
	localparam STATE_BITS            = 3;

	localparam SRC_COUNT     = 2;
	localparam SRC_CNT_WIDTH = $clog2(SRC_COUNT);

	localparam SRC_STATE_IDLE   = 3'd0;
	localparam SRC_STATE_FINISH = 3'd1;
	localparam SRC_STATE_RX_DW0 = 3'd2;
	localparam SRC_STATE_RX_DW1 = 3'd3;
	localparam SRC_STATE_BITS   = 3;

	genvar gen_i;
	integer i;

	reg [STATE_BITS-1:0]    ap_state;
	reg [63:0]              buf_addr_int;
	wire [63:0]             buf_addr_next;
	reg [SIZE_WIDTH-1:0]    buf_addr_adder_inc;
	reg                     buf_addr_32bit;
	reg [SIZE_WIDTH-1:0]    size_int, size_next;
	reg [TIMES_WIDTH-1:0]   times_int;
	reg [BURST_WIDTH-1:0]   burst_bytes_int;
	reg [SRC_CNT_WIDTH-1:0] src_sel;
	wire [31:0]             size_4x;
	wire [31:0]             buf_addr_lo;
	wire [31:0]             buf_addr_hi;

	wire s_axis_rr_fire;

	// RX TLP DW1_DW0
	wire [6:0]  rx_tlp_hdr_fmt_type;
	wire [2:0]  rx_tlp_hdr_tc;
	wire        rx_tlp_hdr_td;
	wire        rx_tlp_hdr_ep;
	wire [1:0]  rx_tlp_hdr_attr;
	wire [9:0]  rx_tlp_hdr_len;
	wire [15:0] rx_tlp_hdr_rid;
	wire [7:0]  rx_tlp_hdr_tag;
	wire [7:0]  rx_tlp_hdr_be;

	wire [7:0] tlp_hdr_tag;
	wire [3:0] tlp_hdr_last_be;
	wire [3:0] tlp_hdr_first_be;
	wire [6:0] tlp_hdr_fmt_type;
	wire [2:0] tlp_hdr_tc;
	wire       tlp_hdr_td;
	wire       tlp_hdr_ep;
	wire [1:0] tlp_hdr_attr;
	wire [9:0] tlp_hdr_len;

	wire [C_SRC_DATA_WIDTH-1:0] m_axis_tdata [SRC_COUNT-1:0];
	reg                         m_axis_tready [SRC_COUNT-1:0];
	wire                        m_axis_tvalid [SRC_COUNT-1:0];
	wire                        m_axis_fire [SRC_COUNT-1:0];

	// m_fifo_U[SRC_COUNT]
	reg [SRC_STATE_BITS-1:0]   m_state [SRC_COUNT-1:0];
	reg [C_SRC_DATA_WIDTH-1:0] m_dw0 [SRC_COUNT-1:0];
	reg                        m_fifo_wr_en [SRC_COUNT-1:0];
	wire [C_DATA_WIDTH-1:0]    m_fifo_wr_data [SRC_COUNT-1:0];
	reg                        m_fifo_rd_en [SRC_COUNT-1:0];
	wire [C_DATA_WIDTH-1:0]    m_fifo_rd_data [SRC_COUNT-1:0];
	wire                       m_fifo_data_valid [SRC_COUNT-1:0];
	wire                       m_fifo_full [SRC_COUNT-1:0];

	assign ap_idle = (ap_state == STATE_IDLE);
	assign ap_done = (ap_state == STATE_FINISH);
	assign ap_ready = ap_done;

	assign m_axis_rc_tready = 1'b0; // Never receive RC

	assign m_axis_tdata[0] = m00_axis_tdata;
	assign m00_axis_tready = m_axis_tready[0];
	assign m_axis_tvalid[0] = m00_axis_tvalid;

	assign m_axis_tdata[1] = m01_axis_tdata;
	assign m01_axis_tready = m_axis_tready[1];
	assign m_axis_tvalid[1] = m01_axis_tvalid;

	assign rx_tlp_hdr_fmt_type = tlp_hdr_dw1_0[30:24];
	assign rx_tlp_hdr_tc = tlp_hdr_dw1_0[22:20];
	assign rx_tlp_hdr_td = tlp_hdr_dw1_0[15];
	assign rx_tlp_hdr_ep = tlp_hdr_dw1_0[14];
	assign rx_tlp_hdr_attr = tlp_hdr_dw1_0[13:12];
	assign rx_tlp_hdr_len = tlp_hdr_dw1_0[9:0];
	assign rx_tlp_hdr_rid = tlp_hdr_dw1_0[63:48];
	assign rx_tlp_hdr_tag = tlp_hdr_dw1_0[47:40];
	assign rx_tlp_hdr_be = tlp_hdr_dw1_0[39:32];

	assign tlp_hdr_tag = 8'd0; // for tlp_demuxer/s00_axis_rc
	assign tlp_hdr_last_be = 4'b1111;
	assign tlp_hdr_first_be = 4'b1111;
	assign tlp_hdr_fmt_type = (buf_addr_32bit ? TLP_MEM_WR32_FMT_TYPE : TLP_MEM_WR64_FMT_TYPE);
	assign tlp_hdr_tc = 3'b0;
	assign tlp_hdr_td = 1'b0;
	assign tlp_hdr_ep = 1'b0;
	assign tlp_hdr_attr = 2'b0;
	assign tlp_hdr_len = (burst_bytes_int >> 2); // measured by DW unit

	assign size_4x = (size & ~32'b11);
	assign buf_addr_lo = {buf_addr_int[31:2], 2'b00};
	assign buf_addr_hi = buf_addr_int[63:32];

	buf_addr_adder #(
		.C_INC_WIDTH(SIZE_WIDTH)
	) buf_addr_adder_U (
		.clk(clk),
		.rst_n(rst_n),
		.buf_addr(buf_addr_int),
		.inc(buf_addr_adder_inc),
		.buf_addr_next(buf_addr_next)
	);

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
							if(m_axis_fire[gen_i]) begin
								m_dw0[gen_i] <= m_axis_tdata[gen_i];
								m_state[gen_i] <= SRC_STATE_RX_DW1;
							end

						SRC_STATE_RX_DW1:
							if(m_axis_fire[gen_i]) begin
								m_state[gen_i] <= SRC_STATE_RX_DW0;
							end
					endcase
				end
			end

			// @COMB m_axis_tready[gen_i], @COMB m_fifo_wr_en[gen_i]
			always @(*) begin
				m_axis_tready[gen_i] = 0;
				m_fifo_wr_en[gen_i] = 0;

				case(m_state[gen_i])
					SRC_STATE_RX_DW0: begin
						m_axis_tready[gen_i] = 1;
					end

					SRC_STATE_RX_DW1: begin
						m_axis_tready[gen_i] = !m_fifo_full[gen_i];
						m_fifo_wr_en[gen_i] = m_axis_fire[gen_i];
					end
				endcase
			end

			assign m_fifo_wr_data[gen_i] = { m_axis_tdata[gen_i], m_dw0[gen_i] };
			assign m_axis_fire[gen_i] = m_axis_tready[gen_i] && m_axis_tvalid[gen_i];
		end
	endgenerate

	// @FF ap_state
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= STATE_IDLE;
		end else begin
			case(ap_state)
				STATE_IDLE:
					if(ap_start) begin
						ap_state <= STATE_DMA_WR_DONE;
					end

				STATE_DMA_WR_DW1_0:
					if(s_axis_rr_fire) begin
						ap_state <= STATE_DMA_WR_DW3_2;
					end

				STATE_DMA_WR_DW3_2: begin
					if(s_axis_rr_fire) begin
						ap_state <= STATE_DMA_WR;

						if(buf_addr_32bit) begin
							if(s_axis_rr_tlast) begin
								ap_state <= STATE_DMA_WR_DONE;

								if(size_next == 0 && times_int == 1)
									ap_state <= STATE_FINISH;
							end
						end
					end
				end

				STATE_DMA_WR: begin
					if(s_axis_rr_fire) begin
						if(s_axis_rr_tlast) begin
							ap_state <= STATE_DMA_WR_DONE;

							if(size_next == 0 && times_int == 1)
								ap_state <= STATE_FINISH;
						end
					end
				end

				STATE_DMA_WR_DONE: begin
					ap_state <= STATE_DMA_WR_DW1_0;
				end

				STATE_FINISH: begin
					ap_state <= STATE_IDLE;
				end
			endcase
		end
	end

	// @FF buf_addr_int, @FF buf_addr_adder_inc, @FF size_int, @FF size_next, @FF times_int, @FF burst_bytes_int, @FF src_sel
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			buf_addr_int <= 0;
			buf_addr_32bit <= 0;
			buf_addr_adder_inc <= 0;
			size_int <= 0;
			size_next <= 0;
			times_int <= 0;
			burst_bytes_int <= 0;
			src_sel <= 0;
		end else begin
			case(ap_state)
				STATE_IDLE:
					if(ap_start) begin
						buf_addr_int <= buf_addr;
						buf_addr_32bit <= (buf_addr[63:32] == 32'b0);
						size_int <= size_4x;
						times_int <= times;
					end

				STATE_DMA_WR_DW3_2: begin
					if(buf_addr_32bit) begin
						if(s_axis_rr_fire) begin
							burst_bytes_int <= burst_bytes_int - 4;

							if(s_axis_rr_tlast) begin
								buf_addr_int <= buf_addr_next;
								size_int <= size_next;

								if(size_next == 0) begin
									times_int <= times_int - 1;

									buf_addr_int <= buf_addr;
									size_int <= size_4x;
								end
							end
						end
					end
				end

				STATE_DMA_WR: begin
					if(s_axis_rr_fire) begin
						burst_bytes_int <= ((burst_bytes_int == 4) ? 0 : (burst_bytes_int - 8));

						if(s_axis_rr_tlast) begin
							buf_addr_int <= buf_addr_next;
							size_int <= size_next;

							if(size_next == 0) begin
								times_int <= times_int - 1;

								buf_addr_int <= buf_addr;
								size_int <= size_4x;
							end
						end
					end
				end

				STATE_DMA_WR_DONE: begin
					if(size_int > BURST_SIZE) begin
						buf_addr_adder_inc <= BURST_SIZE;
						size_next <= size_int - BURST_SIZE;
						burst_bytes_int <= BURST_SIZE;
					end else begin
						buf_addr_adder_inc <= size_int;
						size_next <= 0;
						burst_bytes_int <= size_int;
					end

					src_sel <= ((src_sel == 0) ? 1 : 0);
				end
			endcase
		end
	end

	// @COMB s_axis_rr_tdata, @COMB s_axis_rr_tkeep, @COMB s_axis_rr_tlast, @COMB s_axis_rr_tvalid, @COMB m_fifo_rd_en[SRC_COUNT]
	always @(*) begin
		s_axis_rr_tdata = 'hAABBCCDDCAFECAFE; // for debug purpose
		s_axis_rr_tkeep = 0;
		s_axis_rr_tlast = 0;
		s_axis_rr_tvalid = 0;

		for(i = 0;i < SRC_COUNT;i = i + 1) begin
			m_fifo_rd_en[i] = 0;
		end

		case(ap_state)
			STATE_DMA_WR_DW1_0: begin
				s_axis_rr_tlast = 0;
				s_axis_rr_tdata = {           // Bits
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
				s_axis_rr_tkeep = 8'hFF;
				s_axis_rr_tvalid = 1;
			end

			STATE_DMA_WR_DW3_2: begin
				if(buf_addr_32bit) begin
					s_axis_rr_tlast = (burst_bytes_int == 4);
					s_axis_rr_tkeep = 8'hFF;

					s_axis_rr_tdata = {
						// DW3 Data
						{m_fifo_rd_data[src_sel][7:0], m_fifo_rd_data[src_sel][15:8], m_fifo_rd_data[src_sel][23:16], m_fifo_rd_data[src_sel][31:24]},
						// DW2 Addr
						buf_addr_lo
					};
					s_axis_rr_tvalid = m_fifo_data_valid[src_sel];
					m_fifo_rd_en[src_sel] = s_axis_rr_fire;
				end else begin
					s_axis_rr_tlast = 0;
					s_axis_rr_tdata = {
						// DW3 Addr LO
						buf_addr_lo,
						// DW2 Addr HI
						buf_addr_hi
					};
					s_axis_rr_tkeep = 8'hFF;
					s_axis_rr_tvalid = 1;
				end
			end

			STATE_DMA_WR: begin
				s_axis_rr_tlast = (burst_bytes_int == 4 || burst_bytes_int == 8);
				s_axis_rr_tkeep = (burst_bytes_int == 4 ? 8'h0F : 8'hFF);

				s_axis_rr_tdata = m_fifo_rd_data[src_sel];
				s_axis_rr_tvalid = m_fifo_data_valid[src_sel];
				m_fifo_rd_en[src_sel] = s_axis_rr_fire;
			end
		endcase
	end

	assign s_axis_rr_tuser[0] = 1'b0; // Unused for V6
	assign s_axis_rr_tuser[1] = 1'b0; // Error forward packet
	assign s_axis_rr_tuser[2] = 1'b0; // Stream packet
	assign s_axis_rr_tuser[3] = 1'b0; // Unused discontinue
	assign s_axis_rr_fire = s_axis_rr_tready && s_axis_rr_tvalid;
endmodule
