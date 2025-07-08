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
	parameter C_DATA_WIDTH = 64,
	parameter C_RC_USER_WIDTH = 22,
	parameter C_RR_USER_WIDTH = 4,
	parameter C_SINK_DATA_WIDTH = 64,

	// Do not override parameters below this line
	parameter KEEP_WIDTH = C_DATA_WIDTH / 8,
	parameter SINK_KEEP_WIDTH = C_SINK_DATA_WIDTH / 8
) (
	input clk,
	input rst_n,

	// RC00 TLP
	input [C_DATA_WIDTH-1:0]    m00_axis_rc_tdata,
	input [KEEP_WIDTH-1:0]      m00_axis_rc_tkeep,
	input                       m00_axis_rc_tlast,
	input                       m00_axis_rc_tvalid,
	output reg                  m00_axis_rc_tready,
	input [C_RC_USER_WIDTH-1:0] m00_axis_rc_tuser,

	// RR TLP
	output reg [C_DATA_WIDTH-1:0] s_axis_rr_tdata,
	output reg [KEEP_WIDTH-1:0]   s_axis_rr_tkeep,
	output reg                    s_axis_rr_tlast,
	output reg                    s_axis_rr_tvalid,
	input                         s_axis_rr_tready,
	output [C_RR_USER_WIDTH-1:0]  s_axis_rr_tuser,

	// S00 SINK
	output reg [C_SINK_DATA_WIDTH-1:0] s00_axis_tdata,
	output reg [SINK_KEEP_WIDTH-1:0]   s00_axis_tkeep,
	output                             s00_axis_tlast,
	output reg                         s00_axis_tvalid,
	input                              s00_axis_tready,
	output                             s00_axis_tuser,

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
	localparam TLP_MEM_RD32_FMT_TYPE = 7'b00_00000;
	localparam TLP_MEM_RD64_FMT_TYPE = 7'b01_00000;
	localparam TLP_CPLD_FMT_TYPE     = 7'b10_01010;

	localparam RC_TAG_BASE  = 8'h10; // tag base for tlp_demuxer/s01_axis_rc
	localparam RC_COUNT     = 15;
	localparam RC_CNT_WIDTH = 4;

	localparam BURST_SIZE  = 256; // bytes per burst
	localparam BURST_WIDTH = $clog2(BURST_SIZE + 1);

	localparam MAX_SIZE   = 4096;
	localparam SIZE_WIDTH = $clog2(MAX_SIZE + 1);

	localparam MAX_TIMES   = 4320;
	localparam TIMES_WIDTH = $clog2(MAX_TIMES + 1);

	localparam RC_FIFO_DATA_WIDTH = C_DATA_WIDTH + KEEP_WIDTH + 1;

	localparam STATE_IDLE            = 4'd0;
	localparam STATE_FINISH          = 4'd1;
	localparam STATE_DMA_RD_DW1_0    = 4'd2;
	localparam STATE_DMA_RD32_DW3_2  = 4'd3;
	localparam STATE_DMA_RD64_DW3_2  = 4'd4;
	localparam STATE_DMA_RD          = 4'd5;
	localparam STATE_DMA_RD_NEXT     = 4'd6;
	localparam STATE_WAIT_PENDING    = 4'd7;
	localparam STATE_BITS            = 4;

	localparam RC_STATE_IDLE      = 3'd0;
	localparam RC_STATE_MATCH_IDX = 3'd1;
	localparam RC_STATE_DMA_RD    = 3'd2;
	localparam RC_STATE_BITS      = 3;

	localparam M_STATE_IDLE        = 3'd0;
	localparam M_STATE_TLP_DW3_2   = 3'd1;
	localparam M_STATE_DMA_RD      = 3'd2;
	localparam M_STATE_DMA_RD_LAST = 3'd3;
	localparam M_STATE_BITS        = 3;

	genvar gen_i;
	integer rc_i;

	reg [STATE_BITS-1:0]   state_reg;
	reg [63:0]             buf_addr_int, buf_addr_next;
	reg                    buf_addr_next_carry;
	reg [SIZE_WIDTH-1:0]   size_int, size_next;
	reg [TIMES_WIDTH-1:0]  times_int;
	reg [BURST_WIDTH-1:0]  burst_bytes_int;
	wire [31:0]            size_4x;
	wire                   buf_addr_32bit;
	wire [31:0]            buf_addr_lo;
	wire [31:0]            buf_addr_hi;
	reg [C_DATA_WIDTH-1:0] rc_tdata;
	reg [KEEP_WIDTH-1:0]   rc_tkeep;
	reg                    rc_tlast;

	wire m00_axis_rc_fire;
	wire s_axis_rr_fire;
	wire s00_axis_fire;

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

	wire [6:0] tlp_hdr_fmt_type;
	wire [2:0] tlp_hdr_tc;
	wire       tlp_hdr_td;
	wire       tlp_hdr_ep;
	wire [1:0] tlp_hdr_attr;
	wire [9:0] tlp_hdr_len;
	wire [7:0] tlp_hdr_tag [RC_COUNT-1:0];
	wire [3:0] tlp_hdr_last_be;
	wire [3:0] tlp_hdr_first_be;

	// rc signals
	reg [RC_STATE_BITS-1:0] rc_state_reg [RC_COUNT-1:0];
	reg                     rc_req [RC_COUNT-1:0];
	reg [RC_COUNT-1:0]      rc_avail;
	reg [RC_CNT_WIDTH-1:0]  rc_cur_idx;
	reg [RC_CNT_WIDTH-1:0]  s_idx;
	reg [BURST_WIDTH-1:0]   rc_burst_bytes_int [RC_COUNT-1:0];
	reg [BURST_WIDTH-1:0]   req_len_int;

	// rc_fifo signals
	reg                           rc_fifo_wr_en [RC_COUNT-1:0];
	reg [RC_FIFO_DATA_WIDTH-1:0]  rc_fifo_wr_data [RC_COUNT-1:0];
	reg                           rc_fifo_rd_en [RC_COUNT-1:0];
	wire [RC_FIFO_DATA_WIDTH-1:0] rc_fifo_rd_data [RC_COUNT-1:0];
	wire                          rc_fifo_data_valid [RC_COUNT-1:0];
	wire                          rc_fifo_full [RC_COUNT-1:0];
	wire                          rc_fifo_empty [RC_COUNT-1:0];
	wire [C_DATA_WIDTH-1:0]       rc_fifo_rd_data_tdata [RC_COUNT-1:0];
	wire [KEEP_WIDTH-1:0]         rc_fifo_rd_data_tkeep [RC_COUNT-1:0];
	wire                          rc_fifo_rd_data_tlast [RC_COUNT-1:0];
	wire                          rc_fifo_rd_last [RC_COUNT-1:0];
	wire                          rc_fifo_rd_eos [RC_COUNT-1:0]; // end of stream

	// rc_idx_fifo signals
	reg                     rc_idx_fifo_wr_en;
	reg [RC_CNT_WIDTH-1:0]  rc_idx_fifo_wr_data;
	reg                     rc_idx_fifo_rd_en;
	wire [RC_CNT_WIDTH-1:0] rc_idx_fifo_rd_data;
	wire                    rc_idx_fifo_data_valid;
	wire                    rc_idx_fifo_full;
	wire                    rc_idx_fifo_empty;

	reg [RC_CNT_WIDTH-1:0] s00_selected_rc_idx;
	reg                    s00_selected_rc_valid;

	// m00 signals
	reg [M_STATE_BITS-1:0] m00_state_reg;

	assign ap_idle = (state_reg == STATE_IDLE);
	assign ap_done = (state_reg == STATE_FINISH);
	assign ap_ready = ap_done;

	assign size_4x = (size & ~32'b11);
	assign buf_addr_32bit = (buf_addr[63:32] == 32'b0);
	assign buf_addr_lo = {buf_addr_int[31:2], 2'b00};
	assign buf_addr_hi = buf_addr_int[63:32];

	assign m00_axis_rc_fire = m00_axis_rc_tready && m00_axis_rc_tvalid;
	assign s_axis_rr_tuser[0] = 1'b0; // Unused for V6
	assign s_axis_rr_tuser[1] = 1'b0; // Error forward packet
	assign s_axis_rr_tuser[2] = 1'b0; // Stream packet
	assign s_axis_rr_tuser[3] = 1'b0; // Unused discontinue
	assign s_axis_rr_fire = s_axis_rr_tready && s_axis_rr_tvalid;
	assign s00_axis_fire = s00_axis_tready && s00_axis_tvalid;

	assign rx_tlp_hdr_fmt_type = tlp_hdr_dw1_0[30:24];
	assign rx_tlp_hdr_tc = tlp_hdr_dw1_0[22:20];
	assign rx_tlp_hdr_td = tlp_hdr_dw1_0[15];
	assign rx_tlp_hdr_ep = tlp_hdr_dw1_0[14];
	assign rx_tlp_hdr_attr = tlp_hdr_dw1_0[13:12];
	assign rx_tlp_hdr_len = tlp_hdr_dw1_0[9:0];
	assign rx_tlp_hdr_rid = tlp_hdr_dw1_0[63:48];
	assign rx_tlp_hdr_tag = tlp_hdr_dw1_0[47:40];
	assign rx_tlp_hdr_be = tlp_hdr_dw1_0[39:32];

	assign tlp_hdr_fmt_type = (buf_addr_32bit ? TLP_MEM_RD32_FMT_TYPE : TLP_MEM_RD64_FMT_TYPE);
	assign tlp_hdr_tc = 3'b0;
	assign tlp_hdr_td = 1'b0;
	assign tlp_hdr_ep = 1'b0;
	assign tlp_hdr_attr = 2'b0;
	assign tlp_hdr_len = (burst_bytes_int >> 2); // measured by DW unit

	generate
		for(gen_i = 0;gen_i < RC_COUNT;gen_i = gen_i + 1) begin
			assign tlp_hdr_tag[gen_i] = RC_TAG_BASE + gen_i;
		end
	endgenerate

	assign tlp_hdr_last_be = (burst_bytes_int == 4 ? 4'b0000 : 4'b1111);
	assign tlp_hdr_first_be = 4'b1111;

	// state_reg
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			state_reg <= STATE_IDLE;
		end else begin
			case(state_reg)
				STATE_IDLE:
					if(ap_start) begin
						state_reg <= STATE_DMA_RD_NEXT;
					end

				STATE_DMA_RD_DW1_0:
					if(rc_avail[rc_cur_idx]) begin
						if(s_axis_rr_fire) begin
							state_reg <= buf_addr_32bit ? STATE_DMA_RD32_DW3_2 : STATE_DMA_RD64_DW3_2;
						end
					end

				STATE_DMA_RD32_DW3_2:
					if(s_axis_rr_fire) begin
						state_reg <= STATE_DMA_RD;
					end

				STATE_DMA_RD64_DW3_2:
					if(s_axis_rr_fire) begin
						state_reg <= STATE_DMA_RD;
					end

				STATE_DMA_RD:
					if(size_next == 0 && times_int == 1) begin
						state_reg <= STATE_WAIT_PENDING;
					end else begin
						state_reg <= STATE_DMA_RD_NEXT;
					end

				STATE_DMA_RD_NEXT: begin
					state_reg <= STATE_DMA_RD_DW1_0;
				end

				STATE_WAIT_PENDING:
					if(&rc_avail) begin
						state_reg <= STATE_FINISH;
					end

				STATE_FINISH: begin
					state_reg <= STATE_IDLE;
				end
			endcase
		end
	end

	// s_axis_rr_tdata, s_axis_rr_tkeep, s_axis_rr_tlast, s_axis_rr_tvalid
	always @(*) begin
		s_axis_rr_tdata = 'hEEFFAABBCAFECAFE; // for debug purpose
		s_axis_rr_tkeep = 0;
		s_axis_rr_tlast = 0;
		s_axis_rr_tvalid = 0;

		case(state_reg)
			STATE_DMA_RD_DW1_0:
				if(rc_avail[rc_cur_idx]) begin
					s_axis_rr_tlast = 0;
					s_axis_rr_tdata = {           // Bits
						// DW1
						cfg_completer_id,         // 16
						tlp_hdr_tag[rc_cur_idx],  // 8
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

			STATE_DMA_RD32_DW3_2: begin
				s_axis_rr_tlast = 1;
				s_axis_rr_tdata = {
					// DW3
					32'b0,
					// DW2 Addr
					buf_addr_lo
				};
				s_axis_rr_tkeep = 8'h0F;
				s_axis_rr_tvalid = 1;
			end

			STATE_DMA_RD64_DW3_2: begin
				s_axis_rr_tlast = 1;
				s_axis_rr_tdata = {
					// DW3 Addr LO
					buf_addr_lo,
					// DW2 Addr HI
					buf_addr_hi
				};
				s_axis_rr_tkeep = 8'hFF;
				s_axis_rr_tvalid = 1;
			end
		endcase
	end

	// buf_addr_int, buf_addr_next, buf_addr_next_carry, size_int, size_next, times_int, burst_bytes_int, rc_cur_idx
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			buf_addr_int <= 0;
			buf_addr_next <= 0;
			buf_addr_next_carry <= 0;
			size_int <= 0;
			size_next <= 0;
			times_int <= 0;
			burst_bytes_int <= 0;
			rc_cur_idx <= RC_COUNT - 1;
		end else begin
			case(state_reg)
				STATE_IDLE:
					if(ap_start) begin
						buf_addr_int <= buf_addr;
						size_int <= size_4x;
						times_int <= times;
					end

				STATE_DMA_RD_DW1_0: begin
					buf_addr_next[63:32] <= buf_addr_int[63:32] + buf_addr_next_carry;
				end

				STATE_DMA_RD: begin
					buf_addr_int <= buf_addr_next;
					size_int <= size_next;

					if(size_next == 0) begin
						times_int <= times_int - 1;

						buf_addr_int <= buf_addr;
						size_int <= size_4x;
					end
				end

				STATE_DMA_RD_NEXT: begin
					rc_cur_idx <= (rc_cur_idx == RC_COUNT - 1) ? 0 : rc_cur_idx + 1;

					if(size_int > BURST_SIZE) begin
						{buf_addr_next_carry, buf_addr_next[31:0]} <= buf_addr_int[31:0] + BURST_SIZE;
						size_next <= size_int - BURST_SIZE;
						burst_bytes_int <= BURST_SIZE;
					end else begin
						{buf_addr_next_carry, buf_addr_next[31:0]} <= buf_addr_int[31:0] + size_int;
						size_next <= 0;
						burst_bytes_int <= size_int;
					end
				end
			endcase
		end
	end

	rc_idx_fifo rc_idx_fifo_U (
		.clk(clk),
		.srst(~rst_n),
		.wr_en(rc_idx_fifo_wr_en),
		.din(rc_idx_fifo_wr_data),
		.rd_en(rc_idx_fifo_rd_en),
		.dout(rc_idx_fifo_rd_data),
		.full(rc_idx_fifo_full),
		.empty(rc_idx_fifo_empty),
		.valid(rc_idx_fifo_data_valid)
	);

	// rc_idx_fifo_wr_en, rc_idx_fifo_wr_data
	always @(*) begin
		rc_idx_fifo_wr_en = 0;
		rc_idx_fifo_wr_data = 0;

		case(state_reg)
			STATE_DMA_RD_DW1_0: begin
				rc_idx_fifo_wr_en = rc_avail[rc_cur_idx] && s_axis_rr_fire;
				rc_idx_fifo_wr_data = rc_cur_idx;
			end
		endcase
	end

	// rc_idx_fifo_rd_en
	always @(*) begin
		rc_idx_fifo_rd_en = 0;

		if(s00_selected_rc_valid && rc_fifo_rd_last[s00_selected_rc_idx] && rc_fifo_rd_eos[s00_selected_rc_idx]) begin
			rc_idx_fifo_rd_en = 1;
		end
	end

	// s00_selected_rc_idx, s00_selected_rc_valid
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			s00_selected_rc_idx <= 0;
			s00_selected_rc_valid <= 0;
		end else begin
			if(s00_selected_rc_valid && rc_fifo_rd_last[s00_selected_rc_idx] && rc_fifo_rd_eos[s00_selected_rc_idx]) begin
				s00_selected_rc_valid <= 0;
			end

			if(! s00_selected_rc_valid && rc_idx_fifo_data_valid) begin
				s00_selected_rc_idx <= rc_idx_fifo_rd_data;
				s00_selected_rc_valid <= 1;
			end
		end
	end

	generate
		for(gen_i = 0;gen_i < RC_COUNT;gen_i = gen_i + 1) begin
			assign rc_fifo_rd_data_tdata[gen_i] = rc_fifo_rd_data[gen_i][C_DATA_WIDTH+KEEP_WIDTH:KEEP_WIDTH+1];
			assign rc_fifo_rd_data_tkeep[gen_i] = rc_fifo_rd_data[gen_i][KEEP_WIDTH:1];
			assign rc_fifo_rd_data_tlast[gen_i] = rc_fifo_rd_data[gen_i][0:0];
			assign rc_fifo_rd_last[gen_i] = (rc_fifo_rd_en[gen_i] && rc_fifo_rd_data_tlast[gen_i]);
			assign rc_fifo_rd_eos[gen_i] = (rc_burst_bytes_int[gen_i] - req_len_int == 0);

			rc_fifo rc_fifo_U (
				.clk(clk),
				.srst(~rst_n),
				.wr_en(rc_fifo_wr_en[gen_i]),
				.din(rc_fifo_wr_data[gen_i]),
				.rd_en(rc_fifo_rd_en[gen_i]),
				.dout(rc_fifo_rd_data[gen_i]),
				.full(rc_fifo_full[gen_i]),
				.empty(rc_fifo_empty[gen_i]),
				.valid(rc_fifo_data_valid[gen_i])
			);
		end
	endgenerate

	generate
		for(gen_i = 0;gen_i < RC_COUNT;gen_i = gen_i + 1) begin
			// rc_state_reg[gen_i]
			always @(posedge clk or negedge rst_n) begin
				if(~rst_n) begin
					rc_state_reg[gen_i] <= RC_STATE_IDLE;
				end else begin
					case(rc_state_reg[gen_i])
						RC_STATE_IDLE:
							if(rc_req[gen_i]) begin
								rc_state_reg[gen_i] <= RC_STATE_MATCH_IDX;
							end

						RC_STATE_MATCH_IDX:
							if(rc_idx_fifo_data_valid && rc_idx_fifo_rd_data == gen_i) begin
								rc_state_reg[gen_i] <= RC_STATE_DMA_RD;
							end

						RC_STATE_DMA_RD: begin
							if(rc_fifo_rd_last[gen_i] && rc_fifo_rd_eos[gen_i]) begin
								rc_state_reg[gen_i] <= RC_STATE_IDLE;
							end
						end
					endcase
				end
			end

			// rc_avail[gen_i]
			always @(posedge clk or negedge rst_n) begin
				if(~rst_n) begin
					rc_avail[gen_i] <= 1;
				end else begin
					case(rc_state_reg[gen_i])
						RC_STATE_IDLE:
							if(rc_req[gen_i]) begin
								rc_avail[gen_i] <= 0;
							end

						RC_STATE_DMA_RD: begin
							if(rc_fifo_rd_last[gen_i] && rc_fifo_rd_eos[gen_i]) begin
								rc_avail[gen_i] <= 1;
							end
						end
					endcase
				end
			end

			// rc_req[gen_i]
			always @(posedge clk or negedge rst_n) begin
				if(~rst_n) begin
					rc_req[gen_i] <= 0;
				end else begin
					case(state_reg)
						STATE_DMA_RD_DW1_0:
							if(s_axis_rr_fire && rc_cur_idx == gen_i) begin
								rc_req[gen_i] <= 1;
							end

						STATE_DMA_RD:
							if(rc_cur_idx == gen_i) begin
								rc_req[gen_i] <= 0;
							end
					endcase
				end
			end

			// rc_burst_bytes_int[gen_i]
			always @(posedge clk or negedge rst_n) begin
				if(~rst_n) begin
					rc_burst_bytes_int[gen_i] <= 0;
				end else begin
					case(state_reg)
						STATE_DMA_RD_DW1_0:
							if(s_axis_rr_fire && rc_cur_idx == gen_i) begin
								rc_burst_bytes_int[gen_i] <= burst_bytes_int;
							end
					endcase

					case(rc_state_reg[gen_i])
						RC_STATE_DMA_RD: begin
							if(rc_fifo_rd_last[gen_i]) begin
								rc_burst_bytes_int[gen_i] <= rc_burst_bytes_int[gen_i] - req_len_int;
							end
						end
					endcase
				end
			end
		end
	endgenerate

	// m00_state_reg
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			m00_state_reg <= M_STATE_IDLE;
		end else begin
			case(m00_state_reg)
				M_STATE_IDLE: begin
					if(ap_start) begin
						m00_state_reg <= M_STATE_TLP_DW3_2;
					end
				end

				M_STATE_TLP_DW3_2: begin
					if(m00_axis_rc_fire) begin
						m00_state_reg <= M_STATE_DMA_RD;

						if(m00_axis_rc_tlast) begin
							m00_state_reg <= M_STATE_DMA_RD_LAST;
						end
					end
				end

				M_STATE_DMA_RD:
					if(! rc_fifo_full[s_idx]) begin
						if(m00_axis_rc_fire) begin
							m00_state_reg <= M_STATE_DMA_RD;

							if(m00_axis_rc_tlast) begin
								m00_state_reg <= M_STATE_DMA_RD_LAST;
							end
						end
					end

				M_STATE_DMA_RD_LAST:
					if(! rc_fifo_full[s_idx]) begin
						m00_state_reg <= M_STATE_TLP_DW3_2;

						// back-to-back
						if(m00_axis_rc_fire) begin
							m00_state_reg <= M_STATE_DMA_RD;

							if(m00_axis_rc_tlast) begin
								m00_state_reg <= M_STATE_DMA_RD_LAST;
							end
						end
					end
			endcase
		end
	end

	// m00_axis_rc_tready
	always @(*) begin
		m00_axis_rc_tready = 0;

		case(m00_state_reg)
			M_STATE_TLP_DW3_2: begin
				m00_axis_rc_tready = 1;
			end

			M_STATE_DMA_RD: begin
				if(! rc_fifo_full[s_idx]) begin
					m00_axis_rc_tready = 1;
				end
			end

			M_STATE_DMA_RD_LAST: begin
				// back-to-back
				if(! rc_fifo_full[s_idx]) begin
					m00_axis_rc_tready = 1;
				end
			end
		endcase
	end

	// req_len_int
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			req_len_int <= 0;
		end else begin
			case(m00_state_reg)
				M_STATE_TLP_DW3_2: begin
					req_len_int <= (rx_tlp_hdr_len << 2);
				end
			endcase
		end
	end

	// s_idx, rc_tdata, rc_tkeep, rc_tlast
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			s_idx <= 0;
			rc_tdata <= 0;
			rc_tkeep <= 0;
			rc_tlast <= 0;
		end else begin
			case(m00_state_reg)
				M_STATE_TLP_DW3_2:
					if(m00_axis_rc_fire) begin
						s_idx <= m00_axis_rc_tdata[8+RC_CNT_WIDTH-1:8];
						rc_tdata <= m00_axis_rc_tdata;
						rc_tkeep <= m00_axis_rc_tkeep;
						rc_tlast <= m00_axis_rc_tlast;
					end

				M_STATE_DMA_RD:
					if(m00_axis_rc_fire) begin
						rc_tdata <= m00_axis_rc_tdata;
						rc_tkeep <= m00_axis_rc_tkeep;
						rc_tlast <= m00_axis_rc_tlast;
					end

				M_STATE_DMA_RD_LAST:
					// back-to-back
					if(m00_axis_rc_fire) begin
						s_idx <= m00_axis_rc_tdata[8+RC_CNT_WIDTH-1:8];
						rc_tdata <= m00_axis_rc_tdata;
						rc_tkeep <= m00_axis_rc_tkeep;
						rc_tlast <= m00_axis_rc_tlast;
					end
			endcase
		end
	end

	assign s00_axis_tuser = 0; // TODO: SOF?
	assign s00_axis_tlast = 0; // TODO: EOL?

	// s00_axis_tdata, s00_axis_tkeep, s00_axis_tvalid
	always @(*) begin
		s00_axis_tdata = 0;
		s00_axis_tkeep = 0;
		s00_axis_tvalid = 0;

		if(s00_selected_rc_valid) begin
			s00_axis_tdata = rc_fifo_rd_data_tdata[s00_selected_rc_idx];
			s00_axis_tkeep = rc_fifo_rd_data_tkeep[s00_selected_rc_idx];
			s00_axis_tvalid = rc_fifo_data_valid[s00_selected_rc_idx];
		end
	end

	// rc_fifo_wr_en[rc_i], rc_fifo_wr_data[rc_i]
	always @(*) begin
		for(rc_i = 0;rc_i < RC_COUNT;rc_i = rc_i + 1) begin
			rc_fifo_wr_en[rc_i] = 0;
			rc_fifo_wr_data[rc_i] = 0;
		end

		case(m00_state_reg)
			M_STATE_DMA_RD:
				if(! rc_fifo_full[s_idx]) begin
					rc_fifo_wr_data[s_idx] = {
						rc_tdata, rc_tkeep, rc_tlast
					};
					rc_fifo_wr_en[s_idx] = m00_axis_rc_fire;
				end

			M_STATE_DMA_RD_LAST:
				if(! rc_fifo_full[s_idx]) begin
					rc_fifo_wr_data[s_idx] = {
						rc_tdata, rc_tkeep, rc_tlast
					};
					rc_fifo_wr_en[s_idx] = 1;
				end
		endcase
	end

	// rc_fifo_rd_en[gen_i]
	generate
		for(gen_i = 0;gen_i < RC_COUNT;gen_i = gen_i + 1) begin
			always @(*) begin
				rc_fifo_rd_en[gen_i] = 0;

				case(rc_state_reg[gen_i])
					RC_STATE_DMA_RD: begin
						rc_fifo_rd_en[gen_i] = s00_axis_fire;
					end
				endcase
			end
		end
	endgenerate
endmodule
