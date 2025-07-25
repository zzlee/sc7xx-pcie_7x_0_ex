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
	parameter C_RC_USER_WIDTH   = 22,
	parameter C_RR_USER_WIDTH   = 4,
	parameter C_SINK_DATA_WIDTH = 64,

	// Do not override parameters below this line
	parameter KEEP_WIDTH      = C_DATA_WIDTH / 8,
	parameter SINK_KEEP_WIDTH = C_SINK_DATA_WIDTH / 8
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

	// RC TLP
	input [C_DATA_WIDTH-1:0]    s_axis_rc_tdata,
	input [KEEP_WIDTH-1:0]      s_axis_rc_tkeep,
	input                       s_axis_rc_tlast,
	input                       s_axis_rc_tvalid,
	output                      s_axis_rc_tready,
	input [C_RC_USER_WIDTH-1:0] s_axis_rc_tuser,

	// RR TLP
	output reg [C_DATA_WIDTH-1:0] m_axis_rr_tdata,
	output reg [KEEP_WIDTH-1:0]   m_axis_rr_tkeep,
	output reg                    m_axis_rr_tlast,
	output reg                    m_axis_rr_tvalid,
	input                         m_axis_rr_tready,
	output [C_RR_USER_WIDTH-1:0]  m_axis_rr_tuser,

	// M00 SINK
	output reg [C_SINK_DATA_WIDTH-1:0] m00_axis_tdata,
	output reg [SINK_KEEP_WIDTH-1:0]   m00_axis_tkeep,
	output                             m00_axis_tlast,
	output reg                         m00_axis_tvalid,
	input                              m00_axis_tready,
	output                             m00_axis_tuser,

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

	localparam TLP_MEM_RD32_FMT_TYPE = 7'b00_00000;
	localparam TLP_MEM_RD64_FMT_TYPE = 7'b01_00000;

	localparam RC_TAG_BASE  = 8'h10; // tag base for tlp_demuxer/m01_axis_rc
	localparam RC_COUNT     = 15;
	localparam RC_CNT_WIDTH = 4;

	localparam BURST_SIZE  = 256; // bytes per burst
	localparam BURST_WIDTH = $clog2(BURST_SIZE + 1);

	localparam MAX_SIZE   = 4096 * 2160 * 4;
	localparam SIZE_WIDTH = $clog2(MAX_SIZE + 1);

	localparam DESC_WIDTH = 32+32+32; // {bytes, addr_hi, addr_lo}

	localparam STATE_IDLE         = 4'd0;
	localparam STATE_FINISH       = 4'd1;
	localparam STATE_DMA_RD_DW1_0 = 4'd2;
	localparam STATE_DMA_RD_DW3_2 = 4'd3;
	localparam STATE_WAIT_DESC    = 4'd4;
	localparam STATE_DMA_RD       = 4'd5;
	localparam STATE_DMA_RD_NEXT  = 4'd6;
	localparam STATE_WAIT_PENDING = 4'd7;
	localparam STATE_ERROR        = 4'b1111;
	localparam STATE_BITS         = 4;

	genvar gen_i;
	integer i;

	reg [STATE_BITS-1:0]   ap_state;
	reg [31:0]             ERR_MASK_next;
	reg [63:0]             buf_addr_int;
	wire [63:0]            buf_addr_next;
	reg [SIZE_WIDTH-1:0]   addr_adder_inc;
	reg [31:0]             buf_size_int, buf_size_next;
	reg                    buf_addr_32bit;
	wire [31:0]            buf_addr_lo;
	wire [31:0]            buf_addr_hi;
	reg [SIZE_WIDTH-1:0]   size_int, size_next;
	reg [BURST_WIDTH-1:0]  burst_bytes_int;

	wire m_axis_rr_fire;

	// RX TLP DW1_0
	wire [6:0]  rx_tlp_hdr_fmt_type;
	wire [2:0]  rx_tlp_hdr_tc;
	wire        rx_tlp_hdr_td;
	wire        rx_tlp_hdr_ep;
	wire [1:0]  rx_tlp_hdr_attr;
	wire [9:0]  rx_tlp_hdr_len;
	wire [15:0] rx_tlp_hdr_rid;
	wire [7:0]  rx_tlp_hdr_tag;
	wire [7:0]  rx_tlp_hdr_be;

	// TX TLP DW1_0
	wire [6:0] tlp_hdr_fmt_type;
	wire [2:0] tlp_hdr_tc;
	wire       tlp_hdr_td;
	wire       tlp_hdr_ep;
	wire [1:0] tlp_hdr_attr;
	wire [9:0] tlp_hdr_len;
	wire [7:0] tlp_hdr_tag [RC_COUNT-1:0];
	wire [3:0] tlp_hdr_last_be;
	wire [3:0] tlp_hdr_first_be;

	// dma_rd_burst_U signals
	reg [RC_CNT_WIDTH-1:0]         dma_rd_burst_idx;
	reg [RC_COUNT*BURST_WIDTH-1:0] dma_rd_burst_burst_bytes;
	reg [RC_COUNT-1:0]             dma_rd_burst_req;
	wire [RC_COUNT-1:0]            dma_rd_burst_avail;

	// desc_rd_U signals
	reg                   desc_rd_rd_en;
	wire [DESC_WIDTH-1:0] desc_rd_rd_data;
	wire                  desc_rd_data_valid;
	reg                   desc_rd_ap_start;
	wire                  desc_rd_ap_ready;
	wire [31:0]           desc_rd_ERR_MASK;

	assign ap_idle = (ap_state == STATE_IDLE);
	assign ap_done = (ap_state == STATE_FINISH);
	assign ap_ready = ap_done;

	tlp_xdma_desc_rd #(
		.C_DATA_WIDTH(C_DATA_WIDTH),
		.C_RC_USER_WIDTH(C_RC_USER_WIDTH),
		.C_RR_USER_WIDTH(C_RR_USER_WIDTH),
		.C_RC_TAG(8'hF1), // tag base for tlp_demuxer/m01_axis_desc_rc
		.C_DMA_DIR(1) // DMA_TO_DEVICE
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
		for(gen_i = 0;gen_i < RC_COUNT;gen_i = gen_i + 1) begin
			assign tlp_hdr_tag[gen_i] = RC_TAG_BASE + gen_i;
		end
	endgenerate

	assign tlp_hdr_last_be = (burst_bytes_int == 4 ? 4'b0000 : 4'b1111);
	assign tlp_hdr_first_be = 4'b1111;

	// @FF ap_state
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= STATE_IDLE;
		end else begin
			case(ap_state)
				STATE_IDLE:
					if(ap_start) begin
						ap_state <= STATE_WAIT_DESC;
					end

				STATE_WAIT_DESC: begin
					if(desc_rd_data_valid) begin
						ap_state <= STATE_DMA_RD_NEXT;
					end
				end

				STATE_DMA_RD_DW1_0: begin
					if(dma_rd_burst_avail[dma_rd_burst_idx]) begin
						if(m_axis_rr_fire) begin
							ap_state <= STATE_DMA_RD_DW3_2;
						end
					end
				end

				STATE_DMA_RD_DW3_2:
					if(m_axis_rr_fire) begin
						ap_state <= STATE_DMA_RD;
					end

				STATE_DMA_RD: begin
					ap_state <= STATE_DMA_RD_NEXT;

					if(size_next == 0) begin
						ap_state <= STATE_WAIT_PENDING;
					end else if(buf_size_next == 0) begin
						ap_state <= STATE_WAIT_DESC;
					end
				end

				STATE_DMA_RD_NEXT: begin
					ap_state <= STATE_DMA_RD_DW1_0;
				end

				STATE_WAIT_PENDING: begin
					if(&dma_rd_burst_avail) begin
						ap_state <= STATE_FINISH;
					end
				end

				STATE_FINISH: begin
					ap_state <= STATE_IDLE;
				end
			endcase

			if(|ERR_MASK) begin
				ap_state <= STATE_ERROR;
			end
		end
	end

	// @FF desc_rd_ap_start
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			desc_rd_ap_start <= 0;
		end else begin
			case(ap_state)
				STATE_IDLE: begin
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
			STATE_WAIT_DESC: begin
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
			STATE_DMA_RD_DW1_0:
				if(dma_rd_burst_avail[dma_rd_burst_idx]) begin
					m_axis_rr_tlast = 0;
					m_axis_rr_tdata = {           // Bits
						// DW1
						cfg_completer_id,         // 16
						tlp_hdr_tag[dma_rd_burst_idx],  // 8
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

			STATE_DMA_RD_DW3_2: begin
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

	assign m_axis_rr_tuser[0] = 1'b0; // Unused for V6
	assign m_axis_rr_tuser[1] = 1'b0; // Error forward packet
	assign m_axis_rr_tuser[2] = 1'b0; // Stream packet
	assign m_axis_rr_tuser[3] = 1'b0; // Unused discontinue
	assign m_axis_rr_fire = m_axis_rr_tready && m_axis_rr_tvalid;

	// @FF buf_addr_int, @FF buf_addr_32bit, @FF addr_adder_inc, @FF size_int, @FF size_next,
	// @FF buf_size_int, @FF buf_size_next, @FF burst_bytes_int
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			buf_addr_int <= 'hCAFE0001; // for debug purpose
			buf_addr_32bit <= 0;
			addr_adder_inc <= 0;
			size_int <= 0;
			size_next <= 0;
			buf_size_int <= 'hCAFE0002; // for debug purpose
			buf_size_next <= 0;
			burst_bytes_int <= 0;
		end else begin
			case(ap_state)
				STATE_IDLE: begin
					if(ap_start) begin
						size_int <= (SIZE & ~32'b11);
					end
				end

				STATE_WAIT_DESC: begin
					if(desc_rd_data_valid) begin
						buf_addr_int <= desc_rd_rd_data[63:0];
						buf_addr_32bit <= (desc_rd_rd_data[63:32] == 32'b0);
						buf_size_int <= desc_rd_rd_data[64 +: 32];
					end
				end

				STATE_DMA_RD: begin
					buf_addr_int <= buf_addr_next;
					size_int <= size_next;
					buf_size_int <= buf_size_next;
				end

				STATE_DMA_RD_NEXT: begin
					if(size_int > BURST_SIZE) begin
						addr_adder_inc <= BURST_SIZE;
						size_next <= size_int - BURST_SIZE;
						burst_bytes_int <= BURST_SIZE;
					end else begin
						addr_adder_inc <= size_int;
						size_next <= 0;
						burst_bytes_int <= size_int;
					end
				end
			endcase
		end
	end

	tlp_dma_rd_burst #(
		.C_DATA_WIDTH(C_DATA_WIDTH),
		.C_RC_USER_WIDTH(C_RC_USER_WIDTH),
		.C_RC_COUNT(RC_COUNT),
		.C_RC_CNT_WIDTH(RC_CNT_WIDTH),
		.C_BURST_SIZE(BURST_SIZE)
	) dma_rd_burst_U(
		.clk(clk),
		.rst_n(rst_n),

		.s_axis_rc_tdata(s_axis_rc_tdata),
		.s_axis_rc_tkeep(s_axis_rc_tkeep),
		.s_axis_rc_tlast(s_axis_rc_tlast),
		.s_axis_rc_tvalid(s_axis_rc_tvalid),
		.s_axis_rc_tready(s_axis_rc_tready),
		.s_axis_rc_tuser(s_axis_rc_tuser),

		.burst_bytes(dma_rd_burst_burst_bytes),
		.req(dma_rd_burst_req),
		.avail(dma_rd_burst_avail)
	);

	// @FF dma_rd_burst_idx
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			dma_rd_burst_idx <= RC_COUNT - 1;
		end else begin
			case(ap_state)
				STATE_DMA_RD_NEXT: begin
					dma_rd_burst_idx <= (dma_rd_burst_idx == RC_COUNT - 1) ? 0 : dma_rd_burst_idx + 1;
				end
			endcase
		end
	end

	// @FF dma_rd_burst_req[*], @FF dma_rd_burst_burst_bytes[*]
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			for(i = 0;i < RC_COUNT;i = i + 1) begin
				dma_rd_burst_req[i] <= 0;
				dma_rd_burst_burst_bytes[i*BURST_WIDTH +: BURST_WIDTH] <= 0;
			end
		end else begin
			case(ap_state)
				STATE_DMA_RD_DW1_0:
					if(dma_rd_burst_avail[dma_rd_burst_idx]) begin
						if(m_axis_rr_fire) begin
							dma_rd_burst_req[dma_rd_burst_idx] <= 1;
							dma_rd_burst_burst_bytes[dma_rd_burst_idx*BURST_WIDTH +: BURST_WIDTH] <= burst_bytes_int;
						end
					end

				STATE_DMA_RD_DW3_2: begin
					dma_rd_burst_req[dma_rd_burst_idx] <= 0;
				end
			endcase
		end
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

		case(ap_state)
			STATE_DMA_RD_NEXT: begin
				if(buf_size_int > size_int) begin
					ERR_MASK_next = ERR_MASK_next | ERR_MASK_BUF_SIZE;
				end
			end
		endcase
	end
endmodule
