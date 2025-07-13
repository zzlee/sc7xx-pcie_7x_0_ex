`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/21/2025 04:44:53 PM
// Design Name: 
// Module Name: tlp_reg
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

module tlp_reg #(
	parameter C_DATA_WIDTH = 64,
	parameter C_CQ_USER_WIDTH = 22,
	parameter C_CC_USER_WIDTH = 4,

	parameter C_CTRL_ADDR_WIDTH = 32,
	parameter C_CTRL_DATA_WIDTH = 32,

	parameter C_BAR0_ADDR_MASK = 32'hFFF00000,

	// Do not override parameters below this line
	parameter KEEP_WIDTH = C_DATA_WIDTH / 8,
	parameter CTRL_KEEP_WIDTH = C_CTRL_DATA_WIDTH / 8,
	parameter CTRL_WSTRB_WIDTH = CTRL_KEEP_WIDTH
) (
	input clk,
	input rst_n,

	// CQ TLP
	input [C_DATA_WIDTH-1:0]    s_axis_cq_tdata,
	input [KEEP_WIDTH-1:0]      s_axis_cq_tkeep,
	input                       s_axis_cq_tlast,
	input                       s_axis_cq_tvalid,
	output reg                  s_axis_cq_tready,
	input [C_CQ_USER_WIDTH-1:0] s_axis_cq_tuser,

	// CC TLP
	output reg [C_DATA_WIDTH-1:0] m_axis_cc_tdata,
	output reg [KEEP_WIDTH-1:0]   m_axis_cc_tkeep,
	output reg                    m_axis_cc_tlast,
	output reg                    m_axis_cc_tvalid,
	input                         m_axis_cc_tready,
	output [C_CC_USER_WIDTH-1:0]  m_axis_cc_tuser,

	input [15:0]             cfg_completer_id,
	input [C_DATA_WIDTH-1:0] tlp_hdr_dw1_0,

	output [C_CTRL_ADDR_WIDTH-1:0] m_axi_ctrl_awaddr,
	output reg                     m_axi_ctrl_awvalid,
	input                          m_axi_ctrl_awready,
	output [C_CTRL_DATA_WIDTH-1:0] m_axi_ctrl_wdata,
	output [CTRL_WSTRB_WIDTH-1:0]  m_axi_ctrl_wstrb,
	output reg                     m_axi_ctrl_wvalid,
	input                          m_axi_ctrl_wready,
	input [1:0]                    m_axi_ctrl_bresp,
	input                          m_axi_ctrl_bvalid,
	output reg                     m_axi_ctrl_bready,
	output [C_CTRL_ADDR_WIDTH-1:0] m_axi_ctrl_araddr,
	output reg                     m_axi_ctrl_arvalid,
	input                          m_axi_ctrl_arready,
	input [C_CTRL_DATA_WIDTH-1:0]  m_axi_ctrl_rdata,
	input [1:0]                    m_axi_ctrl_rresp,
	input                          m_axi_ctrl_rvalid,
	output reg                     m_axi_ctrl_rready
);

	localparam STATE_IDLE     = 4'd0;
	localparam STATE_RX_DW3_2 = 4'd2;
	localparam STATE_TX_DW1_0 = 4'd3;
	localparam STATE_TX_DW3_2 = 4'd4;
	localparam STATE_CTRL_AW  = 4'd5;
	localparam STATE_CTRL_W   = 4'd6;
	localparam STATE_CTRL_B   = 4'd7;
	localparam STATE_CTRL_AR  = 4'd8;
	localparam STATE_CTRL_R   = 4'd9;
	localparam STATE_BITS     = 4;

	localparam TLP_MEM_RD32_FMT_TYPE = 7'b00_00000;
	localparam TLP_MEM_WR32_FMT_TYPE = 7'b10_00000;
	localparam TLP_CPLD_FMT_TYPE     = 7'b10_01010;

	reg [STATE_BITS-1:0]   state_reg;
	reg [C_DATA_WIDTH-1:0] ctrl_rx_data;

	wire s_axis_cq_fire;
	wire m_axis_cc_fire;
	wire m_axi_ctrl_aw_fire;
	wire m_axi_ctrl_w_fire;
	wire m_axi_ctrl_b_fire;
	wire m_axi_ctrl_ar_fire;
	wire m_axi_ctrl_r_fire;

	// TLP DW1_DW0
	wire [6:0]  tlp_hdr_fmt_type;
	wire [2:0]  tlp_hdr_tc;
	wire        tlp_hdr_td;
	wire        tlp_hdr_ep;
	wire [1:0]  tlp_hdr_attr;
	wire [9:0]  tlp_hdr_len;
	wire [15:0] tlp_hdr_rid;
	wire [7:0]  tlp_hdr_tag;
	wire [7:0]  tlp_hdr_be;

	// TLP DW3_DW2
	reg [C_DATA_WIDTH-1:0] tlp_hdr_dw3_2;
	wire [31:0]            tlp_hdr_addr32;

	wire [11:0] byte_count;
	wire [6:0]  lower_addr;
	wire [31:0] bar0_addr;
	wire [31:0] pci_rd32_data;
	wire [31:0] axi_wr32_data;

	assign m_axis_cc_tuser[0] = 1'b0; // Unused for V6
	assign m_axis_cc_tuser[1] = 1'b0; // Error forward packet
	assign m_axis_cc_tuser[2] = 1'b0; // Stream packet
	assign m_axis_cc_tuser[3] = 1'b0; // Unused discontinue

	assign s_axis_cq_fire = s_axis_cq_tready && s_axis_cq_tvalid;
	assign m_axis_cc_fire = m_axis_cc_tready && m_axis_cc_tvalid;
	assign m_axi_ctrl_aw_fire = m_axi_ctrl_awready && m_axi_ctrl_awvalid;
	assign m_axi_ctrl_w_fire = m_axi_ctrl_wready && m_axi_ctrl_wvalid;
	assign m_axi_ctrl_b_fire = m_axi_ctrl_bready && m_axi_ctrl_bvalid;
	assign m_axi_ctrl_ar_fire = m_axi_ctrl_arready && m_axi_ctrl_arvalid;
	assign m_axi_ctrl_r_fire = m_axi_ctrl_rready && m_axi_ctrl_rvalid;

	assign tlp_hdr_fmt_type = tlp_hdr_dw1_0[30:24];
	assign tlp_hdr_tc = tlp_hdr_dw1_0[22:20];
	assign tlp_hdr_td = tlp_hdr_dw1_0[15];
	assign tlp_hdr_ep = tlp_hdr_dw1_0[14];
	assign tlp_hdr_attr = tlp_hdr_dw1_0[13:12];
	assign tlp_hdr_len = tlp_hdr_dw1_0[9:0];
	assign tlp_hdr_rid = tlp_hdr_dw1_0[63:48];
	assign tlp_hdr_tag = tlp_hdr_dw1_0[47:40];
	assign tlp_hdr_be = tlp_hdr_dw1_0[39:32];
	assign tlp_hdr_addr32 = {tlp_hdr_dw3_2[31:2], 2'b00};

	assign byte_count = 12'd4;
	assign lower_addr = {tlp_hdr_addr32[6:0]};
	assign bar0_addr = (tlp_hdr_addr32 & ~C_BAR0_ADDR_MASK);
	assign pci_rd32_data = {ctrl_rx_data[7:0], ctrl_rx_data[15:8], ctrl_rx_data[23:16], ctrl_rx_data[31:24]};
	assign axi_wr32_data = {tlp_hdr_dw3_2[39:32], tlp_hdr_dw3_2[47:40], tlp_hdr_dw3_2[55:48], tlp_hdr_dw3_2[63:56]};

	assign m_axi_ctrl_awaddr = bar0_addr;
	assign m_axi_ctrl_wdata = axi_wr32_data;
	assign m_axi_ctrl_wstrb = {(CTRL_WSTRB_WIDTH){1'b1}};
	assign m_axi_ctrl_araddr = bar0_addr;

	// @FF state_reg
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			state_reg <= STATE_IDLE;
		end else begin
			case(state_reg)
				STATE_IDLE: state_reg <= STATE_RX_DW3_2;

				STATE_RX_DW3_2: begin
					if(s_axis_cq_fire) begin
						case(tlp_hdr_fmt_type)
							TLP_MEM_RD32_FMT_TYPE:
								if(tlp_hdr_len == 10'b1)
									state_reg <= STATE_CTRL_AR;
								else
									state_reg <= STATE_IDLE;

							TLP_MEM_WR32_FMT_TYPE:
								if(tlp_hdr_len == 10'b1)
									state_reg <= STATE_CTRL_AW;
								else
									state_reg <= STATE_IDLE;

							default: state_reg <= STATE_IDLE;
						endcase
					end
				end

				STATE_CTRL_AW: begin
					if(m_axi_ctrl_aw_fire) begin
						state_reg <= STATE_CTRL_W;
					end
				end

				STATE_CTRL_W: begin
					if(m_axi_ctrl_w_fire) begin
						state_reg <= STATE_CTRL_B;
					end
				end

				STATE_CTRL_B: begin
					if(m_axi_ctrl_b_fire) begin
						state_reg <= STATE_RX_DW3_2;
					end
				end

				STATE_CTRL_AR: begin
					if(m_axi_ctrl_ar_fire) begin
						state_reg <= STATE_CTRL_R;
					end
				end

				STATE_CTRL_R: begin
					if(m_axi_ctrl_r_fire) begin
						state_reg <= STATE_TX_DW1_0;
					end
				end

				STATE_TX_DW1_0: begin
					if(m_axis_cc_fire) begin
						state_reg <= STATE_TX_DW3_2;
					end
				end

				STATE_TX_DW3_2: begin
					if(m_axis_cc_fire) begin
						state_reg <= STATE_RX_DW3_2;
					end
				end
			endcase
		end
	end

	// @COMB s_axis_cq_tready, @COMB m_axis_cc_tdata, @COMB m_axis_cc_tkeep, @COMB m_axis_cc_tlast, @COMB m_axis_cc_tvalid
	// @COMB m_axi_ctrl_awvalid, @COMB m_axi_ctrl_wvalid, @COMB m_axi_ctrl_bready, @COMB m_axi_ctrl_arvalid, @COMB m_axi_ctrl_rready
	always @(*) begin
		s_axis_cq_tready = 0;
		m_axis_cc_tdata = 'hAABBCCDDCAFECAFE; // for debug purpose
		m_axis_cc_tkeep = 0;
		m_axis_cc_tlast = 0;
		m_axis_cc_tvalid = 0;

		m_axi_ctrl_awvalid = 0;
		m_axi_ctrl_wvalid = 0;
		m_axi_ctrl_bready = 0;
		m_axi_ctrl_arvalid = 0;
		m_axi_ctrl_rready = 0;

		case(state_reg)
			STATE_RX_DW3_2: begin
				s_axis_cq_tready = 1;
			end

			STATE_CTRL_AW: begin
				m_axi_ctrl_awvalid = 1;
			end

			STATE_CTRL_W: begin
				m_axi_ctrl_wvalid = 1;
			end

			STATE_CTRL_B: begin
				m_axi_ctrl_bready = 1;
			end

			STATE_CTRL_AR: begin
				m_axi_ctrl_arvalid = 1;
			end

			STATE_CTRL_R: begin
				m_axi_ctrl_rready = 1;
			end

			STATE_TX_DW1_0: begin
				m_axis_cc_tlast = 0;
				m_axis_cc_tdata = {           // Bits
					// DW1
					cfg_completer_id,         // 16
					{3'b0},                   // 3
					{1'b0},                   // 1
					{byte_count},             // 12
                    // DW0
					{1'b0},                   // 1
					(TLP_CPLD_FMT_TYPE),      // 7
					{1'b0},                   // 1
					tlp_hdr_tc,               // 3
					{4'b0},                   // 4
					tlp_hdr_td,               // 1
					tlp_hdr_ep,               // 1
					tlp_hdr_attr,             // 2
					{2'b0},                   // 2
					tlp_hdr_len               // 10
				};
				m_axis_cc_tkeep = 8'hFF;
				m_axis_cc_tvalid = 1;
			end

			STATE_TX_DW3_2: begin
				m_axis_cc_tlast = 1;
                m_axis_cc_tdata = {      // Bits
                	// DW3
					{pci_rd32_data},     // 32
                	// DW2
					tlp_hdr_rid,         // 16
					tlp_hdr_tag,         //  8
					{1'b0},              //  1
					{lower_addr}         //  7
				};
				m_axis_cc_tkeep = 8'hFF;
				m_axis_cc_tvalid = 1;
			end
		endcase
	end

	// tlp_hdr_dw3_2
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			tlp_hdr_dw3_2 <= 0;
		end else begin
			case(state_reg)
				STATE_RX_DW3_2: begin
					if(s_axis_cq_fire) begin
						tlp_hdr_dw3_2 <= s_axis_cq_tdata;
					end
				end
			endcase
		end
	end

	// ctrl_rx_data
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ctrl_rx_data <= 0;
		end else begin
			case(state_reg)
				STATE_CTRL_R: begin
					if(m_axi_ctrl_r_fire) begin
						ctrl_rx_data <= m_axi_ctrl_rdata;
					end
				end
			endcase
		end
	end
endmodule
