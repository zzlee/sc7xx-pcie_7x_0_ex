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

	// CC TLP
	output reg [C_DATA_WIDTH-1:0] m_axis_cc_tdata,
	output reg [KEEP_WIDTH-1:0]   m_axis_cc_tkeep,
	output reg                    m_axis_cc_tlast,
	output reg                    m_axis_cc_tvalid,
	input                         m_axis_cc_tready,

	input [15:0] cfg_completer_id,

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
	localparam TLP_MEM_RD32_FMT_TYPE = 7'b00_00000;
	localparam TLP_MEM_WR32_FMT_TYPE = 7'b10_00000;
	localparam TLP_CPLD_FMT_TYPE     = 7'b10_01010;

	localparam AP_STATE_IDLE  = 2'd0;
	localparam AP_STATE_RX_DW = 2'd1;
	localparam AP_STATE_TX_DW = 2'd2;
	localparam AP_STATE_CTRL  = 2'd3;
	localparam AP_STATE_WIDTH = 2;

	localparam CTRL_STATE_IDLE  = 3'd0;
	localparam CTRL_STATE_AW    = 3'd1;
	localparam CTRL_STATE_W     = 3'd2;
	localparam CTRL_STATE_B     = 3'd3;
	localparam CTRL_STATE_AR    = 3'd4;
	localparam CTRL_STATE_R     = 3'd5;
	localparam CTRL_STATE_WIDTH = 3;

	reg [AP_STATE_WIDTH-1:0] ap_state;
	reg [C_DATA_WIDTH-1:0]   ctrl_rx_data;
	reg [0:0]                rx_dw_idx;
	reg [C_DATA_WIDTH-1:0]   rx_dw [1:0];
	reg [0:0]                tx_dw_idx;
	reg [3:0]                ctrl_state;

	wire s_axis_cq_fire;
	wire m_axis_cc_fire;
	wire m_axi_ctrl_aw_fire;
	wire m_axi_ctrl_w_fire;
	wire m_axi_ctrl_b_fire;
	wire m_axi_ctrl_ar_fire;
	wire m_axi_ctrl_r_fire;

	// TLP DW1_DW0
	wire [C_DATA_WIDTH-1:0] tlp_hdr_dw1_0;
	wire [6:0]              tlp_hdr_fmt_type;
	wire [2:0]              tlp_hdr_tc;
	wire                    tlp_hdr_td;
	wire                    tlp_hdr_ep;
	wire [1:0]              tlp_hdr_attr;
	wire [9:0]              tlp_hdr_len;
	wire [15:0]             tlp_hdr_rid;
	wire [7:0]              tlp_hdr_tag;
	wire [7:0]              tlp_hdr_be;

	// TLP DW3_DW2
	wire [C_DATA_WIDTH-1:0] tlp_hdr_dw3_2;
	wire [31:0]             tlp_hdr_addr32;

	wire [11:0] byte_count;
	wire [6:0]  lower_addr;
	wire [31:0] bar0_addr;
	wire [31:0] pci_rd32_data;
	wire [31:0] axi_wr32_data;

	assign s_axis_cq_fire = s_axis_cq_tready && s_axis_cq_tvalid;
	assign m_axis_cc_fire = m_axis_cc_tready && m_axis_cc_tvalid;
	assign m_axi_ctrl_aw_fire = m_axi_ctrl_awready && m_axi_ctrl_awvalid;
	assign m_axi_ctrl_w_fire = m_axi_ctrl_wready && m_axi_ctrl_wvalid;
	assign m_axi_ctrl_b_fire = m_axi_ctrl_bready && m_axi_ctrl_bvalid;
	assign m_axi_ctrl_ar_fire = m_axi_ctrl_arready && m_axi_ctrl_arvalid;
	assign m_axi_ctrl_r_fire = m_axi_ctrl_rready && m_axi_ctrl_rvalid;

	assign tlp_hdr_dw1_0 = rx_dw[0];
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

	assign tlp_hdr_dw3_2 = rx_dw[1];

	assign byte_count = 12'd4;
	assign lower_addr = {tlp_hdr_addr32[6:0]};
	assign bar0_addr = (tlp_hdr_addr32 & ~C_BAR0_ADDR_MASK);
	assign pci_rd32_data = {ctrl_rx_data [0*8 +: 8], ctrl_rx_data [1*8 +: 8], ctrl_rx_data [2*8 +: 8], ctrl_rx_data [3*8 +: 8]};
	assign axi_wr32_data = {tlp_hdr_dw3_2[4*8 +: 8], tlp_hdr_dw3_2[5*8 +: 8], tlp_hdr_dw3_2[6*8 +: 8], tlp_hdr_dw3_2[7*8 +: 8]};

	assign m_axi_ctrl_awaddr = bar0_addr;
	assign m_axi_ctrl_wdata = axi_wr32_data;
	assign m_axi_ctrl_wstrb = {(CTRL_WSTRB_WIDTH){1'b1}};
	assign m_axi_ctrl_araddr = bar0_addr;

	// @FF ap_state, @FF rx_dw_idx, @FF tx_dw_idx, @FF ctrl_state
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_IDLE;
			rx_dw_idx <= 0;
			tx_dw_idx <= 0;
			ctrl_state <= CTRL_STATE_IDLE;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					ap_state <= AP_STATE_RX_DW;
					rx_dw_idx <= 0;
				end

				AP_STATE_RX_DW: begin
					if(s_axis_cq_fire) begin
						if(rx_dw_idx == 1) begin
							case(tlp_hdr_fmt_type)
								TLP_MEM_RD32_FMT_TYPE:
									if(tlp_hdr_len == 10'b1) begin
										ap_state <= AP_STATE_CTRL;
										ctrl_state <= CTRL_STATE_AR;
									end else begin
										ap_state <= AP_STATE_IDLE;
									end

								TLP_MEM_WR32_FMT_TYPE:
									if(tlp_hdr_len == 10'b1) begin
										ap_state <= AP_STATE_CTRL;
										ctrl_state <= CTRL_STATE_AW;
									end else begin
										ap_state <= AP_STATE_IDLE;
									end

								default: ap_state <= AP_STATE_IDLE;
							endcase
						end else begin
							rx_dw_idx <= rx_dw_idx + 1;
						end
					end
				end

				AP_STATE_CTRL: begin
					case(ctrl_state)
						CTRL_STATE_AW: begin
							if(m_axi_ctrl_aw_fire) begin
								ctrl_state <= CTRL_STATE_W;
							end
						end

						CTRL_STATE_W: begin
							if(m_axi_ctrl_w_fire) begin
								ctrl_state <= CTRL_STATE_B;
							end
						end

						CTRL_STATE_B: begin
							if(m_axi_ctrl_b_fire) begin
								ctrl_state <= CTRL_STATE_IDLE;
								ap_state <= AP_STATE_RX_DW;
								rx_dw_idx <= 0;
							end
						end

						CTRL_STATE_AR: begin
							if(m_axi_ctrl_ar_fire) begin
								ctrl_state <= CTRL_STATE_R;
							end
						end

						CTRL_STATE_R: begin
							if(m_axi_ctrl_r_fire) begin
								ctrl_state <= CTRL_STATE_IDLE;
								ap_state <= AP_STATE_TX_DW;
								tx_dw_idx <= 0;
							end
						end
					endcase
				end

				AP_STATE_TX_DW: begin
					if(m_axis_cc_fire) begin
						if(tx_dw_idx == 1) begin
							ap_state <= AP_STATE_RX_DW;
							rx_dw_idx <= 0;
						end else begin
							tx_dw_idx <= tx_dw_idx + 1;
						end
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

		case(ap_state)
			AP_STATE_RX_DW: begin
				s_axis_cq_tready = 1;
			end

			AP_STATE_CTRL: begin
				case(ctrl_state)
					CTRL_STATE_AW: begin
						m_axi_ctrl_awvalid = 1;
					end

					CTRL_STATE_W: begin
						m_axi_ctrl_wvalid = 1;
					end

					CTRL_STATE_B: begin
						m_axi_ctrl_bready = 1;
					end

					CTRL_STATE_AR: begin
						m_axi_ctrl_arvalid = 1;
					end

					CTRL_STATE_R: begin
						m_axi_ctrl_rready = 1;
					end
				endcase
			end

			AP_STATE_TX_DW: begin
				case(tx_dw_idx)
					0: begin
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

					1: begin
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
		endcase
	end

	// @FF rx_dw[i]
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			rx_dw[0] <= 0;
			rx_dw[1] <= 0;
		end else begin
			case(ap_state)
				AP_STATE_RX_DW: begin
					if(s_axis_cq_fire) begin
						rx_dw[rx_dw_idx] <= s_axis_cq_tdata;
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
			case(ap_state)
				AP_STATE_CTRL: begin
					case(ctrl_state)
						CTRL_STATE_R: begin
							if(m_axi_ctrl_r_fire) begin
								ctrl_rx_data <= m_axi_ctrl_rdata;
							end
						end
					endcase
				end
			endcase
		end
	end
endmodule
