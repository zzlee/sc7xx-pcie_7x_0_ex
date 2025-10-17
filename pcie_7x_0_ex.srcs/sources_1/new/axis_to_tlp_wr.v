`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 10/96/2025 10:52:29 AM
// Design Name:
// Module Name: axis_to_tlp_wr
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

module axis_to_tlp_wr #(
	parameter C_DATA_WIDTH = 64,

	// Do not override parameters below this line
	parameter KEEP_WIDTH = C_DATA_WIDTH / 8
) (
	input clk,
	input rst_n,

	// REQ signals
	input [79:0] s_axis_req_tdata,
	input        s_axis_req_tvalid,
	output reg   s_axis_req_tready,

	// RR TLP
	output reg [C_DATA_WIDTH-1:0] m_axis_rr_tdata,
	output reg [KEEP_WIDTH-1:0]   m_axis_rr_tkeep,
	output reg                    m_axis_rr_tlast,
	output reg                    m_axis_rr_tvalid,
	input                         m_axis_rr_tready,

	// SRC data
	input [C_DATA_WIDTH-1:0] s_axis_tdata,
	input                    s_axis_tvalid,
	output reg               s_axis_tready,

	input [15:0] cfg_completer_id,

	output [9:0] probe0,
	output [1:0] probe1,
	output [1:0] probe2,
	output [2:0] probe3
);
	`include "err_mask.vh"

	localparam TLP_MEM_WR32_FMT_TYPE = 7'b10_00000;
	localparam TLP_MEM_WR64_FMT_TYPE = 7'b11_00000;

	localparam DATA_FIFO_WIDTH = 32 * 5; // 5DWs

	localparam AP_STATE_REQ         = 2'd0;
	localparam AP_STATE_BURST       = 2'd1;
	localparam AP_STATE_WIDTH       = 2;

	integer i, j, k;

	reg [AP_STATE_WIDTH-1:0] ap_state;
	wire                     m_axis_rr_fire;
	wire                     s_axis_fire;
	wire [63:0]              s_axis_req_tdata_addr;
	wire [9:0]               s_axis_req_tdata_len;
	reg [63:0]               addr_int;
	reg                      addr_32bit;
	reg [1:0]                tlp_hdr_idx;
	reg [1:0]                tlp_payload_dws;
	wire                     req_addr_32bit;

	wire [7:0] tlp_hdr_tag;
	wire [3:0] tlp_hdr_last_be;
	wire [3:0] tlp_hdr_first_be;
	wire [6:0] tlp_hdr_fmt_type;
	wire [6:0] tlp_hdr_fmt_type_req;
	wire [2:0] tlp_hdr_tc;
	wire       tlp_hdr_td;
	wire       tlp_hdr_ep;
	wire [1:0] tlp_hdr_attr;
	reg [9:0]  tlp_hdr_len;
	wire [9:0] tlp_hdr_len_req;

	reg [DATA_FIFO_WIDTH-1:0] s_fifo;
	reg [2:0]                 s_fifo_dws;
	wire [31:0]               s_fifo_pci_dw0;
	wire [31:0]               s_fifo_pci_dw1;

	assign probe0 = tlp_hdr_len;
	assign probe1 = tlp_payload_dws;
	assign probe2 = tlp_hdr_idx;
	assign probe3 = s_fifo_dws;

	assign s_axis_req_tdata_addr = s_axis_req_tdata[0 +: 64];
	assign s_axis_req_tdata_len = s_axis_req_tdata[64 +: 10];
	assign m_axis_rr_fire = m_axis_rr_tvalid && m_axis_rr_tready;
	assign s_axis_fire = s_axis_tready && s_axis_tvalid;

	assign req_addr_32bit = (s_axis_req_tdata_addr[63:32] == 32'b0);

	assign tlp_hdr_tag = 8'd0; // Don't care
	assign tlp_hdr_last_be = 4'b1111;
	assign tlp_hdr_first_be = 4'b1111;
	assign tlp_hdr_fmt_type = (addr_32bit ? TLP_MEM_WR32_FMT_TYPE : TLP_MEM_WR64_FMT_TYPE);
	assign tlp_hdr_fmt_type_req = (req_addr_32bit ? TLP_MEM_WR32_FMT_TYPE : TLP_MEM_WR64_FMT_TYPE);
	assign tlp_hdr_tc = 3'b0;
	assign tlp_hdr_td = 1'b0;
	assign tlp_hdr_ep = 1'b0;
	assign tlp_hdr_attr = 2'b0;
	assign tlp_hdr_len_req = s_axis_req_tdata_len;

	assign s_fifo_pci_dw0 = {s_fifo[0*8 +: 8], s_fifo[1*8 +: 8], s_fifo[2*8 +: 8], s_fifo[3*8 +: 8]};
	assign s_fifo_pci_dw1 = {s_fifo[4*8 +: 8], s_fifo[5*8 +: 8], s_fifo[6*8 +: 8], s_fifo[7*8 +: 8]};

	// @FF ap_state, @FF addr_int, @FF addr_32bit, @FF tlp_hdr_idx, @FF tlp_hdr_len
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_REQ;
			addr_int <= 0;
			addr_32bit <= 0;
			tlp_hdr_idx <= 0;
			tlp_hdr_len <= 0;
		end else begin
			case(ap_state)
				AP_STATE_REQ: begin
					if(s_axis_req_tvalid) begin
						addr_int <= s_axis_req_tdata_addr;
						addr_32bit <= req_addr_32bit;

						tlp_hdr_idx <= 0;
						tlp_hdr_len <= s_axis_req_tdata_len;

						ap_state <= AP_STATE_BURST;
					end
				end

				AP_STATE_BURST: begin
					if(m_axis_rr_fire) begin
						case(tlp_hdr_idx)
							0: begin
								tlp_hdr_idx <= 1;
							end

							1, 2: begin
								tlp_hdr_idx <= 2;
								tlp_hdr_len <= tlp_hdr_len - tlp_payload_dws;

								if(m_axis_rr_tlast) begin
									ap_state <= AP_STATE_REQ;
								end
							end
						endcase
					end
				end
			endcase
		end
	end

	// @COMB m_axis_rr_tdata, @COMB m_axis_rr_tkeep, @COMB m_axis_rr_tlast, @COMB m_axis_rr_tvalid
	always @(*) begin
		m_axis_rr_tdata = 'hAABBCCDDCAFECAFE; // for debug purpose
		m_axis_rr_tkeep = 0;
		m_axis_rr_tlast = 0;
		m_axis_rr_tvalid = 0;

		case(ap_state)
			AP_STATE_BURST: begin
				case(tlp_hdr_idx)
					0: begin
						m_axis_rr_tlast = 0;
						m_axis_rr_tdata = {   // Bits
							// DW1
							cfg_completer_id, // 16
							tlp_hdr_tag,      // 8
							tlp_hdr_last_be,  // 4
							tlp_hdr_first_be, // 4
							// DW0
							{1'b0},           // 1
							tlp_hdr_fmt_type, // 7
							{1'b0},           // 1
							tlp_hdr_tc,       // 3
							{4'b0},           // 4
							tlp_hdr_td,       // 1
							tlp_hdr_ep,       // 1
							tlp_hdr_attr,     // 2
							{2'b0},           // 2
							tlp_hdr_len       // 10
						};
						m_axis_rr_tkeep = 8'hFF;
						m_axis_rr_tvalid = 1;
					end

					1: begin
						m_axis_rr_tkeep = 8'hFF;

						if(addr_32bit) begin
							m_axis_rr_tlast = (tlp_hdr_len == 1);
							m_axis_rr_tdata = {
								// DW3 Data
								s_fifo_pci_dw0,
								// DW2 Addr LO
								{addr_int[31:2], 2'b00}
							};
							m_axis_rr_tvalid = (s_fifo_dws >= tlp_payload_dws);
						end else begin
							m_axis_rr_tlast = 0;
							m_axis_rr_tdata = {
								// DW3 Addr LO
								{addr_int[31:2], 2'b00},
								// DW2 Addr HI
								addr_int[63:32]
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

	// @COMB s_axis_req_tready
	always @(*) begin
		s_axis_req_tready = 0;

		case(ap_state)
			AP_STATE_REQ: begin
				s_axis_req_tready = 1;
			end
		endcase
	end

	// @COMB s_axis_tready
	always @(*) begin
		s_axis_tready = 0;

		case(ap_state)
			AP_STATE_BURST: begin
				case(tlp_hdr_idx)
					1, 2: begin
						if(s_fifo_dws <= ((DATA_FIFO_WIDTH - C_DATA_WIDTH) / 32)) begin
							s_axis_tready = 1;
						end
					end
				endcase
			end
		endcase
	end

	// @COMB tlp_payload_dws
	always @(*) begin
		tlp_payload_dws = 0;

		case(ap_state)
			AP_STATE_BURST: begin
				case(tlp_hdr_idx)
					1: begin
						tlp_payload_dws = addr_32bit ? 1 : 0;
					end

					2: begin
						tlp_payload_dws = (m_axis_rr_tkeep == 'hFF ? 2 : 1);
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
				AP_STATE_BURST: begin
					case(tlp_hdr_idx)
						1, 2: begin
							case({s_axis_fire, m_axis_rr_fire})
								2'b01: begin
									// if (s_fifo_dws > tlp_payload_dws) begin
									// 	for (i = 0; i < (s_fifo_dws - tlp_payload_dws); i = i + 1) begin
									// 		s_fifo[i*32 +: 32] <= s_fifo[(i + tlp_payload_dws)*32 +: 32];
									// 	end
									// 	s_fifo_dws <= s_fifo_dws - tlp_payload_dws;
									// end

									for(i = 1;i <= 2;i = i + 1) begin
										if(tlp_payload_dws == i) begin
											for(j = 2;j <= (DATA_FIFO_WIDTH / 32);j = j + 1) begin
												if(s_fifo_dws == j && j > i) begin
													for (k = 0; k < (j - i); k = k + 1) begin
														s_fifo[k*32 +: 32] <= s_fifo[(k+i)*32 +: 32];
													end

													s_fifo_dws <= j - i;
												end
											end
										end
									end

									// // s_fifo[0*32 +: (s_fifo_dws-tlp_payload_dws)*32] <= s_fifo[tlp_payload_dws*32 +: (s_fifo_dws-tlp_payload_dws)*32];
									// // s_fifo_dws <= s_fifo_dws - tlp_payload_dws;
									// case(tlp_payload_dws)
									// 	1:
									// 		case(s_fifo_dws)
									// 			2: begin
									// 				s_fifo[0*32 +: 1*32] <= s_fifo[1*32 +: 1*32];
									// 				s_fifo_dws <= 1;
									// 			end
									// 			3: begin
									// 				s_fifo[0*32 +: 2*32] <= s_fifo[1*32 +: 2*32];
									// 				s_fifo_dws <= 2;
									// 			end
									// 			4: begin
									// 				s_fifo[0*32 +: 3*32] <= s_fifo[1*32 +: 3*32];
									// 				s_fifo_dws <= 3;
									// 			end
									// 			5: begin
									// 				s_fifo[0*32 +: 4*32] <= s_fifo[1*32 +: 4*32];
									// 				s_fifo_dws <= 4;
									// 			end
									// 			default: ;
									// 		endcase
									// 	2:
									// 		case(s_fifo_dws)
									// 			3: begin
									// 				s_fifo[0*32 +: 1*32] <= s_fifo[2*32 +: 1*32];
									// 				s_fifo_dws <= 1;
									// 			end
									// 			4: begin
									// 				s_fifo[0*32 +: 2*32] <= s_fifo[2*32 +: 2*32];
									// 				s_fifo_dws <= 2;
									// 			end
									// 			5: begin
									// 				s_fifo[0*32 +: 3*32] <= s_fifo[2*32 +: 3*32];
									// 				s_fifo_dws <= 3;
									// 			end
									// 			default: ;
									// 		endcase
									// 	default: ;
									// endcase
								end

								2'b10: begin
									for(i = 0;i <= ((DATA_FIFO_WIDTH - C_DATA_WIDTH) / 32);i = i + 1) begin
										if(s_fifo_dws == i) begin
											s_fifo[i*32 +: C_DATA_WIDTH] <= s_axis_tdata;
											s_fifo_dws <= i + (C_DATA_WIDTH / 32);
										end
									end

									// // s_fifo[s_fifo_dws*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// // s_fifo_dws <= s_fifo_dws + (C_DATA_WIDTH / 32);
									// case(s_fifo_dws)
									// 	0: begin
									// 		s_fifo[0*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 		s_fifo_dws <= 0 + (C_DATA_WIDTH / 32);
									// 	end
									// 	1: begin
									// 		s_fifo[1*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 		s_fifo_dws <= 1 + (C_DATA_WIDTH / 32);
									// 	end
									// 	2: begin
									// 		s_fifo[2*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 		s_fifo_dws <= 2 + (C_DATA_WIDTH / 32);
									// 	end
									// 	3: begin
									// 		s_fifo[3*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 		s_fifo_dws <= 3 + (C_DATA_WIDTH / 32);
									// 	end
									// 	default: ;
									// endcase
								end

								2'b11: begin
									for(i = 0;i <= 2;i = i + 1) begin
										if(tlp_payload_dws == i) begin
											for(j = i;j <= ((DATA_FIFO_WIDTH - C_DATA_WIDTH) / 32 + i);j = j + 1) begin
												if(s_fifo_dws == j) begin
													if(i > 0 && j > i) begin
														for (k = 0; k < (j - i); k = k + 1) begin
															s_fifo[k*32 +: 32] <= s_fifo[(k+i)*32 +: 32];
														end
													end
													s_fifo[(j-i)*32 +: C_DATA_WIDTH] <= s_axis_tdata;
													s_fifo_dws <= (j - i) + (C_DATA_WIDTH / 32);
												end
											end
										end
									end

									// // s_fifo[0*32 +: (s_fifo_dws-tlp_payload_dws)*32] <= s_fifo[tlp_payload_dws*32 +: (s_fifo_dws-tlp_payload_dws)*32];
									// // s_fifo[(s_fifo_dws-tlp_payload_dws)*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// // s_fifo_dws <= (s_fifo_dws - tlp_payload_dws) + (C_DATA_WIDTH / 32);
									// case(tlp_payload_dws)
									// 	0:
									// 		case(s_fifo_dws)
									// 			0: begin
									// 				s_fifo[0*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 				s_fifo_dws <= 0 + (C_DATA_WIDTH / 32);
									// 			end
									// 			1: begin
									// 				s_fifo[1*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 				s_fifo_dws <= 1 + (C_DATA_WIDTH / 32);
									// 			end
									// 			2: begin
									// 				s_fifo[2*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 				s_fifo_dws <= 2 + (C_DATA_WIDTH / 32);
									// 			end
									// 			3: begin
									// 				s_fifo[3*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 				s_fifo_dws <= 3 + (C_DATA_WIDTH / 32);
									// 			end
									// 			default: ;
									// 		endcase
									// 	1:
									// 		case(s_fifo_dws)
									// 			1: begin
									// 				s_fifo[0*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 				s_fifo_dws <= (C_DATA_WIDTH / 32);
									// 			end
									// 			2: begin
									// 				s_fifo[0*32 +: 1*32] <= s_fifo[1*32 +: 1*32];
									// 				s_fifo[1*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 				s_fifo_dws <= 1 + (C_DATA_WIDTH / 32);
									// 			end
									// 			3: begin
									// 				s_fifo[0*32 +: 2*32] <= s_fifo[1*32 +: 2*32];
									// 				s_fifo[2*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 				s_fifo_dws <= 2 + (C_DATA_WIDTH / 32);
									// 			end
									// 			4: begin
									// 				s_fifo[0*32 +: 3*32] <= s_fifo[1*32 +: 3*32];
									// 				s_fifo[3*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 				s_fifo_dws <= 3 + (C_DATA_WIDTH / 32);
									// 			end
									// 			default: ;
									// 		endcase
									// 	2:
									// 		case(s_fifo_dws)
									// 			2: begin
									// 				s_fifo[0*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 				s_fifo_dws <= 0 + (C_DATA_WIDTH / 32);
									// 			end
									// 			3: begin
									// 				s_fifo[0*32 +: 1*32] <= s_fifo[2*32 +: 1*32];
									// 				s_fifo[1*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 				s_fifo_dws <= 1 + (C_DATA_WIDTH / 32);
									// 			end
									// 			4: begin
									// 				s_fifo[0*32 +: 2*32] <= s_fifo[2*32 +: 2*32];
									// 				s_fifo[2*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 				s_fifo_dws <= 2 + (C_DATA_WIDTH / 32);
									// 			end
									// 			5: begin
									// 				s_fifo[0*32 +: 3*32] <= s_fifo[2*32 +: 3*32];
									// 				s_fifo[3*32 +: C_DATA_WIDTH] <= s_axis_tdata;
									// 				s_fifo_dws <= 3 + (C_DATA_WIDTH / 32);
									// 			end
									// 			default: ;
									// 		endcase
									// 	default: ;
									// endcase
								end
							endcase
						end
					endcase
				end
			endcase
		end
	end

endmodule
