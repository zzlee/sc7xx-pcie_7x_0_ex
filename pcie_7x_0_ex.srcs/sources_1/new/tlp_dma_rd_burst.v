`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/10/2025 02:07:29 PM
// Design Name: 
// Module Name: tlp_dma_rd_burst
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


module tlp_dma_rd_burst #(
	parameter C_DATA_WIDTH    = 64,
	parameter C_RC_USER_WIDTH = 22,
	parameter C_RC_COUNT      = 15,
	parameter C_RC_CNT_WIDTH  = 4,
	parameter C_BURST_SIZE    = 256, // bytes per burst

	// Do not override parameters below this line
	parameter KEEP_WIDTH = C_DATA_WIDTH / 8,
	parameter BURST_WIDTH = $clog2(C_BURST_SIZE + 1)
) (
	input clk,
	input rst_n,

	// RC TLP
	input [C_DATA_WIDTH-1:0]    s_axis_rc_tdata,
	input [KEEP_WIDTH-1:0]      s_axis_rc_tkeep,
	input                       s_axis_rc_tlast,
	input                       s_axis_rc_tvalid,
	output reg                  s_axis_rc_tready,
	input [C_RC_USER_WIDTH-1:0] s_axis_rc_tuser,

	// ap params
	input [C_RC_COUNT*BURST_WIDTH-1:0] burst_bytes,
	input [C_RC_COUNT-1:0]             req,
	output [C_RC_COUNT-1:0]            avail
);

	genvar gen_i;
	integer i;

	// fifo_dma_rd_U signals
	wire [C_DATA_WIDTH-1:0]    fifo_s_axis_tdata [C_RC_COUNT-1:0];
	wire [KEEP_WIDTH-1:0]      fifo_s_axis_tkeep [C_RC_COUNT-1:0];
	wire                       fifo_s_axis_tlast [C_RC_COUNT-1:0];
	wire                       fifo_s_axis_tvalid [C_RC_COUNT-1:0];
	wire                       fifo_s_axis_tready [C_RC_COUNT-1:0];
	wire [C_RC_USER_WIDTH-1:0] fifo_s_axis_tuser [C_RC_COUNT-1:0];
	wire [C_DATA_WIDTH-1:0]    fifo_m_axis_tdata [C_RC_COUNT-1:0];
	wire [KEEP_WIDTH-1:0]      fifo_m_axis_tkeep [C_RC_COUNT-1:0];
	wire                       fifo_m_axis_tlast [C_RC_COUNT-1:0];
	wire                       fifo_m_axis_tvalid [C_RC_COUNT-1:0];
	wire                       fifo_m_axis_tready [C_RC_COUNT-1:0];
	wire [C_RC_USER_WIDTH-1:0] fifo_m_axis_tuser [C_RC_COUNT-1:0];

	always @(*) begin
		s_axis_rc_tready = 0;
	end

	generate
		for(gen_i = 0;gen_i < C_RC_COUNT;gen_i = gen_i + 1) begin
			assign avail[gen_i] = 0;
		end
	endgenerate

	// generate
	// 	for(gen_i = 0;gen_i < C_RC_COUNT;gen_i = gen_i + 1) begin
	// 		fifo_dma_rd fifo_dma_rd_U(
	// 			.s_aclk(clk),
	// 			.s_aresetn(rst_n),

	// 			.s_axis_tvalid(fifo_s_axis_tvalid),
	// 			.s_axis_tready(fifo_s_axis_tready),
	// 			.s_axis_tdata(fifo_s_axis_tdata),
	// 			.s_axis_tkeep(fifo_s_axis_tkeep),
	// 			.s_axis_tlast(fifo_s_axis_tlast),
	// 			.s_axis_tuser(fifo_s_axis_tuser),
	// 			.m_axis_tvalid(fifo_m_axis_tvalid),
	// 			.m_axis_tready(fifo_m_axis_tready),
	// 			.m_axis_tdata(fifo_m_axis_tdata),
	// 			.m_axis_tkeep(fifo_m_axis_tkeep),
	// 			.m_axis_tlast(fifo_m_axis_tlast),
	// 			.m_axis_tuser(fifo_m_axis_tuser),

	// 			.wr_rst_busy(),
	// 			.rd_rst_busy()
	// 		);
	// 	end
	// endgenerate

	// always @(posedge clk or negedge rst_n) begin
	// 	if(~rst_n) begin
	// 		ap_state <= STATE_IDLE;
	// 	end else begin
	// 		case(ap_state)
	// 			STATE_IDLE: begin
	// 				ap_state <= STATE_RX_DW3_2;
	// 			end

	// 			STATE_RX_DW3_2: begin
	// 				if(s_axis_rx_fire) begin
	// 					if(s_axis_rx_tlast) begin
	// 						ap_state <= STATE_RX_LAST;
	// 					end
	// 				end
	// 			end

	// 			STATE_RX: begin
	// 				if(s_axis_rx_fire) begin
	// 					if(s_axis_rx_tlast) begin
	// 						ap_state <= STATE_RX_LAST;
	// 					end
	// 				end
	// 			end

	// 			STATE_RX_LAST: begin
	// 				if(fifo_dma_rd_fire[rc_idx]) begin
	// 					ap_state <= STATE_RX_DW3_2;
	// 				end
	// 			end
	// 		endcase
	// 	end
	// end

	// always @(posedge clk or negedge rst_n) begin
	// 	if(~rst_n) begin
	// 		rc_idx <= 0;
	// 	end else begin
	// 		case(ap_state)
	// 			STATE_RX_DW3_2: begin
	// 				if(s_axis_rx_fire) begin
	// 					rc_idx <= s_axis_rc_tdata[8 +: C_RC_CNT_WIDTH];
	// 				end
	// 			end
	// 		endcase
	// 	end
	// end
endmodule