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
	input [C_DATA_WIDTH-1:0]    m_axis_rc_tdata,
	input [KEEP_WIDTH-1:0]      m_axis_rc_tkeep,
	input                       m_axis_rc_tlast,
	input                       m_axis_rc_tvalid,
	output reg                  m_axis_rc_tready,
	input [C_RC_USER_WIDTH-1:0] m_axis_rc_tuser,

	input [C_RC_COUNT*BURST_WIDTH-1:0] rc_burst_bytes,
	input [C_RC_COUNT-1:0]             rc_req,
	output [C_RC_COUNT-1:0]            rc_avail
);

	localparam STATE_IDLE  = 2'd0;
	localparam STATE_RECV  = 2'd1;
	localparam STATE_LAST  = 2'd2;
	localparam STATE_UNEXP = 2'd3;
	localparam STATE_BITS  = 2;

	localparam RC_STATE_IDLE  = 2'd0;
	localparam RC_STATE_RECV  = 2'd1;
	localparam RC_STATE_LAST  = 2'd2;
	localparam RC_STATE_UNEXP = 2'd3;
	localparam RC_STATE_BITS  = 2;

	integer i;
	genvar gen_i;

	reg [STATE_BITS-1:0]     ap_state;
	reg [C_RC_CNT_WIDTH-1:0] rc_idx;

	wire m_axis_rc_fire;

	reg [RC_STATE_BITS-1:0]    rc_state [C_RC_COUNT-1:0];
	reg [BURST_WIDTH-1:0]      rc_burst_bytes_int [C_RC_COUNT-1:0];

	wire [C_DATA_WIDTH-1:0]    rc_tdata [C_RC_COUNT-1:0];
	wire [KEEP_WIDTH-1:0]      rc_tkeep [C_RC_COUNT-1:0];
	wire                       rc_tlast [C_RC_COUNT-1:0];
	reg                        rc_tvalid [C_RC_COUNT-1:0];
	reg                        rc_tready [C_RC_COUNT-1:0];
	wire [C_RC_USER_WIDTH-1:0] rc_tuser [C_RC_COUNT-1:0];
	wire                       rc_fire [C_RC_COUNT-1:0];

	reg [C_DATA_WIDTH-1:0] tdata_q;
	reg [KEEP_WIDTH-1:0]   tkeep_q;
	reg                    tlast_q;
	reg                    tuser_q; // SOF

	// @FF ap_state, @FF rc_idx
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= STATE_IDLE;
		end else begin
			case(ap_state)
				STATE_IDLE: begin
					if(m_axis_rc_fire) begin
						if(m_axis_rc_tdata[8 +: C_RC_CNT_WIDTH] < C_RC_COUNT) begin
							ap_state <= STATE_RECV;
							rc_idx <= m_axis_rc_tdata[8 +: C_RC_CNT_WIDTH];

							if(m_axis_rc_tlast) begin
								ap_state <= STATE_LAST;
							end
						end else begin
							ap_state <= STATE_UNEXP;
							if(m_axis_rc_tlast) begin
								ap_state <= STATE_IDLE;
							end
						end
					end
				end

				STATE_RECV: begin
					if(m_axis_rc_fire) begin
						if(m_axis_rc_tlast) begin
							ap_state <= STATE_LAST;
						end
					end
				end

				STATE_LAST: begin
					if(m_axis_rc_fire) begin
						ap_state <= STATE_IDLE;
					end
				end

				STATE_UNEXP: begin
					if(m_axis_rc_fire) begin
						if(m_axis_rc_tlast) begin
							ap_state <= STATE_IDLE;
						end
					end
				end
			endcase
		end
	end

	// @COMB m_axis_rc_tready
	always @(*) begin
		m_axis_rc_tready = 0;

		case (ap_state)
			STATE_IDLE: begin
				m_axis_rc_tready = 1;
			end

			STATE_RECV: begin
				m_axis_rc_tready = rc_tready[rc_idx];
			end

			STATE_UNEXP: begin
				m_axis_rc_tready = 1;
			end
		endcase
	end

	assign m_axis_rc_fire = m_axis_rc_tready && m_axis_rc_tvalid;

	// @FF tdata_q, @FF tkeep_q, @FF tlast_q, @FF tuser_q
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			tdata_q <= 0;
			tkeep_q <= 0;
			tlast_q <= 0;
			tuser_q <= 0;
		end else begin
			case(ap_state)
				STATE_IDLE: begin
					if(m_axis_rc_fire) begin
						tdata_q <= m_axis_rc_tdata;
						tkeep_q <= m_axis_rc_tkeep;
						tlast_q <= m_axis_rc_tlast;
						tuser_q <= 1;
					end
				end

				STATE_RECV: begin
					if(m_axis_rc_fire) begin
						tdata_q <= m_axis_rc_tdata;
						tkeep_q <= m_axis_rc_tkeep;
						tlast_q <= m_axis_rc_tlast;
						tuser_q <= 0;
					end
				end
			endcase
		end
	end

	// @FF rc_state[i]
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			for(i = 0;i < C_RC_COUNT;i = i + 1) begin
				rc_state[i] <= RC_STATE_IDLE;
				rc_burst_bytes_int[i] <= 0;
			end
		end else begin
			for(i = 0;i < C_RC_COUNT;i = i + 1) begin
				case(rc_state[i])
					RC_STATE_IDLE: begin
						if(rc_req[i]) begin
							rc_state[i] <= RC_STATE_RECV;
							rc_burst_bytes_int[i] <= rc_burst_bytes[i*BURST_WIDTH +: BURST_WIDTH];
						end
					end

					RC_STATE_RECV: begin
						if(rc_fire[i]) begin
							if(rc_tlast[i]) begin
								rc_state[i] <= RC_STATE_IDLE;
							end
						end
					end
				endcase
			end
		end
	end

	generate
		for(gen_i = 0;gen_i < C_RC_COUNT;gen_i = gen_i + 1) begin
			assign rc_tdata[gen_i] = tdata_q;
			assign rc_tkeep[gen_i] = tkeep_q;
			assign rc_tlast[gen_i] = tlast_q;
			assign rc_tuser[gen_i] = tuser_q;
			assign rc_fire[gen_i] = rc_tready[gen_i] && rc_tvalid[gen_i];
		end
	endgenerate

	// @COMB rc_tready[i]
	always @(*) begin
		rc_tready[i] = 0;

		for(i = 0;i < C_RC_COUNT;i = i + 1) begin
			case(rc_state[i])
				RC_STATE_RECV: begin
					rc_tready[i] = 1;
				end
			endcase
		end
	end

	// @COMB rc_tvalid[i]
	always @(*) begin
		for(i = 0; i < C_RC_COUNT; i = i + 1) begin
			rc_tvalid[i] = 0;
		end

		case(ap_state)
			STATE_RECV: begin
				rc_tvalid[rc_idx] = m_axis_rc_tvalid;
			end

			STATE_LAST: begin
				rc_tvalid[rc_idx] = 1;
			end
		endcase
	end
endmodule