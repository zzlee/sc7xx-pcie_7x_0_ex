`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/15/2025 08:21:23 PM
// Design Name: 
// Module Name: tlp_demuxer_cq
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

module tlp_demuxer_cq #(
	parameter C_DATA_WIDTH    = 64,
	parameter C_USER_WIDTH    = 22,
	parameter C_TLP_HDR_COUNT = 2,

	// Do not override parameters below this line
	parameter KEEP_WIDTH = C_DATA_WIDTH / 8
)
(
	input clk,
	input rst_n,

	// RX
	input [C_DATA_WIDTH-1:0] s_axis_rx_tdata,
	input [KEEP_WIDTH-1:0]   s_axis_rx_tkeep,
	input                    s_axis_rx_tlast,
	input                    s_axis_rx_tvalid,
	output reg               s_axis_rx_tready,
	input [C_USER_WIDTH-1:0] s_axis_rx_tuser,

	// CQ
	output reg [C_DATA_WIDTH-1:0] m_axis_cq_tdata,
	output reg [KEEP_WIDTH-1:0]   m_axis_cq_tkeep,
	output reg                    m_axis_cq_tlast,
	output reg                    m_axis_cq_tvalid,
	input                         m_axis_cq_tready,

	// ap args
	input [C_DATA_WIDTH*C_TLP_HDR_COUNT-1:0] s_axis_ap_tlp_hdr_tdata,
	input [KEEP_WIDTH*C_TLP_HDR_COUNT-1:0]   s_axis_ap_tlp_hdr_tkeep,
	input [C_TLP_HDR_COUNT-1:0]              s_axis_ap_tlp_hdr_tlast,
	input                                    s_axis_ap_tvalid,
	output reg                               s_axis_ap_tready
);
	localparam AP_STATE_IDLE  = 1'd0;
	localparam AP_STATE_DEMUX = 1'd1;
	localparam AP_STATE_WIDTH = 1;

	integer i;
	genvar gen_i;

	reg [AP_STATE_WIDTH-1:0] ap_state;
	wire                     s_axis_ap_fire;
	wire                     s_axis_rx_fire;
	wire                     m_axis_cq_fire;

	reg                         demux_fifo; // demux from FIFO
	reg [C_DATA_WIDTH-1:0]      fifo_tdata [C_TLP_HDR_COUNT-1:0];
	reg [KEEP_WIDTH-1:0]        fifo_tkeep [C_TLP_HDR_COUNT-1:0];
	reg                         fifo_tlast [C_TLP_HDR_COUNT-1:0];

	assign s_axis_ap_fire = s_axis_ap_tready && s_axis_ap_tvalid;
	assign s_axis_rx_fire = s_axis_rx_tready && s_axis_rx_tvalid;
	assign m_axis_cq_fire = m_axis_cq_tready && m_axis_cq_tvalid;

	// @FF ap_state, @FF demux_fifo
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_IDLE;
			demux_fifo <= 0;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					if(s_axis_ap_fire) begin
						ap_state <= AP_STATE_DEMUX;
						demux_fifo <= 0;
					end
				end

				AP_STATE_DEMUX: begin
					if(demux_fifo) begin
						if(m_axis_cq_fire) begin
							if(m_axis_cq_tlast) begin
								ap_state <= AP_STATE_IDLE;
							end
					end else begin
						if(s_axis_rx_fire) begin
							if(s_axis_rx_tlast) begin
								demux_fifo <= 1; // switch to fifo
							end
						end
					end
				end
			endcase
		end
	end

	// @COMB s_axis_ap_tready, @COMB s_axis_rx_tready
	always @(*) begin
		s_axis_ap_tready = 0;
		s_axis_rx_tready = 0;

		case(ap_state)
			AP_STATE_IDLE: begin
				s_axis_ap_tready = 1;
			end

			AP_STATE_DEMUX: begin
				s_axis_rx_tready = ~demux_fifo;
			end
		endcase
	end

	// @COMB m_axis_cq_tdata, @COMB m_axis_cq_tkeep, @COMB m_axis_cq_tlast, @COMB m_axis_cq_tvalid
	always @(*) begin
		m_axis_cq_tdata = 'hCAFE0002;
		m_axis_cq_tkeep = 0;
		m_axis_cq_tlast = 0;
		m_axis_cq_tvalid = 0;

		case(ap_state)
			AP_STATE_DEMUX: begin
				m_axis_cq_tdata = fifo_tdata[0];
				m_axis_cq_tkeep = fifo_tkeep[0];
				m_axis_cq_tlast = fifo_tlast[0];

				if(demux_fifo) begin
					m_axis_cq_tvalid = 1;
				end else begin
					m_axis_cq_tvalid = s_axis_rx_tvalid;
				end
			end
		endcase
	end

	// @FF fifo_tdata[i], @FF fifo_tkeep[i], @FF fifo_tlast[i]
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			for(i = 0;i < C_TLP_HDR_COUNT;i = i + 1) begin
				fifo_tdata[i] <= 'hCAFE0003;
			end
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					if(s_axis_ap_fire) begin
						for(i = 0;i < C_TLP_HDR_COUNT;i = i + 1) begin
							fifo_tdata[i] <= s_axis_ap_tlp_hdr_tdata[i*C_DATA_WIDTH +: C_DATA_WIDTH];
							fifo_tkeep[i] <= s_axis_ap_tlp_hdr_tkeep[i*C_DATA_WIDTH +: C_DATA_WIDTH];
							fifo_tlast[i] <= s_axis_ap_tlp_hdr_tlast[i*C_DATA_WIDTH +: C_DATA_WIDTH];
						end
					end
				end

				AP_STATE_DEMUX: begin
					if(demux_fifo) begin
						if(m_axis_cq_fire) begin
							fifo_tdata[0] <= fifo_tdata[1];
							fifo_tkeep[0] <= fifo_tkeep[1];
							fifo_tlast[0] <= fifo_tlast[1];
							fifo_tdata[1] <= 'hCAFE0001; // for debug purpose
							fifo_tkeep[1] <= 0;
							fifo_tlast[1] <= 0;
						end
					end else begin
						if(s_axis_rx_fire) begin
							fifo_tdata[0] <= fifo_tdata[1];
							fifo_tkeep[0] <= fifo_tkeep[1];
							fifo_tlast[0] <= fifo_tlast[1];
							fifo_tdata[1] <= s_axis_rx_tdata;
							fifo_tkeep[1] <= s_axis_rx_tkeep;
							fifo_tlast[1] <= s_axis_rx_tlast;
						end
					end
				end
			endcase
		end
	end
endmodule