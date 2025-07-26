`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/15/2025 08:21:23 PM
// Design Name: 
// Module Name: tlp_demuxer
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

module tlp_demuxer #(
	parameter C_DATA_WIDTH = 64,
	parameter C_USER_WIDTH = 22,

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
	output [C_DATA_WIDTH-1:0] m_axis_cq_tdata,
	output [KEEP_WIDTH-1:0]   m_axis_cq_tkeep,
	output                    m_axis_cq_tlast,
	output                    m_axis_cq_tvalid,
	input                     m_axis_cq_tready,

	// RC00 DESC
	output [C_DATA_WIDTH-1:0] m00_axis_rc_desc_tdata,
	output [KEEP_WIDTH-1:0]   m00_axis_rc_desc_tkeep,
	output                    m00_axis_rc_desc_tlast,
	output                    m00_axis_rc_desc_tvalid,
	input                     m00_axis_rc_desc_tready,

	// RC01 DESC
	output [C_DATA_WIDTH-1:0] m01_axis_rc_desc_tdata,
	output [KEEP_WIDTH-1:0]   m01_axis_rc_desc_tkeep,
	output                    m01_axis_rc_desc_tlast,
	output                    m01_axis_rc_desc_tvalid,
	input                     m01_axis_rc_desc_tready,

	// RC00 GRP
	output [C_DATA_WIDTH-1:0] m00_axis_rc_grp_tdata,
	output [KEEP_WIDTH-1:0]   m00_axis_rc_grp_tkeep,
	output                    m00_axis_rc_grp_tlast,
	output                    m00_axis_rc_grp_tvalid,
	input                     m00_axis_rc_grp_tready,

	// RC01 GRP
	output [C_DATA_WIDTH-1:0] m01_axis_rc_grp_tdata,
	output [KEEP_WIDTH-1:0]   m01_axis_rc_grp_tkeep,
	output                    m01_axis_rc_grp_tlast,
	output                    m01_axis_rc_grp_tvalid,
	input                     m01_axis_rc_grp_tready,

	input [15:0] cfg_completer_id
);
	localparam TLP_MEM_TYPE = 5'b00000;
	localparam TLP_CPL_TYPE = 5'b01010;

	localparam RC_DESC_GRP       = 4'b1111;
	localparam RC_DESC_COUNT     = 2;

	localparam RC_GRP_COUNT     = 2;

	localparam AP_STATE_IDLE          = 3'd0;
	localparam AP_STATE_TLP_HDR       = 3'd1;
	localparam AP_STATE_DEMUX_CQ      = 3'd2;
	localparam AP_STATE_DEMUX_RC_DESC = 3'd3;
	localparam AP_STATE_DEMUX_RC_GRP  = 3'd4;
	localparam AP_STATE_DEMUX_ERR     = 3'd5;
	localparam AP_STATE_WIDTH         = 3;

	localparam TLP_HDR_COUNT = 2;
	localparam TLP_HDR_CNT_WIDTH = $clog2(TLP_HDR_COUNT);

	integer i;

	reg [AP_STATE_WIDTH-1:0]    ap_state;
	reg [TLP_HDR_CNT_WIDTH-1:0] tlp_hdr_idx;
	reg [C_DATA_WIDTH-1:0]      tlp_hdr_tdata [TLP_HDR_COUNT-1:0];
	reg [KEEP_WIDTH-1:0]        tlp_hdr_tkeep [TLP_HDR_COUNT-1:0];
	reg                         tlp_hdr_tlast [TLP_HDR_COUNT-1:0];
	wire [4:0]                  tlp_hdr_tdata_type;
	reg                         tlp_demuxer_cq_busy;
	reg                         tlp_demuxer_rc_desc_busy;
	reg                         tlp_demuxer_rc_grp_busy;

	// RC DESCs
	wire [C_DATA_WIDTH-1:0] m_axis_rc_desc_tdata [RC_DESC_COUNT-1:0];
	wire [KEEP_WIDTH-1:0]   m_axis_rc_desc_tkeep [RC_DESC_COUNT-1:0];
	wire                    m_axis_rc_desc_tlast [RC_DESC_COUNT-1:0];
	wire                    m_axis_rc_desc_tvalid [RC_DESC_COUNT-1:0];
	wire                    m_axis_rc_desc_tready [RC_DESC_COUNT-1:0];

	// RC GRPs
	wire [C_DATA_WIDTH-1:0] m_axis_rc_grp_tdata [RC_GRP_COUNT-1:0];
	wire [KEEP_WIDTH-1:0]   m_axis_rc_grp_tkeep [RC_GRP_COUNT-1:0];
	wire                    m_axis_rc_grp_tlast [RC_GRP_COUNT-1:0];
	wire                    m_axis_rc_grp_tvalid [RC_GRP_COUNT-1:0];
	wire                    m_axis_rc_grp_tready [RC_GRP_COUNT-1:0];

	wire s_axis_rx_fire;

	// tlp_demuxer_cq_U signals
	reg [C_DATA_WIDTH-1:0]               tlp_demuxer_cq_s_axis_rx_tdata;
	reg [KEEP_WIDTH-1:0]                 tlp_demuxer_cq_s_axis_rx_tkeep;
	reg                                  tlp_demuxer_cq_s_axis_rx_tlast;
	reg                                  tlp_demuxer_cq_s_axis_rx_tvalid;
	wire                                 tlp_demuxer_cq_s_axis_rx_tready;
	reg [C_USER_WIDTH-1:0]               tlp_demuxer_cq_s_axis_rx_tuser;
	wire                                 tlp_demuxer_cq_s_axis_rx_fire;
	reg [C_DATA_WIDTH*TLP_HDR_COUNT-1:0] tlp_demuxer_cq_s_axis_ap_tlp_hdr_tdata;
	reg [KEEP_WIDTH*TLP_HDR_COUNT-1:0]   tlp_demuxer_cq_s_axis_ap_tlp_hdr_tkeep;
	reg [TLP_HDR_COUNT-1:0]              tlp_demuxer_cq_s_axis_ap_tlp_hdr_tlast;
	reg                                  tlp_demuxer_cq_s_axis_ap_tvalid;
	wire                                 tlp_demuxer_cq_s_axis_ap_tready;
	wire                                 tlp_demuxer_cq_s_axis_ap_fire;

	// tlp_demuxer_rc_desc_U signals
	reg [C_DATA_WIDTH-1:0]               tlp_demuxer_rc_desc_s_axis_rx_tdata;
	reg [KEEP_WIDTH-1:0]                 tlp_demuxer_rc_desc_s_axis_rx_tkeep;
	reg                                  tlp_demuxer_rc_desc_s_axis_rx_tlast;
	reg                                  tlp_demuxer_rc_desc_s_axis_rx_tvalid;
	wire                                 tlp_demuxer_rc_desc_s_axis_rx_tready;
	reg [C_USER_WIDTH-1:0]               tlp_demuxer_rc_desc_s_axis_rx_tuser;
	wire                                 tlp_demuxer_rc_desc_s_axis_rx_fire;
	reg [C_DATA_WIDTH*TLP_HDR_COUNT-1:0] tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tdata;
	reg [KEEP_WIDTH*TLP_HDR_COUNT-1:0]   tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tkeep;
	reg [TLP_HDR_COUNT-1:0]              tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tlast;
	reg                                  tlp_demuxer_rc_desc_s_axis_ap_tvalid;
	wire                                 tlp_demuxer_rc_desc_s_axis_ap_tready;
	wire                                 tlp_demuxer_rc_desc_s_axis_ap_fire;

	// tlp_demuxer_rc_grp_U signals
	reg [C_DATA_WIDTH-1:0]               tlp_demuxer_rc_grp_s_axis_rx_tdata;
	reg [KEEP_WIDTH-1:0]                 tlp_demuxer_rc_grp_s_axis_rx_tkeep;
	reg                                  tlp_demuxer_rc_grp_s_axis_rx_tlast;
	reg                                  tlp_demuxer_rc_grp_s_axis_rx_tvalid;
	wire                                 tlp_demuxer_rc_grp_s_axis_rx_tready;
	reg [C_USER_WIDTH-1:0]               tlp_demuxer_rc_grp_s_axis_rx_tuser;
	wire                                 tlp_demuxer_rc_grp_s_axis_rx_fire;
	reg [C_DATA_WIDTH*TLP_HDR_COUNT-1:0] tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tdata;
	reg [KEEP_WIDTH*TLP_HDR_COUNT-1:0]   tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tkeep;
	reg [TLP_HDR_COUNT-1:0]              tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tlast;
	reg                                  tlp_demuxer_rc_grp_s_axis_ap_tvalid;
	wire                                 tlp_demuxer_rc_grp_s_axis_ap_tready;
	wire                                 tlp_demuxer_rc_grp_s_axis_ap_fire;

	assign tlp_hdr_tdata_type = tlp_hdr_tdata[0][28:24];

	// @FF ap_state, @FF tlp_hdr_idx, @FF tlp_hdr_tdata[i], @FF tlp_hdr_tkeep[i], @FF tlp_hdr_tlast[i],
	// @FF tlp_demuxer_cq_busy, @FF tlp_demuxer_rc_desc_busy, @FF tlp_demuxer_rc_grp_busy
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ap_state <= AP_STATE_IDLE;
			tlp_hdr_idx <= 0;
			for(i = 0;i < TLP_HDR_COUNT;i = i + 1) begin
				tlp_hdr_tdata[i] <= 0;
				tlp_hdr_tkeep[i] <= 0;
				tlp_hdr_tlast[i] <= 0;
			end
			tlp_demuxer_cq_busy <= 0;
			tlp_demuxer_rc_desc_busy <= 0;
			tlp_demuxer_rc_grp_busy <= 0;
		end else begin
			case(ap_state)
				AP_STATE_IDLE: begin
					ap_state <= AP_STATE_TLP_HDR;
					tlp_hdr_idx <= 0;
				end

				AP_STATE_TLP_HDR: begin
					if(s_axis_rx_fire) begin
						tlp_hdr_tdata[tlp_hdr_idx] <= s_axis_rx_tdata;
						tlp_hdr_tkeep[tlp_hdr_idx] <= s_axis_rx_tkeep;
						tlp_hdr_tlast[tlp_hdr_idx] <= s_axis_rx_tlast;
						tlp_hdr_idx <= tlp_hdr_idx + 1;

						if(tlp_hdr_idx == 1) begin
							case(tlp_hdr_tdata_type)
								TLP_MEM_TYPE: begin
									ap_state <= AP_STATE_DEMUX_CQ;
									tlp_demuxer_cq_busy <= 1;
								end

								TLP_CPL_TYPE: begin
									if(s_axis_rx_tdata[31:16] == cfg_completer_id) begin
										if(s_axis_rx_tdata[(8+4) +: 4] == RC_DESC_GRP) begin
											ap_state <= AP_STATE_DEMUX_RC_DESC;
											tlp_demuxer_rc_desc_busy <= 1;
										end else begin
											ap_state <= AP_STATE_DEMUX_RC_GRP;
											tlp_demuxer_rc_grp_busy <= 1;
										end
									end else begin
										ap_state <= AP_STATE_DEMUX_ERR;
									end
								end

								default: begin
									ap_state <= AP_STATE_DEMUX_ERR;
								end
							endcase
						end
					end
				end

				AP_STATE_DEMUX_CQ: begin
					if(tlp_demuxer_cq_busy) begin
						if(tlp_demuxer_cq_s_axis_ap_fire) begin
							tlp_demuxer_cq_busy <= 0;

							if(tlp_demuxer_cq_s_axis_ap_tlp_hdr_tlast[TLP_HDR_COUNT-1]) begin
								ap_state <= AP_STATE_TLP_HDR;
								tlp_hdr_idx <= 0;
							end
						end
					end else begin
						if(s_axis_rx_fire) begin
							if(s_axis_rx_tlast) begin
								ap_state <= AP_STATE_TLP_HDR;
								tlp_hdr_idx <= 0;
							end
						end
					end
				end

				AP_STATE_DEMUX_RC_DESC: begin
					if(tlp_demuxer_rc_desc_busy) begin
						if(tlp_demuxer_rc_desc_s_axis_ap_fire) begin
							tlp_demuxer_rc_desc_busy <= 0;

							if(tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tlast[TLP_HDR_COUNT-1]) begin
								ap_state <= AP_STATE_TLP_HDR;
								tlp_hdr_idx <= 0;
							end
						end
					end else begin
						if(s_axis_rx_fire) begin
							if(s_axis_rx_tlast) begin
								ap_state <= AP_STATE_TLP_HDR;
								tlp_hdr_idx <= 0;
							end
						end
					end
				end

				AP_STATE_DEMUX_RC_GRP: begin
					if(tlp_demuxer_rc_grp_busy) begin
						if(tlp_demuxer_rc_grp_s_axis_ap_fire) begin
							tlp_demuxer_rc_grp_busy <= 0;

							if(tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tlast[TLP_HDR_COUNT-1]) begin
								ap_state <= AP_STATE_TLP_HDR;
								tlp_hdr_idx <= 0;
							end
						end
					end else begin
						if(s_axis_rx_fire) begin
							if(s_axis_rx_tlast) begin
								ap_state <= AP_STATE_TLP_HDR;
								tlp_hdr_idx <= 0;
							end
						end
					end
				end

				AP_STATE_DEMUX_ERR: begin
					if(s_axis_rx_fire) begin
						if(s_axis_rx_tlast) begin
							ap_state <= AP_STATE_TLP_HDR;
							tlp_hdr_idx <= 0;
						end
					end
				end
			endcase
		end
	end

	// @COMB s_axis_rx_tready
	always @(*) begin
		s_axis_rx_tready = 0;

		case(ap_state)
			AP_STATE_TLP_HDR: begin
				s_axis_rx_tready = 1;
			end

			AP_STATE_DEMUX_CQ: begin
				if(! tlp_demuxer_cq_busy) begin
					s_axis_rx_tready = tlp_demuxer_cq_s_axis_rx_tready;
				end
			end

			AP_STATE_DEMUX_RC_DESC: begin
				if(! tlp_demuxer_rc_desc_busy) begin
					s_axis_rx_tready = tlp_demuxer_rc_desc_s_axis_rx_tready;
				end
			end

			AP_STATE_DEMUX_RC_GRP: begin
				if(! tlp_demuxer_rc_grp_busy) begin
					s_axis_rx_tready = tlp_demuxer_rc_grp_s_axis_rx_tready;
				end
			end

			AP_STATE_DEMUX_ERR: begin
				s_axis_rx_tready = 1;
			end
		endcase
	end

	assign s_axis_rx_fire = s_axis_rx_tready && s_axis_rx_tvalid;

	// @COMB tlp_demuxer_cq_s_axis_ap_tlp_hdr_tdata, @COMB tlp_demuxer_cq_s_axis_ap_tlp_hdr_tkeep,
	// @COMB tlp_demuxer_cq_s_axis_ap_tlp_hdr_tlast, @COMB tlp_demuxer_cq_s_axis_ap_tvalid
	always @(*) begin
		tlp_demuxer_cq_s_axis_ap_tlp_hdr_tdata = 'hCAFE0003;
		tlp_demuxer_cq_s_axis_ap_tlp_hdr_tkeep = 0;
		tlp_demuxer_cq_s_axis_ap_tlp_hdr_tlast = 0;
		tlp_demuxer_cq_s_axis_ap_tvalid = 0;

		case(ap_state)
			AP_STATE_DEMUX_CQ: begin
				if(tlp_demuxer_cq_busy) begin
					tlp_demuxer_cq_s_axis_ap_tlp_hdr_tdata = {
						tlp_hdr_tdata[1], tlp_hdr_tdata[0]
					};
					tlp_demuxer_cq_s_axis_ap_tlp_hdr_tkeep = {
						tlp_hdr_tkeep[1], tlp_hdr_tkeep[0]
					};
					tlp_demuxer_cq_s_axis_ap_tlp_hdr_tlast = {
						tlp_hdr_tlast[1], tlp_hdr_tlast[0]
					};
					tlp_demuxer_cq_s_axis_ap_tvalid = 1;
				end
			end
		endcase
	end

	// @COMB tlp_demuxer_cq_s_axis_rx_tdata, @COMB tlp_demuxer_cq_s_axis_rx_tkeep,
	// @COMB tlp_demuxer_cq_s_axis_rx_tlast, @COMB tlp_demuxer_cq_s_axis_rx_tvalid,
	// @COMB tlp_demuxer_cq_s_axis_rx_tuser
	always @(*) begin
		tlp_demuxer_cq_s_axis_rx_tdata = 'hCAFE0004;
		tlp_demuxer_cq_s_axis_rx_tkeep = 0;
		tlp_demuxer_cq_s_axis_rx_tlast = 0;
		tlp_demuxer_cq_s_axis_rx_tvalid = 0;
		tlp_demuxer_cq_s_axis_rx_tuser = 0;

		case(ap_state)
			AP_STATE_DEMUX_CQ: begin
				if(! tlp_demuxer_cq_busy) begin
					tlp_demuxer_cq_s_axis_rx_tdata = s_axis_rx_tdata;
					tlp_demuxer_cq_s_axis_rx_tkeep = s_axis_rx_tkeep;
					tlp_demuxer_cq_s_axis_rx_tlast = s_axis_rx_tlast;
					tlp_demuxer_cq_s_axis_rx_tvalid = s_axis_rx_tvalid;
					tlp_demuxer_cq_s_axis_rx_tuser = s_axis_rx_tuser;
				end
			end
		endcase
	end

	tlp_demuxer_cq #(
		.C_DATA_WIDTH(C_DATA_WIDTH),
		.C_USER_WIDTH(C_USER_WIDTH),
		.C_TLP_HDR_COUNT(TLP_HDR_COUNT)
	) tlp_demuxer_cq_U (
		.clk(clk),
		.rst_n(rst_n),

		.s_axis_rx_tdata(tlp_demuxer_cq_s_axis_rx_tdata),
		.s_axis_rx_tkeep(tlp_demuxer_cq_s_axis_rx_tkeep),
		.s_axis_rx_tlast(tlp_demuxer_cq_s_axis_rx_tlast),
		.s_axis_rx_tvalid(tlp_demuxer_cq_s_axis_rx_tvalid),
		.s_axis_rx_tready(tlp_demuxer_cq_s_axis_rx_tready),
		.s_axis_rx_tuser(tlp_demuxer_cq_s_axis_rx_tuser),

		.m_axis_cq_tdata(m_axis_cq_tdata),
		.m_axis_cq_tkeep(m_axis_cq_tkeep),
		.m_axis_cq_tlast(m_axis_cq_tlast),
		.m_axis_cq_tvalid(m_axis_cq_tvalid),
		.m_axis_cq_tready(m_axis_cq_tready),

		.s_axis_ap_tlp_hdr_tdata(tlp_demuxer_cq_s_axis_ap_tlp_hdr_tdata),
		.s_axis_ap_tlp_hdr_tkeep(tlp_demuxer_cq_s_axis_ap_tlp_hdr_tkeep),
		.s_axis_ap_tlp_hdr_tlast(tlp_demuxer_cq_s_axis_ap_tlp_hdr_tlast),
		.s_axis_ap_tvalid(tlp_demuxer_cq_s_axis_ap_tvalid),
		.s_axis_ap_tready(tlp_demuxer_cq_s_axis_ap_tready)
	);

	assign tlp_demuxer_cq_s_axis_rx_fire = tlp_demuxer_cq_s_axis_rx_tready && tlp_demuxer_cq_s_axis_rx_tvalid;
	assign tlp_demuxer_cq_s_axis_ap_fire = tlp_demuxer_cq_s_axis_ap_tready && tlp_demuxer_cq_s_axis_ap_tvalid;

	assign m00_axis_rc_desc_tdata = m_axis_rc_desc_tdata[0];
	assign m00_axis_rc_desc_tkeep = m_axis_rc_desc_tkeep[0];
	assign m00_axis_rc_desc_tlast = m_axis_rc_desc_tlast[0];
	assign m00_axis_rc_desc_tvalid = m_axis_rc_desc_tvalid[0];
	assign m_axis_rc_desc_tready[0] = m00_axis_rc_desc_tready;

	assign m01_axis_rc_desc_tdata = m_axis_rc_desc_tdata[1];
	assign m01_axis_rc_desc_tkeep = m_axis_rc_desc_tkeep[1];
	assign m01_axis_rc_desc_tlast = m_axis_rc_desc_tlast[1];
	assign m01_axis_rc_desc_tvalid = m_axis_rc_desc_tvalid[1];
	assign m_axis_rc_desc_tready[1] = m01_axis_rc_desc_tready;

	// @COMB tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tdata, @COMB tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tkeep,
	// @COMB tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tlast, @COMB tlp_demuxer_rc_desc_s_axis_ap_tvalid
	always @(*) begin
		tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tdata = 'hCAFE0003;
		tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tkeep = 0;
		tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tlast = 0;
		tlp_demuxer_rc_desc_s_axis_ap_tvalid = 0;

		case(ap_state)
			AP_STATE_DEMUX_RC_DESC: begin
				if(tlp_demuxer_rc_desc_busy) begin
					tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tdata = {
						tlp_hdr_tdata[1], tlp_hdr_tdata[0]
					};
					tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tkeep = {
						tlp_hdr_tkeep[1], tlp_hdr_tkeep[0]
					};
					tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tlast = {
						tlp_hdr_tlast[1], tlp_hdr_tlast[0]
					};
					tlp_demuxer_rc_desc_s_axis_ap_tvalid = 1;
				end
			end
		endcase
	end

	// @COMB tlp_demuxer_rc_desc_s_axis_rx_tdata, @COMB tlp_demuxer_rc_desc_s_axis_rx_tkeep,
	// @COMB tlp_demuxer_rc_desc_s_axis_rx_tlast, @COMB tlp_demuxer_rc_desc_s_axis_rx_tvalid,
	// @COMB tlp_demuxer_rc_desc_s_axis_rx_tuser
	always @(*) begin
		tlp_demuxer_rc_desc_s_axis_rx_tdata = 'hCAFE0004;
		tlp_demuxer_rc_desc_s_axis_rx_tkeep = 0;
		tlp_demuxer_rc_desc_s_axis_rx_tlast = 0;
		tlp_demuxer_rc_desc_s_axis_rx_tvalid = 0;
		tlp_demuxer_rc_desc_s_axis_rx_tuser = 0;

		case(ap_state)
			AP_STATE_DEMUX_RC_DESC: begin
				if(! tlp_demuxer_rc_desc_busy) begin
					tlp_demuxer_rc_desc_s_axis_rx_tdata = s_axis_rx_tdata;
					tlp_demuxer_rc_desc_s_axis_rx_tkeep = s_axis_rx_tkeep;
					tlp_demuxer_rc_desc_s_axis_rx_tlast = s_axis_rx_tlast;
					tlp_demuxer_rc_desc_s_axis_rx_tvalid = s_axis_rx_tvalid;
					tlp_demuxer_rc_desc_s_axis_rx_tuser = s_axis_rx_tuser;
				end
			end
		endcase
	end

	tlp_demuxer_rc_desc #(
		.C_DATA_WIDTH(C_DATA_WIDTH),
		.C_USER_WIDTH(C_USER_WIDTH),
		.C_TLP_HDR_COUNT(TLP_HDR_COUNT),
		.C_RC_DESC_COUNT(RC_DESC_COUNT)
	) tlp_demuxer_rc_desc_U (
		.clk(clk),
		.rst_n(rst_n),

		.s_axis_rx_tdata(tlp_demuxer_rc_desc_s_axis_rx_tdata),
		.s_axis_rx_tkeep(tlp_demuxer_rc_desc_s_axis_rx_tkeep),
		.s_axis_rx_tlast(tlp_demuxer_rc_desc_s_axis_rx_tlast),
		.s_axis_rx_tvalid(tlp_demuxer_rc_desc_s_axis_rx_tvalid),
		.s_axis_rx_tready(tlp_demuxer_rc_desc_s_axis_rx_tready),
		.s_axis_rx_tuser(tlp_demuxer_rc_desc_s_axis_rx_tuser),

		.m_axis_rc_desc_tdata({m_axis_rc_desc_tdata[1], m_axis_rc_desc_tdata[0]}),
		.m_axis_rc_desc_tkeep({m_axis_rc_desc_tkeep[1], m_axis_rc_desc_tkeep[0]}),
		.m_axis_rc_desc_tlast({m_axis_rc_desc_tlast[1], m_axis_rc_desc_tlast[0]}),
		.m_axis_rc_desc_tvalid({m_axis_rc_desc_tvalid[1], m_axis_rc_desc_tvalid[0]}),
		.m_axis_rc_desc_tready({m_axis_rc_desc_tready[1], m_axis_rc_desc_tready[0]}),

		.s_axis_ap_tlp_hdr_tdata(tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tdata),
		.s_axis_ap_tlp_hdr_tkeep(tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tkeep),
		.s_axis_ap_tlp_hdr_tlast(tlp_demuxer_rc_desc_s_axis_ap_tlp_hdr_tlast),
		.s_axis_ap_tvalid(tlp_demuxer_rc_desc_s_axis_ap_tvalid),
		.s_axis_ap_tready(tlp_demuxer_rc_desc_s_axis_ap_tready)
	);

	assign tlp_demuxer_rc_desc_s_axis_rx_fire = tlp_demuxer_rc_desc_s_axis_rx_tready && tlp_demuxer_rc_desc_s_axis_rx_tvalid;
	assign tlp_demuxer_rc_desc_s_axis_ap_fire = tlp_demuxer_rc_desc_s_axis_ap_tready && tlp_demuxer_rc_desc_s_axis_ap_tvalid;

	assign m00_axis_rc_grp_tdata = m_axis_rc_grp_tdata[0];
	assign m00_axis_rc_grp_tkeep = m_axis_rc_grp_tkeep[0];
	assign m00_axis_rc_grp_tlast = m_axis_rc_grp_tlast[0];
	assign m00_axis_rc_grp_tvalid = m_axis_rc_grp_tvalid[0];
	assign m_axis_rc_grp_tready[0] = m00_axis_rc_grp_tready;

	assign m01_axis_rc_grp_tdata = m_axis_rc_grp_tdata[1];
	assign m01_axis_rc_grp_tkeep = m_axis_rc_grp_tkeep[1];
	assign m01_axis_rc_grp_tlast = m_axis_rc_grp_tlast[1];
	assign m01_axis_rc_grp_tvalid = m_axis_rc_grp_tvalid[1];
	assign m_axis_rc_grp_tready[1] = m01_axis_rc_grp_tready;

	// @COMB tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tdata, @COMB tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tkeep,
	// @COMB tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tlast, @COMB tlp_demuxer_rc_grp_s_axis_ap_tvalid
	always @(*) begin
		tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tdata = 'hCAFE0005;
		tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tkeep = 0;
		tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tlast = 0;
		tlp_demuxer_rc_grp_s_axis_ap_tvalid = 0;

		case(ap_state)
			AP_STATE_DEMUX_RC_GRP: begin
				if(tlp_demuxer_rc_grp_busy) begin
					tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tdata = {
						tlp_hdr_tdata[1], tlp_hdr_tdata[0]
					};
					tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tkeep = {
						tlp_hdr_tkeep[1], tlp_hdr_tkeep[0]
					};
					tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tlast = {
						tlp_hdr_tlast[1], tlp_hdr_tlast[0]
					};
					tlp_demuxer_rc_grp_s_axis_ap_tvalid = 1;
				end
			end
		endcase
	end

	// @COMB tlp_demuxer_rc_grp_s_axis_rx_tdata, @COMB tlp_demuxer_rc_grp_s_axis_rx_tkeep,
	// @COMB tlp_demuxer_rc_grp_s_axis_rx_tlast, @COMB tlp_demuxer_rc_grp_s_axis_rx_tvalid,
	// @COMB tlp_demuxer_rc_grp_s_axis_rx_tuser
	always @(*) begin
		tlp_demuxer_rc_grp_s_axis_rx_tdata = 'hCAFE0006;
		tlp_demuxer_rc_grp_s_axis_rx_tkeep = 0;
		tlp_demuxer_rc_grp_s_axis_rx_tlast = 0;
		tlp_demuxer_rc_grp_s_axis_rx_tvalid = 0;
		tlp_demuxer_rc_grp_s_axis_rx_tuser = 0;

		case(ap_state)
			AP_STATE_DEMUX_RC_GRP: begin
				if(! tlp_demuxer_rc_grp_busy) begin
					tlp_demuxer_rc_grp_s_axis_rx_tdata = s_axis_rx_tdata;
					tlp_demuxer_rc_grp_s_axis_rx_tkeep = s_axis_rx_tkeep;
					tlp_demuxer_rc_grp_s_axis_rx_tlast = s_axis_rx_tlast;
					tlp_demuxer_rc_grp_s_axis_rx_tvalid = s_axis_rx_tvalid;
					tlp_demuxer_rc_grp_s_axis_rx_tuser = s_axis_rx_tuser;
				end
			end
		endcase
	end

	tlp_demuxer_rc_grp #(
		.C_DATA_WIDTH(C_DATA_WIDTH),
		.C_USER_WIDTH(C_USER_WIDTH),
		.C_TLP_HDR_COUNT(TLP_HDR_COUNT),
		.C_RC_GRP_COUNT(RC_GRP_COUNT)
	) tlp_demuxer_rc_grp_U (
		.clk(clk),
		.rst_n(rst_n),

		.s_axis_rx_tdata(tlp_demuxer_rc_grp_s_axis_rx_tdata),
		.s_axis_rx_tkeep(tlp_demuxer_rc_grp_s_axis_rx_tkeep),
		.s_axis_rx_tlast(tlp_demuxer_rc_grp_s_axis_rx_tlast),
		.s_axis_rx_tvalid(tlp_demuxer_rc_grp_s_axis_rx_tvalid),
		.s_axis_rx_tready(tlp_demuxer_rc_grp_s_axis_rx_tready),
		.s_axis_rx_tuser(tlp_demuxer_rc_grp_s_axis_rx_tuser),

		.m_axis_rc_grp_tdata({m_axis_rc_grp_tdata[1], m_axis_rc_grp_tdata[0]}),
		.m_axis_rc_grp_tkeep({m_axis_rc_grp_tkeep[1], m_axis_rc_grp_tkeep[0]}),
		.m_axis_rc_grp_tlast({m_axis_rc_grp_tlast[1], m_axis_rc_grp_tlast[0]}),
		.m_axis_rc_grp_tvalid({m_axis_rc_grp_tvalid[1], m_axis_rc_grp_tvalid[0]}),
		.m_axis_rc_grp_tready({m_axis_rc_grp_tready[1], m_axis_rc_grp_tready[0]}),

		.s_axis_ap_tlp_hdr_tdata(tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tdata),
		.s_axis_ap_tlp_hdr_tkeep(tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tkeep),
		.s_axis_ap_tlp_hdr_tlast(tlp_demuxer_rc_grp_s_axis_ap_tlp_hdr_tlast),
		.s_axis_ap_tvalid(tlp_demuxer_rc_grp_s_axis_ap_tvalid),
		.s_axis_ap_tready(tlp_demuxer_rc_grp_s_axis_ap_tready)
	);

	assign tlp_demuxer_rc_grp_s_axis_rx_fire = tlp_demuxer_rc_grp_s_axis_rx_tready && tlp_demuxer_rc_grp_s_axis_rx_tvalid;
	assign tlp_demuxer_rc_grp_s_axis_ap_fire = tlp_demuxer_rc_grp_s_axis_ap_tready && tlp_demuxer_rc_grp_s_axis_ap_tvalid;
endmodule
