module axis_fifo #(
	parameter DATA_WIDTH = 32,  // Width of the data bus
	parameter FIFO_DEPTH = 16,  // Depth of the FIFO (must be power of 2)
	parameter KEEP_WIDTH = DATA_WIDTH/8 // Width of TKEEP (1 bit per byte)
) (
	// Clock and reset
	input  wire                  clk,
	input  wire                  rst_n,

	// AXI Stream Slave Interface (Input)
	input  wire [DATA_WIDTH-1:0] s_axis_tdata,
	input  wire [KEEP_WIDTH-1:0] s_axis_tkeep,
	input  wire                  s_axis_tvalid,
	output reg                   s_axis_tready,
	input  wire                  s_axis_tlast,

	// AXI Stream Master Interface (Output)
	output reg  [DATA_WIDTH-1:0] m_axis_tdata,
	output reg  [KEEP_WIDTH-1:0] m_axis_tkeep,
	output reg                   m_axis_tvalid,
	input  wire                  m_axis_tready,
	output reg                   m_axis_tlast,

	// Status signals
	output reg                   fifo_full,
	output reg                   fifo_empty
);

// Local parameters
localparam ADDR_WIDTH = $clog2(FIFO_DEPTH);

// Internal registers
reg [DATA_WIDTH-1:0] mem_tdata [0:FIFO_DEPTH-1]; // FIFO memory for TDATA
reg [KEEP_WIDTH-1:0] mem_tkeep [0:FIFO_DEPTH-1]; // FIFO memory for TKEEP
reg                  mem_tlast [0:FIFO_DEPTH-1]; // FIFO memory for TLAST
reg [ADDR_WIDTH:0]   wr_ptr;                    // Write pointer (extra bit for wrap-around)
reg [ADDR_WIDTH:0]   rd_ptr;                    // Read pointer (extra bit for wrap-around)

// Internal signals
wire                 wr_en;
wire                 rd_en;
wire                 full;
wire                 empty;

// Assign status signals
assign full  = (wr_ptr[ADDR_WIDTH-1:0] == rd_ptr[ADDR_WIDTH-1:0]) && (wr_ptr[ADDR_WIDTH] != rd_ptr[ADDR_WIDTH]);
assign empty = (wr_ptr == rd_ptr);

// Write and read enable conditions
assign wr_en = s_axis_tvalid && s_axis_tready && !full;
assign rd_en = m_axis_tready && m_axis_tvalid && !empty;

// Write logic
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		wr_ptr <= 0;
		s_axis_tready <= 1'b1;
	end
	else begin
		if (wr_en) begin
			mem_tdata[wr_ptr[ADDR_WIDTH-1:0]] <= s_axis_tdata;
			mem_tkeep[wr_ptr[ADDR_WIDTH-1:0]] <= s_axis_tkeep;
			mem_tlast[wr_ptr[ADDR_WIDTH-1:0]] <= s_axis_tlast;
			wr_ptr <= wr_ptr + 1;
		end
		s_axis_tready <= !full; // Ready to accept data unless FIFO is full
	end
end

// Read logic
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		rd_ptr <= 0;
		m_axis_tvalid <= 1'b0;
		m_axis_tdata <= {DATA_WIDTH{1'b0}};
		m_axis_tkeep <= {KEEP_WIDTH{1'b0}};
		m_axis_tlast <= 1'b0;
	end
	else begin
		if (rd_en) begin
			m_axis_tdata <= mem_tdata[rd_ptr[ADDR_WIDTH-1:0]];
			m_axis_tkeep <= mem_tkeep[rd_ptr[ADDR_WIDTH-1:0]];
			m_axis_tlast <= mem_tlast[rd_ptr[ADDR_WIDTH-1:0]];
			rd_ptr <= rd_ptr + 1;
		end
		m_axis_tvalid <= !empty; // Valid data available unless FIFO is empty
	end
end

// Status signals
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		fifo_full <= 1'b0;
		fifo_empty <= 1'b1;
	end
	else begin
		fifo_full <= full;
		fifo_empty <= empty;
	end
end

endmodule