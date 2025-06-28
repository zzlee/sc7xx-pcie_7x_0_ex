`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/16/2025 05:15:07 PM
// Design Name: 
// Module Name: fifo_fwft
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


/*
 * Parametrized FIFO with Zero-Delay Read (First-Word Fall-Through - FWFT)
 */
module fifo_fwft #(
	parameter DATA_WIDTH = 8,       // Width of data bus
	parameter DEPTH      = 16      // Number of entries in FIFO
) (
	// System Signals
	input wire                  clk,      // Clock
	input wire                  rst_n,    // Asynchronous Reset, active low

	// Write Interface
	input wire                  wr_en,    // Write Enable
	input wire [DATA_WIDTH-1:0] wr_data,  // Data to write

	// Read Interface
	input wire                  rd_en,    // Read Enable (consumes data, advances pointer)
	output wire [DATA_WIDTH-1:0] rd_data, // Data being read (valid when data_valid is high)
	output wire                 data_valid,// Indicates rd_data holds valid data (FIFO not empty)

	// Status Signals
	output wire                 full,     // FIFO is full
	output wire                 empty     // FIFO is empty
);

	// Calculate address width - need enough bits to address DEPTH locations
	localparam ADDR_WIDTH = $clog2(DEPTH);

	// Use pointers with an extra bit to distinguish full/empty conditions easily
	localparam PTR_WIDTH = ADDR_WIDTH + 1;

	// Internal FIFO storage
	reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

	// Read and Write pointers
	reg [PTR_WIDTH-1:0] wr_ptr;
	reg [PTR_WIDTH-1:0] rd_ptr;

	// Internal signals for pointer comparison
	wire [ADDR_WIDTH-1:0] wr_addr = wr_ptr[ADDR_WIDTH-1:0]; // Address bits of write pointer
	wire [ADDR_WIDTH-1:0] rd_addr = rd_ptr[ADDR_WIDTH-1:0]; // Address bits of read pointer

	// --- Status Logic ---
	// Empty condition: read and write pointers are identical
	assign empty = (wr_ptr == rd_ptr);

	// Full condition: pointers have same address bits but different MSB (wrap bit)
	assign full = (wr_addr == rd_addr) && (wr_ptr[ADDR_WIDTH] != rd_ptr[ADDR_WIDTH]);

	// Data Valid signal for FWFT: Data is valid if the FIFO is not empty
	assign data_valid = !empty;

	// --- Write Logic ---
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			wr_ptr <= {PTR_WIDTH{1'b0}}; // Reset write pointer
		end else begin
			// Only write if enabled and FIFO is not full
			if (wr_en && !full) begin
				mem[wr_addr] <= wr_data;     // Write data to memory at current write address
				wr_ptr <= wr_ptr + 1'b1;     // Increment write pointer (handles wrap-around)
			end
		end
	end

	// --- Read Logic ---
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rd_ptr <= {PTR_WIDTH{1'b0}}; // Reset read pointer
		end else begin
			// Only advance read pointer if enabled by user and FIFO is not empty
			if (rd_en && !empty) begin
				rd_ptr <= rd_ptr + 1'b1;     // Increment read pointer (handles wrap-around)
			end
		end
	end

	// --- Read Data Path (Combinational for FWFT) ---
	// Output the data pointed to by the current read address.
	// This assignment is combinational, providing the "zero-delay" read aspect.
	// The data becomes available as soon as rd_ptr points to it (i.e., when !empty).
	assign rd_data = mem[rd_addr];

	// Helper function for calculating ceiling log base 2
	// (Needed if $clog2 is not available in older Verilog versions/tools)
	// function integer clog2 (input integer value);
	//    integer i = 0;
	//    integer temp = value - 1;
	//    begin
	//        while (temp > 0) begin
	//            temp = temp >> 1;
	//            i = i + 1;
	//        end
	//        if (value <= 1) // Handle edge case for DEPTH=1
	//          clog2 = 1;
	//        else
	//          clog2 = i;
	//    end
	// endfunction

endmodule