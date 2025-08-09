# Project Overview

This project is a Xilinx Vivado project for a PCIe (PCI Express) endpoint device. The project is implemented in Verilog and targets a Xilinx Artix-7 FPGA (xc7a50tcsg325-2).

The main functionalities of the project include:
- TLP (Transaction Layer Packet) muxing and demuxing
- TLP to AXI-lite bridging
- TLP DMA (Direct Memory Access) write operations
- MSI-X (Message Signaled Interrupts) handling

# Building and Running

This is a Vivado project. To work with this project, you will need to have Xilinx Vivado Design Suite installed.

1.  **Open the project:**
    Open the project file `pcie_7x_0_ex.xpr` in Vivado.

2.  **Build the project:**
    In Vivado, run the synthesis and implementation steps. This will generate the bitstream file that can be programmed onto the target FPGA.

3.  **Run the simulation:**
    The project includes a testbench (`board.v`). You can run the simulation in Vivado to verify the functionality of the design.

# Development Conventions

- The project follows the standard Vivado project structure.
- Verilog is used as the hardware description language.
- The project uses Xilinx IP cores for PCIe and other functionalities.
- The top-level module for the design is `xilinx_pcie_2_1_ep_7x`.
- The simulation top-level module is `board`.
- Constraints are defined in the `xilinx_pcie_7x_ep_x4g2.xdc` file.
