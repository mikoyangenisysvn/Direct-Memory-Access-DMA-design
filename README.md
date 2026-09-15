# Lab — DMA Controller (APB + AXI4-Lite)

## Overview
This project implements a DMA (Direct Memory Access) controller that moves a block of data from one memory address to another without CPU intervention on every transaction. The design is written in Verilog and exposes two interfaces: an APB slave interface (used by a host/CPU to configure and control the DMA) and an AXI4-Lite master interface (used to read/write memory-mapped regions).

## Objectives
- Design `dma.v` according to the specification (APB slave CSR interface + AXI4-Lite master interface).
- Build a test plan and a top-level testbench with tasks to trigger DMA commands through `apb_master`.
- Write and run tests until all pass, while meeting the performance target.

## Architecture
`dma.v` is decomposed into smaller submodules, reusing patterns from earlier coursework (`apb_slave`, `axi4lite_command`, `counter`):
- `dma_csr` — holds the Control/Status/Source/Destination registers and handles the APB slave protocol.
- `dma_ctrl` — top-level FSM controlling the overall transfer sequence.
- `dma_axi_rd` — AXI4-Lite master responsible for reading data from the source address.
- `dma_axi_wr` — AXI4-Lite master responsible for writing data to the destination address.
- `dma_fifo` — buffer between the read and write paths, allowing reads and writes to proceed in parallel to maximize throughput.

### Control and Status Registers (CSR)
| Name | Address | R/W | Description |
|---|---|---|---|
| Control | 0x00 | R/W | Bit 0: start; Bits 31:16: transfer size (up to 64KB) |
| Status | 0x04 | RO | Bit 0: busy; Bit 1: done |
| Source address | 0x08 | R/W | Source address (4-byte aligned) |
| Destination address | 0x0C | R/W | Destination address (4-byte aligned) |

### Theory of Operation
1. The CPU writes the source and destination addresses to 0x08 and 0x0C.
2. The CPU writes the transfer size (bits 31:16) and sets the start bit (bit 0) in the Control register (0x00).
3. The DMA performs paired AXI read (from source) and AXI write (to destination) transactions, incrementing an internal counter after each pair until the requested size is reached.
4. The CPU polls the Status register: busy=1 means the transfer is still in progress; busy=0 & done=1 means the transfer is complete and the destination memory can be used.
5. Starting a new transfer while busy is still set results in undefined behavior.

## Edge Cases Handled
- Fully overlapping source and destination (same address): must not corrupt data, requires correct read/write ordering (similar to `memmove`).
- Partially overlapping regions: when destination > source, the transfer must proceed backward (from the end toward the start) to avoid overwriting data not yet read.
- Zero-size transfer.
- Transfer size close to the 64KB maximum.
- Repeated restart attempts while still busy (robustness check).

## Performance Requirement
A design that only moves data correctly, without overlapping read and write transactions, receives only half credit. The full requirement is for AXI read and write transactions to proceed in parallel, with throughput limited only by the slower of the two (since the slave can insert delays in any phase of a transaction).

## Verification Environment
- Top-level testbench (`dma_top_tb.v`) uses the provided `apb_master.v` (driver) and `axi4_ram_slave.v` (BFM).
- Two testbench sets were used:
  1. **Baseline/reference testbench** (2 test cases) — the design must pass this before further test development.
  2. **Extended/hardened testbench** (14 test cases) — covers overlap in both directions, size=0, near-64KB sizes, restart-while-busy robustness, status polling, and performance checks.

## Current Status
- All 14 functional test cases pass, after debugging: stale APB read timing, a deadlock on zero-size transfers, and data corruption on forward-overlapping transfers (fixed by traversing backward, memmove-style).
- Minor performance-threshold warnings remain for small transfers (≤16 words).

## Source Files
```
dma.v            — top-level DMA controller
dma_csr.v        — CSR block / APB slave logic
dma_ctrl.v       — control FSM
dma_axi_rd.v     — AXI4-Lite read master
dma_axi_wr.v     — AXI4-Lite write master
dma_fifo.v       — read/write buffer
apb_master.v     — provided APB driver (not authored in this project)
axi4_ram_slave.v — provided AXI4-Lite RAM BFM (not authored in this project)
dma_top_tb.v     — baseline/reference testbench
dma_top_tb_ext.v — extended testbench (14 test cases)
```

