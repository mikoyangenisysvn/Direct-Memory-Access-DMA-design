`timescale 1ns/1ps
//==============================================================================
// dma.v - DMA Controller (top level)
//
// Wires together:
//   dma_apb_csr     - APB register interface (Control/Status/Src/Dst)
//   dma_fifo        - decouples read engine from write engine
//   dma_read_engine - AXI4-lite read master
//   dma_write_engine- AXI4-lite write master
//
// This module itself only:
//   1) computes word_count / last_wstrb from the latched transfer size
//   2) computes the overlap-safety gating signals (overlap_close/gap_words)
//      passed to the write engine
//   3) latches src_base/dst_base/xfer_size when a transfer starts
//   4) tracks the top-level busy/done handshake seen by the CPU
//
// See dma_read_engine.v / dma_write_engine.v for the read/write pipeline
// itself and for why a close dst>src overlap needs explicit gating.
//==============================================================================

module dma (
    input  wire         clk,
    input  wire         rst_n,

    // ---------------- APB slave interface ----------------
    input  wire         psel,
    input  wire         penable,
    input  wire         pwrite,
    input  wire [7:0]   paddr,
    input  wire [31:0]  pwdata,
    output wire [31:0]  prdata,
    output wire         pready,

    // ---------------- AXI4-lite master: Write Address ----------------
    output wire         axi_awvalid,
    output wire [31:0]  axi_awaddr,
    input  wire         axi_awready,

    // ---------------- AXI4-lite master: Write Data ----------------
    output wire         axi_wvalid,
    output wire [31:0]  axi_wdata,
    output wire [3:0]   axi_wstrb,
    input  wire         axi_wready,

    // ---------------- AXI4-lite master: Write Response ----------------
    input  wire         axi_bvalid,
    output wire         axi_bready,

    // ---------------- AXI4-lite master: Read Address ----------------
    output wire         axi_arvalid,
    output wire [31:0]  axi_araddr,
    input  wire         axi_arready,

    // ---------------- AXI4-lite master: Read Data ----------------
    input  wire         axi_rvalid,
    input  wire [31:0]  axi_rdata,
    output wire         axi_rready
);

    // ==========================================================
    // Top-level busy/done + latched transfer parameters
    // ==========================================================
    reg         busy, done;
    reg  [15:0] xfer_size;
    reg  [31:0] src_base, dst_base;

    wire        start_pulse;
    wire [15:0] start_size;
    wire [31:0] src_addr, dst_addr;   // raw CSR registers (may already point to next transfer)
    wire        xfer_done;            // from write engine: all words written back

    // ceil(size/4), computed with an extra bit so it can't overflow near 64KB
    wire [16:0] word_count_ext = {1'b0, xfer_size} + 17'd3;
    wire [15:0] word_count     = word_count_ext[16:2];

    // byte-enable pattern for the final (possibly partial) word
    wire [1:0] last_byte_rem = xfer_size[1:0];
    wire [3:0] last_wstrb    = (last_byte_rem == 2'd0) ? 4'b1111 :
                                (last_byte_rem == 2'd1) ? 4'b0001 :
                                (last_byte_rem == 2'd2) ? 4'b0011 :
                                                           4'b0111;

    // ==========================================================
    // Overlap protection
    //
    // Addresses are required to be 4-byte aligned (per spec). When dst
    // is close enough to src that the two transferred regions actually
    // overlap (dst_base > src_base and the word distance between them
    // is smaller than word_count), a naive low-to-high copy can have
    // the write side clobber a source word the read side hasn't fetched
    // yet.
    //
    // The fix used here is the classic memmove-style one: walk the
    // words from word_count-1 down to 0 instead of 0 up to word_count-1
    // (both read and write engines walk in the same order, in lockstep,
    // via their own `reverse` input). For dst_base > src_base this
    // guarantees every source word a later write could alias has
    // already been read by the time that write happens - regardless of
    // FIFO depth or how close the overlap is - so no extra throttling
    // is needed and full read/write overlap throughput is preserved.
    //
    // dst_base <= src_base (including dst_base == src_base, in-place
    // copy) is always safe in the normal forward order already, so
    // `reverse` stays 0 there.
    // ==========================================================
    localparam FIFO_DEPTH = 4;

    wire [15:0] gap_words = (dst_base - src_base) >> 2;
    wire        reverse   = (dst_base > src_base) && (gap_words < word_count);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy      <= 1'b0;
            done      <= 1'b0;
            xfer_size <= 16'd0;
            src_base  <= 32'd0;
            dst_base  <= 32'd0;
        end else if (start_pulse) begin
            xfer_size <= start_size;
            src_base  <= src_addr;
            dst_base  <= dst_addr;
            if (start_size == 16'd0) begin
                // nothing to transfer
                busy <= 1'b0;
                done <= 1'b1;
            end else begin
                busy <= 1'b1;
                done <= 1'b0;
            end
        end else if (busy && xfer_done) begin
            busy <= 1'b0;
            done <= 1'b1;
        end
    end

    // ==========================================================
    // FIFO <-> read engine / write engine wiring
    // ==========================================================
    wire        fifo_push, fifo_full, fifo_pop, fifo_empty;
    wire [31:0] fifo_push_data, fifo_pop_data;

    // ==========================================================
    // Submodule instances
    // ==========================================================
    dma_apb_csr u_csr (
        .clk         (clk),
        .rst_n       (rst_n),
        .psel        (psel),
        .penable     (penable),
        .pwrite      (pwrite),
        .paddr       (paddr),
        .pwdata      (pwdata),
        .prdata      (prdata),
        .pready      (pready),
        .busy        (busy),
        .done        (done),
        .xfer_size   (xfer_size),
        .src_addr    (src_addr),
        .dst_addr    (dst_addr),
        .start_pulse (start_pulse),
        .start_size  (start_size)
    );

    dma_fifo #(
        .WIDTH (32),
        .DEPTH (FIFO_DEPTH),
        .AW    (2)
    ) u_fifo (
        .clk       (clk),
        .rst_n     (rst_n),
        .flush     (start_pulse),
        .push      (fifo_push),
        .push_data (fifo_push_data),
        .full      (fifo_full),
        .pop       (fifo_pop),
        .pop_data  (fifo_pop_data),
        .empty     (fifo_empty)
    );

    dma_read_engine u_rd (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start_pulse),
        .busy           (busy),
        .src_base       (src_base),
        .word_count     (word_count),
        .reverse        (reverse),
        .axi_arvalid    (axi_arvalid),
        .axi_araddr     (axi_araddr),
        .axi_arready    (axi_arready),
        .axi_rvalid     (axi_rvalid),
        .axi_rdata      (axi_rdata),
        .axi_rready     (axi_rready),
        .fifo_push      (fifo_push),
        .fifo_push_data (fifo_push_data),
        .fifo_full      (fifo_full)
    );

    dma_write_engine u_wr (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (start_pulse),
        .busy          (busy),
        .dst_base      (dst_base),
        .word_count    (word_count),
        .last_wstrb    (last_wstrb),
        .reverse       (reverse),
        .axi_awvalid   (axi_awvalid),
        .axi_awaddr    (axi_awaddr),
        .axi_awready   (axi_awready),
        .axi_wvalid    (axi_wvalid),
        .axi_wdata     (axi_wdata),
        .axi_wstrb     (axi_wstrb),
        .axi_wready    (axi_wready),
        .axi_bvalid    (axi_bvalid),
        .axi_bready    (axi_bready),
        .fifo_pop      (fifo_pop),
        .fifo_pop_data (fifo_pop_data),
        .fifo_empty    (fifo_empty),
        .xfer_done     (xfer_done)
    );

endmodule
