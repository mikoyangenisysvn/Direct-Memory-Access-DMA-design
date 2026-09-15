`timescale 1ns/1ps
//==============================================================================
// dma_apb_csr.v
//
// APB slave register file for the DMA controller.
// - Decodes psel/penable/pwrite/paddr, no wait states (pready = psel&penable).
// - Owns storage for Source (0x08) and Destination (0x0C) registers.
// - Control (0x00) and Status (0x04) are NOT stored here: Control is only
//   a "command" (start bit + size), consumed as a pulse by the top level;
//   Status is built on the fly from busy/done/xfer_size passed in from the
//   datapath, so this module never gets out of sync with real state.
//==============================================================================
module dma_apb_csr (
    input  wire         clk,
    input  wire         rst_n,

    // APB slave
    input  wire         psel,
    input  wire         penable,
    input  wire         pwrite,
    input  wire [7:0]   paddr,
    input  wire [31:0]  pwdata,
    output reg  [31:0]  prdata,
    output wire         pready,

    // live status from the datapath (for read-back only)
    input  wire         busy,
    input  wire         done,
    input  wire [15:0]  xfer_size,

    // stored registers, exposed to the top level
    output reg  [31:0]  src_addr,
    output reg  [31:0]  dst_addr,

    // "start" command decode
    output wire         start_pulse,   // 1-cycle pulse: control reg written with bit0=1 while idle
    output wire [15:0]  start_size     // size field from that same control write
);

    assign pready = psel & penable;

    wire apb_wr = psel & penable & pwrite & pready;

    assign start_pulse = apb_wr && (paddr == 8'h00) && pwdata[0] && !busy;
    assign start_size  = pwdata[31:16];

    // ---- Source / Destination address registers ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            src_addr <= 32'h0;
            dst_addr <= 32'h0;
        end else if (apb_wr) begin
            case (paddr)
                8'h08: src_addr <= pwdata;
                8'h0C: dst_addr <= pwdata;
                default: ;
            endcase
        end
    end

    // ---- Read-back mux ----
    always @(*) begin
        case (paddr)
            8'h00:   prdata = {xfer_size, 15'b0, busy}; // control read-back (best-effort)
            8'h04:   prdata = {30'b0, done, busy};       // status
            8'h08:   prdata = src_addr;
            8'h0C:   prdata = dst_addr;
            default: prdata = 32'h0;
        endcase
    end

endmodule
