`timescale 1ns/1ps
//==============================================================================
// dma_read_engine.v
//
// AXI4-lite READ master. Issues AR for each source word, waits for the
// slave's ARREADY, then waits for RVALID, then pushes the word into the
// FIFO. Only ONE read transaction is ever outstanding on this channel
// (the provided axi4_ram_slave BFM cannot buffer a 2nd ARADDR before the
// 1st RDATA is consumed), so this engine is a simple 3-state loop.
//
// `reverse` (see dma.v) makes this engine walk words from word_count-1
// down to 0 instead of 0 up to word_count-1. This is how a close dst>src
// overlap is made safe (classic memmove-style backward copy) - see
// dma.v for the full reasoning. It runs independently of the write
// engine; the FIFO is what lets read and write proceed concurrently.
//==============================================================================
module dma_read_engine (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         start,      // pulse: (re)start counting from word 0
    input  wire         busy,       // 1 while a transfer is in progress
    input  wire [31:0]  src_base,
    input  wire [15:0]  word_count,
    input  wire         reverse,    // 1: process words word_count-1 .. 0

    // AXI4-lite read address channel
    output reg           axi_arvalid,
    output reg  [31:0]   axi_araddr,
    input  wire          axi_arready,

    // AXI4-lite read data channel
    input  wire          axi_rvalid,
    input  wire [31:0]   axi_rdata,
    output reg            axi_rready,

    // FIFO push side
    output reg           fifo_push,
    output reg  [31:0]   fifo_push_data,
    input  wire          fifo_full
);

    localparam IDLE = 2'd0, ADDR = 2'd1, DATA = 2'd2;
    reg [1:0]  state;
    reg [15:0] rd_cnt;   // words processed so far (0..word_count-1, in processing order)

    wire has_work = busy && (rd_cnt < word_count);
    wire [15:0] word_idx = reverse ? (word_count - 16'd1 - rd_cnt) : rd_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= IDLE;
            rd_cnt         <= 16'd0;
            axi_arvalid    <= 1'b0;
            axi_araddr     <= 32'd0;
            axi_rready     <= 1'b0;
            fifo_push      <= 1'b0;
            fifo_push_data <= 32'd0;
        end else begin
            fifo_push <= 1'b0; // default: 1-cycle pulse

            case (state)
                IDLE: begin
                    if (has_work && !fifo_full) begin
                        axi_araddr  <= src_base + (word_idx << 2);
                        axi_arvalid <= 1'b1;
                        state       <= ADDR;
                    end
                end

                ADDR: begin
                    if (axi_arvalid && axi_arready) begin
                        axi_arvalid <= 1'b0;
                        axi_rready  <= 1'b1;
                        state       <= DATA;
                    end
                end

                DATA: begin
                    if (axi_rready && axi_rvalid) begin
                        fifo_push      <= 1'b1;
                        fifo_push_data <= axi_rdata;
                        axi_rready     <= 1'b0;
                        rd_cnt         <= rd_cnt + 16'd1;
                        state          <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase

            if (start) begin
                state       <= IDLE;
                rd_cnt      <= 16'd0;
                axi_arvalid <= 1'b0;
                axi_rready  <= 1'b0;
                fifo_push   <= 1'b0;
            end
        end
    end

endmodule
