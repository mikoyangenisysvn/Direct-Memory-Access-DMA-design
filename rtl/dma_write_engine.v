`timescale 1ns/1ps
//==============================================================================
// dma_write_engine.v
//
// AXI4-lite WRITE master. Pops one word from the FIFO, issues AW and W
// concurrently (their handshakes are tracked independently since the slave
// may return AWREADY/WREADY on different cycles), then waits for BVALID.
// Only ONE write transaction is ever outstanding on this channel.
//
// `reverse` (see dma.v) makes this engine walk words from word_count-1
// down to 0 instead of 0 up to word_count-1, in lockstep with the read
// engine's own `reverse` walk (same processing-order index, same
// word_idx mapping) - this is what makes a close dst>src overlap safe;
// see dma.v for the reasoning. Runs independently of the read engine;
// the FIFO (in-order) is what ties the two together correctly.
//==============================================================================
module dma_write_engine (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         start,      // pulse: (re)start counting from word 0
    input  wire         busy,
    input  wire [31:0]  dst_base,
    input  wire [15:0]  word_count,
    input  wire [3:0]   last_wstrb, // byte-enable pattern for the final (possibly partial) word
    input  wire         reverse,    // 1: process words word_count-1 .. 0

    // AXI4-lite write address channel
    output reg           axi_awvalid,
    output reg  [31:0]   axi_awaddr,
    input  wire          axi_awready,

    // AXI4-lite write data channel
    output reg           axi_wvalid,
    output reg  [31:0]   axi_wdata,
    output reg  [3:0]    axi_wstrb,
    input  wire          axi_wready,

    // AXI4-lite write response channel
    input  wire          axi_bvalid,
    output reg            axi_bready,

    // FIFO pop side
    output reg           fifo_pop,
    input  wire [31:0]   fifo_pop_data,
    input  wire          fifo_empty,

    output reg            xfer_done   // level, 1 once all `word_count` words have been written
);

    localparam IDLE = 2'd0, XFER = 2'd1, RESP = 2'd2;
    reg [1:0]  state;
    reg [15:0] wr_cnt;   // words processed so far (0..word_count-1, in processing order)
    reg        aw_done, w_done;

    wire [15:0] word_idx = reverse ? (word_count - 16'd1 - wr_cnt) : wr_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            wr_cnt      <= 16'd0;
            axi_awvalid <= 1'b0;
            axi_awaddr  <= 32'd0;
            axi_wvalid  <= 1'b0;
            axi_wdata   <= 32'd0;
            axi_wstrb   <= 4'd0;
            axi_bready  <= 1'b0;
            aw_done     <= 1'b0;
            w_done      <= 1'b0;
            fifo_pop    <= 1'b0;
            xfer_done   <= 1'b0;
        end else begin
            fifo_pop <= 1'b0; // default: 1-cycle pulse

            case (state)
                IDLE: begin
                    if (busy && !fifo_empty) begin
                        axi_awaddr  <= dst_base + (word_idx << 2);
                        axi_awvalid <= 1'b1;
                        axi_wdata   <= fifo_pop_data;
                        axi_wstrb   <= (word_idx == word_count - 16'd1) ? last_wstrb : 4'b1111;
                        axi_wvalid  <= 1'b1;
                        fifo_pop    <= 1'b1;
                        aw_done     <= 1'b0;
                        w_done      <= 1'b0;
                        state       <= XFER;
                    end
                end

                XFER: begin
                    if (axi_awvalid && axi_awready) begin
                        axi_awvalid <= 1'b0;
                        aw_done     <= 1'b1;
                    end
                    if (axi_wvalid && axi_wready) begin
                        axi_wvalid <= 1'b0;
                        w_done     <= 1'b1;
                    end

                    if ((aw_done || (axi_awvalid && axi_awready)) &&
                        (w_done  || (axi_wvalid  && axi_wready))) begin
                        axi_bready <= 1'b1;
                        state      <= RESP;
                    end
                end

                RESP: begin
                    if (axi_bready && axi_bvalid) begin
                        axi_bready <= 1'b0;
                        wr_cnt     <= wr_cnt + 16'd1;
                        state      <= IDLE;
                        if (wr_cnt + 16'd1 == word_count)
                            xfer_done <= 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase

            if (start) begin
                state       <= IDLE;
                wr_cnt      <= 16'd0;
                axi_awvalid <= 1'b0;
                axi_wvalid  <= 1'b0;
                axi_bready  <= 1'b0;
                aw_done     <= 1'b0;
                w_done      <= 1'b0;
                fifo_pop    <= 1'b0;
                xfer_done   <= 1'b0;
            end
        end
    end

endmodule
