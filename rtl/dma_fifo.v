`timescale 1ns/1ps
//==============================================================================
// dma_fifo.v
//
// Small synchronous FIFO used to decouple the read engine from the write
// engine so they can run concurrently (read engine gets ahead of the write
// engine, up to DEPTH words, instead of the two ping-ponging in lockstep).
//
// `flush` synchronously clears the FIFO (used on a new DMA start so stale
// leftover words from a previous transfer, if any, can never leak through).
//==============================================================================
module dma_fifo #(
    parameter WIDTH = 32,
    parameter DEPTH = 4,
    parameter AW    = 2          // ceil(log2(DEPTH)) address bits for the pointers
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire             flush,

    input  wire             push,
    input  wire [WIDTH-1:0] push_data,
    output wire             full,

    input  wire             pop,
    output wire [WIDTH-1:0] pop_data,
    output wire             empty
);

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [AW-1:0]    wptr, rptr;
    reg [AW:0]      count;   // one extra bit so it can represent DEPTH itself

    assign full     = (count == DEPTH);
    assign empty    = (count == 0);
    assign pop_data = mem[rptr];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr  <= {AW{1'b0}};
            rptr  <= {AW{1'b0}};
            count <= {(AW+1){1'b0}};
        end else if (flush) begin
            wptr  <= {AW{1'b0}};
            rptr  <= {AW{1'b0}};
            count <= {(AW+1){1'b0}};
        end else begin
            if (push) begin
                mem[wptr] <= push_data;
                wptr      <= wptr + 1'b1;
            end
            if (pop) begin
                rptr <= rptr + 1'b1;
            end

            case ({push, pop})
                2'b10:   count <= count + 1'b1;  // push only
                2'b01:   count <= count - 1'b1;  // pop only
                default: count <= count;         // 00: no change, 11: push+pop cancel out
            endcase
        end
    end

endmodule
