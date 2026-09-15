//14 case
`timescale 1ns/1ps

module dma_top_tb;
    reg clk = 0;
    reg rst_n = 0;

    always #5 clk = ~clk;

    // --- APB Bus ---
    wire            psel;
    wire            penable;
    wire            pwrite;
    wire [7:0]      paddr;
    wire [31:0]     pwdata;
    wire [31:0]     prdata;
    wire            pready;

    reg             apb_start;
    reg             apb_rw;
    reg [7:0]       apb_addr;
    reg [31:0]      apb_wdata;
    wire [31:0]     apb_rdata;
    wire            apb_idle;
    wire            apb_busy;

    wire            axi_awvalid;
    wire [31:0]     axi_awaddr;
    wire            axi_awready;

    wire            axi_wvalid;
    wire [31:0]     axi_wdata;
    wire [3:0]      axi_wstrb;
    wire            axi_wready;

    wire            axi_bvalid;
    wire            axi_bready;

    wire            axi_arvalid;
    wire [31:0]     axi_araddr;
    wire            axi_arready;

    wire            axi_rvalid;
    wire [31:0]     axi_rdata;
    wire            axi_rready;

    integer total_errors = 0;

    dma dma_inst (
        .clk(clk), .rst_n(rst_n),
        .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr),
        .pwdata(pwdata), .prdata(prdata), .pready(pready),
        .axi_awvalid(axi_awvalid), .axi_awaddr(axi_awaddr), .axi_awready(axi_awready),
        .axi_wvalid(axi_wvalid), .axi_wdata(axi_wdata), .axi_wstrb(axi_wstrb), .axi_wready(axi_wready),
        .axi_bvalid(axi_bvalid), .axi_bready(axi_bready),
        .axi_arvalid(axi_arvalid), .axi_araddr(axi_araddr), .axi_arready(axi_arready),
        .axi_rvalid(axi_rvalid), .axi_rdata(axi_rdata), .axi_rready(axi_rready)
    );

    apb_master apb_m (
        .clk(clk), .rst_n(rst_n),
        .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr),
        .pwdata(pwdata), .prdata(prdata), .pready(pready),
        .start(apb_start), .rw(apb_rw), .addr(apb_addr), .wdata(apb_wdata),
        .rdata(apb_rdata), .idle(apb_idle), .busy(apb_busy)
    );

    axi4_ram_slave mem (
        .clk(clk), .rst_n(rst_n),
        .awvalid(axi_awvalid), .awaddr(axi_awaddr), .awready(axi_awready),
        .wvalid(axi_wvalid), .wdata(axi_wdata), .wstrb(axi_wstrb), .wready(axi_wready),
        .bvalid(axi_bvalid), .bready(axi_bready),
        .arvalid(axi_arvalid), .araddr(axi_araddr), .arready(axi_arready),
        .rvalid(axi_rvalid), .rdata(axi_rdata), .rready(axi_rready)
    );

    // ---------------- APB helper tasks ----------------
    task automatic apb_write(input [7:0] addr, input [31:0] data);
        @(posedge clk);
        while (!apb_idle) @(posedge clk);
        apb_addr  = addr;
        apb_wdata = data;
        apb_rw    = 1;
        apb_start = 1;
        @(posedge clk);
        apb_start = 0;
        while (!apb_idle) @(posedge clk);
    endtask

    task automatic apb_read(input [7:0] addr, output [31:0] data);
        @(posedge clk);
        while (!apb_idle) @(posedge clk);
        apb_addr  = addr;
        apb_rw    = 0;
        apb_start = 1;
        @(posedge clk);
        apb_start = 0;
        while (!apb_idle) @(posedge clk);
        data = apb_rdata;
    endtask

    // Poll the status register (0x04) instead of a fixed delay.
    // This is also the first place that actually exercises busy/done
    // read-back through the APB interface, which the original TB never did.
    task automatic wait_for_done;
        reg [31:0] status;
        integer    cnt;
        reg        finished;
        begin
            cnt      = 0;
            finished = 0;
            while (!finished) begin
                apb_read(8'h04, status);
                cnt = cnt + 1;
                if (!status[0] && status[1])
                    finished = 1;
                else if (cnt > 200000) begin
                    $error("[TIMEOUT] status never showed busy=0/done=1 after %0d polls", cnt);
                    total_errors = total_errors + 1;
                    finished = 1;
                end
            end
        end
    endtask

    // ---------------- Test task ----------------
    // Snapshots the source region BEFORE the transfer starts, so the
    // check is correct even when src/dst overlap or are identical
    // (comparing dst against live src *after* the transfer breaks in
    // those cases, since src may already have been overwritten).
    task automatic run_dma_test(
        input [31:0] src_addr,
        input [31:0] dst_addr,
        input [15:0] size
    );
        integer i, nwords, mismatches;
        reg [31:0] golden [0:16383];
        time t_start;
        integer elapsed_cycles;
        real cycles_per_word;
        begin
            nwords = (size + 3) / 4;

            for (i = 0; i < nwords; i = i + 1)
                golden[i] = dma_top_tb.mem.mem[(src_addr >> 2) + i];

            apb_write(8'h08, src_addr);
            apb_write(8'h0C, dst_addr);
            apb_write(8'h00, {size, 16'h0001}); // start = 1

            t_start = $time;
            wait_for_done();
            elapsed_cycles = ($time - t_start) / 10;
            cycles_per_word = (nwords > 0) ? real'(elapsed_cycles) / real'(nwords) : 0.0;

            mismatches = 0;
            for (i = 0; i < nwords; i = i + 1) begin
                reg [31:0] actual;
                reg [31:0] cmp_mask;
                actual = dma_top_tb.mem.mem[(dst_addr >> 2) + i];
                if (i == nwords - 1 && size[1:0] != 2'd0) begin
                    // last word may be partially written (wstrb masks some bytes);
                    // only the valid low bytes are guaranteed to match the source
                    case (size[1:0])
                        2'd1: cmp_mask = 32'h000000FF;
                        2'd2: cmp_mask = 32'h0000FFFF;
                        2'd3: cmp_mask = 32'h00FFFFFF;
                        default: cmp_mask = 32'hFFFFFFFF;
                    endcase
                end else begin
                    cmp_mask = 32'hFFFFFFFF;
                end
                if ((actual & cmp_mask) !== (golden[i] & cmp_mask)) begin
                    mismatches = mismatches + 1;
                    if (mismatches <= 5)
                        $display("[MISMATCH] word %0d: expected=%h got=%h (mask=%h)",
                                 i, golden[i], actual, cmp_mask);
                end
            end

            if (mismatches == 0) begin
                $display("[PASS] src=%08x dst=%08x size=%0d (%0d words) - %0d cycles, %.2f cycles/word",
                          src_addr, dst_addr, size, nwords, elapsed_cycles, cycles_per_word);
            end else begin
                total_errors = total_errors + 1;
                $display("[FAIL] src=%08x dst=%08x size=%0d : %0d/%0d words mismatched",
                          src_addr, dst_addr, size, mismatches, nwords);
            end

            // Performance check: a fully-sequential (non-overlapping) read-then-
            // write implementation needs roughly AR+ARREADY+RVALID (~3 cycles)
            // plus AW/W+ready+BVALID (~3 cycles) per word with this BFM, i.e.
            // ~6 cycles/word. A correctly overlapped implementation should stay
            // well under that once the pipeline is warmed up (small transfers
            // are skipped here since they never reach steady state).
            if (nwords > 8 && cycles_per_word > 5.5) begin
                $display("[PERF WARNING] src=%08x dst=%08x: %.2f cycles/word looks sequential, not overlapped (expected < ~5.5 for a warmed-up pipeline)",
                          src_addr, dst_addr, cycles_per_word);
                total_errors = total_errors + 1;
            end
        end
    endtask

    // ---------------- Robustness test: 2nd start while busy ----------------
    // Spec explicitly calls this "indeterministic" - so we do NOT assert
    // anything about a spurious 2nd transfer's data. What we DO assert:
    //   (a) the DMA does not lock up / hang,
    //   (b) the in-flight legitimate transfer (A) still completes correctly
    //       despite the spurious 2nd control write,
    //   (c) a subsequent, fully legitimate transfer still works - i.e. the
    //       controller returns to a healthy state afterwards.
    // (Our CSR design happens to gate start_pulse on !busy, so a write to
    // Control with start=1 while busy is simply dropped - this is one
    // valid interpretation of "indeterministic", not a requirement.)
    task automatic test_busy_restart;
        localparam [31:0] SRC_A  = 32'h00005000;
        localparam [31:0] DST_A  = 32'h00005800;
        localparam [15:0] SIZE_A = 16'd256; // 64 words - long enough to land a 2nd start mid-flight
        integer    ii, nwordsA, mism;
        reg [31:0] goldenA [0:255];
        begin
            nwordsA = (SIZE_A + 3) / 4;
            for (ii = 0; ii < nwordsA; ii = ii + 1)
                goldenA[ii] = dma_top_tb.mem.mem[(SRC_A >> 2) + ii];

            apb_write(8'h08, SRC_A);
            apb_write(8'h0C, DST_A);
            apb_write(8'h00, {SIZE_A, 16'h0001}); // start transfer A

            repeat (10) @(posedge clk); // let A get partway through
            apb_write(8'h00, {16'd8, 16'h0001}); // spurious 2nd "start" while busy

            wait_for_done(); // must not hang

            mism = 0;
            for (ii = 0; ii < nwordsA; ii = ii + 1)
                if (dma_top_tb.mem.mem[(DST_A >> 2) + ii] !== goldenA[ii])
                    mism = mism + 1;

            if (mism == 0)
                $display("[PASS] busy-restart robustness: transfer A completed correctly despite spurious 2nd start");
            else begin
                total_errors = total_errors + 1;
                $display("[FAIL] busy-restart robustness: %0d/%0d words of transfer A corrupted", mism, nwordsA);
            end
        end
    endtask

    initial begin
        $display("[TB] Starting DMA Test");
        $dumpfile("dma_top_tb.vcd");
        $dumpvars(1, dma_top_tb);      // top-level signals only
        $dumpvars(1, dma_top_tb.dma_inst);
        $dumpvars(1, dma_top_tb.apb_m);
        // NOTE: intentionally NOT dumping dma_top_tb.mem (16384-word RAM array) -
        // dumping it every cycle was the main reason large transfers timed out
        // under xrun's runtime limit. Use hierarchical access in $display /
        // the testbench itself to inspect mem[] contents instead.

        rst_n = 0;
        #50;
        rst_n = 1;

        // 1) basic aligned, non-overlapping
        run_dma_test(32'h00000000, 32'h00000800, 16'd64);
        // 2) basic aligned, non-overlapping, bigger
        run_dma_test(32'h00004000, 32'h0000C000, 16'd256);
        // 3) size NOT a multiple of 4 -> exercises last-word wstrb
        run_dma_test(32'h00001000, 32'h00002000, 16'd13);
        // 4) size = 0 -> should complete immediately, no data movement
        run_dma_test(32'h00003000, 32'h00003800, 16'd0);
        // 5) source == destination (in-place), size near a 16-bit/word_count
        //    rounding boundary (kept small enough to run fast under a
        //    wall-clock-limited simulator; still exercises the same
        //    (size+3)>>2 boundary math as a true near-64KB size would)
        run_dma_test(32'h00000000, 32'h00000000, 16'd2047);
        // 6) larger transfer, non-overlapping regions - big enough to show
        //    sustained throughput/overlap, small enough to finish quickly
        run_dma_test(32'h00000000, 32'h00002000, 16'd2048);
        // 7) overlap, 1-word offset (dst = src + 4) -- worst case for hazards
        run_dma_test(32'h00001000, 32'h00001004, 16'd64);
        // 8) overlap, offset == FIFO depth (4 words = 16 bytes)
        run_dma_test(32'h00002000, 32'h00002010, 16'd64);
        // 9) overlap, offset > FIFO depth (8 words = 32 bytes), still overlapping
        run_dma_test(32'h00003000, 32'h00003020, 16'd64);
        // 10) size at the true 16-bit max, in-place (source == destination) -
        //     exercises the real word_count boundary math, not a scaled-down stand-in
        run_dma_test(32'h00000000, 32'h00000000, 16'd65532);
        // 11) size = 65535 (not a multiple of 4) at the same near-max boundary,
        //     combining the last-word wstrb case with the size boundary case
        run_dma_test(32'h00000000, 32'h00000000, 16'd65535);
        // 12) largest non-overlapping transfer that fits in the 64KB RAM:
        //     two disjoint 32KB halves
        run_dma_test(32'h00000000, 32'h00008000, 16'd32768);
        // 13) overlap with dst < src (outside the cases the spec requires -
        //     only dst > src overlap is specified - but our forward-copy
        //     path happens to be safe here too, worth confirming)
        run_dma_test(32'h00001010, 32'h00001000, 16'd64);
        // 14) robustness: 2nd "start" while busy (spec: indeterministic) -
        //     must not hang, and the in-flight transfer must still complete
        test_busy_restart();

        $display("--------------------------------------------------");
        if (total_errors == 0)
            $display("[ALL PASS] All test cases passed.");
        else
            $display("[SUMMARY] %0d test case(s) FAILED.", total_errors);

        $finish;
    end

endmodule
