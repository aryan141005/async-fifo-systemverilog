`timescale 1ns/1ps

// Final testbench for the 16 x 32 asynchronous FIFO.
// Checks: data integrity (scoreboard), overflow/underflow protection,
// Gray-code pointer integrity, reset behaviour (power-up and mid-run),
// six clock-ratio configurations with random phase + jitter,
// and a directed coverage checklist printed at the end.

module tb;
  localparam DW    = 32;
  localparam AW    = 4;
  localparam DEPTH = 1 << AW;

  // Change this (0,1,2,...) to get a different random stream
  parameter SEED_OFFSET = 0;

  // clock periods in ns, changed during the test
  real wper = 10.0;
  real rper = 27.0;

  logic          wclk = 0, rclk = 0;
  logic          wrst_n, rrst_n;
  logic          winc, rinc;
  logic [DW-1:0] wdata;
  logic [DW-1:0] rdata;
  logic          wfull, rempty;

  async_fifo #(DW, AW) dut (
    .wclk(wclk), .wrst_n(wrst_n), .winc(winc), .wdata(wdata), .wfull(wfull),
    .rclk(rclk), .rrst_n(rrst_n), .rinc(rinc), .rdata(rdata), .rempty(rempty)
  );

  // ---------------- Counters and flags (declared first) ----------------
  logic [DW-1:0] q[$];
  logic [DW-1:0] expected;
  int errors = 0, wr_cnt = 0, rd_cnt = 0, occ = 0, max_occ = 0;
  int ovf_att = 0, udf_att = 0;
  int gray_chg = 0, gray_viol = 0;
  int ratios_done = 0, resets_mid = 0;
  bit saw_full = 0, saw_empty = 0;
  bit pass;

  logic [AW:0] wprev = '0, rprev = '0, dw, dr;
  int dummy;
  int wj, rj;

  // ---------------- Random seed offset ----------------
  initial repeat (SEED_OFFSET) dummy = $urandom;

  // ---------------- Clocks: random start phase + jitter ----------------
  // jitter is +/-0.5 ns on every half period
  initial begin
    #($urandom_range(0,10));
    forever begin
      wj = $urandom_range(0,100);
      wj = wj - 50;
      #(wper/2.0 + wj/100.0);
      wclk = ~wclk;
    end
  end

  initial begin
    #($urandom_range(0,10));
    forever begin
      rj = $urandom_range(0,100);
      rj = rj - 50;
      #(rper/2.0 + rj/100.0);
      rclk = ~rclk;
    end
  end

  // ---------------- Scoreboard ----------------
  always @(posedge wclk) begin
    if (wrst_n && winc && !wfull) begin
      q.push_back(wdata);
      wr_cnt++;
      occ = wr_cnt - rd_cnt;
      if (occ > DEPTH) begin
        $display("[%0t] ERROR: overflow, write accepted beyond DEPTH", $time);
        errors++;
      end
      if (occ > max_occ) max_occ = occ;
    end
    if (wrst_n && winc && wfull) ovf_att++;
    if (wfull) saw_full = 1;
  end

  always @(posedge rclk) begin
    if (rrst_n && rinc && !rempty) begin
      if (q.size() == 0) begin
        $display("[%0t] ERROR: underflow, read with empty scoreboard", $time);
        errors++;
      end else begin
        expected = q.pop_front();
        if (rdata !== expected) begin
          $display("[%0t] ERROR: rdata=%h expected=%h", $time, rdata, expected);
          errors++;
        end
      end
      rd_cnt++;
    end
    if (rrst_n && rinc && rempty) udf_att++;
    if (rempty && rd_cnt > 0) saw_empty = 1;
  end

  // ---------------- Gray-code check: only 1 bit may change per step ----------------
  always @(dut.wptr) begin
    dw = dut.wptr ^ wprev;
    if (wrst_n === 1'b1) begin
      gray_chg++;
      if ((dw & (dw - 1)) != 0) begin
        $display("[%0t] ERROR: wptr changed more than 1 bit", $time);
        gray_viol++; errors++;
      end
    end
    wprev = dut.wptr;
  end

  always @(dut.rptr) begin
    dr = dut.rptr ^ rprev;
    if (rrst_n === 1'b1) begin
      gray_chg++;
      if ((dr & (dr - 1)) != 0) begin
        $display("[%0t] ERROR: rptr changed more than 1 bit", $time);
        gray_viol++; errors++;
      end
    end
    rprev = dut.rptr;
  end

  // ---------------- Helper tasks ----------------
  task do_reset;
    begin
      winc = 0; rinc = 0; wdata = 0;
      wrst_n = 0; rrst_n = 0;
      #100;
      wrst_n = 1;          // release at different times (async resets)
      #37;
      rrst_n = 1;
      repeat (3) @(posedge wclk);
      repeat (3) @(posedge rclk);
    end
  endtask

  task fill_to_full;
    begin
      while (!wfull) begin
        winc = 1; wdata = $urandom;
        @(posedge wclk); #1;
      end
      winc = 0;
    end
  endtask

  task overflow_attempt;
    int cnt0;
    begin
      cnt0 = wr_cnt;
      repeat (3) begin
        winc = 1; wdata = $urandom;
        @(posedge wclk); #1;
      end
      winc = 0;
      if (wr_cnt !== cnt0) begin
        $display("ERROR: write accepted while full");
        errors++;
      end
    end
  endtask

  task drain_to_empty;
    begin
      repeat (4) @(posedge rclk);   // let write pointer sync across
      while (!rempty) begin
        rinc = 1;
        @(posedge rclk); #1;
      end
      rinc = 0;
    end
  endtask

  task underflow_attempt;
    int cnt0;
    begin
      cnt0 = rd_cnt;
      repeat (3) begin
        rinc = 1;
        @(posedge rclk); #1;
      end
      rinc = 0;
      if (rd_cnt !== cnt0) begin
        $display("ERROR: read accepted while empty");
        errors++;
      end
    end
  endtask

  // after a drain: FIFO must be empty, not full, and counts must match
  task check_settled_empty;
    begin
      repeat (6) @(posedge rclk);
      repeat (6) @(posedge wclk);
      #1;
      if (rempty !== 1'b1) begin
        $display("ERROR: rempty not high after drain");
        errors++;
      end
      if (wfull !== 1'b0) begin
        $display("ERROR: wfull still high after drain");
        errors++;
      end
      if (wr_cnt !== rd_cnt) begin
        $display("ERROR: wr_cnt=%0d rd_cnt=%0d after drain", wr_cnt, rd_cnt);
        errors++;
      end
    end
  endtask

  task writer(input int n, input int pct);
    int target;
    begin
      target = wr_cnt + n;
      while (wr_cnt < target) begin
        winc  = ($urandom_range(99) < pct);
        wdata = $urandom;
        @(posedge wclk); #1;
      end
      winc = 0;
    end
  endtask

  task reader(input int n, input int pct);
    int target;
    begin
      target = rd_cnt + n;
      while (rd_cnt < target) begin
        rinc = ($urandom_range(99) < pct);
        @(posedge rclk); #1;
      end
      rinc = 0;
    end
  endtask

  task random_phase(input int n, input int wpct, input int rpct);
    begin
      fork
        writer(n, wpct);
        reader(n, rpct);
      join
    end
  endtask

  // one clock-ratio configuration: four traffic patterns
  task run_ratio(input real wp, input real rp);
    begin
      wper = wp; rper = rp;
      $display("[%0t] RATIO: wclk period %0.1f ns, rclk period %0.1f ns", $time, wp, rp);
      random_phase(250, 90, 30);    // fast write, slow read
      random_phase(250, 30, 90);    // slow write, fast read
      random_phase(250, 60, 60);    // balanced
      random_phase(250, 100, 100);  // back-to-back both sides
      ratios_done++;
    end
  endtask

  // reset while the FIFO holds data; w_first selects which reset asserts first
  task reset_midrun(input bit w_first);
    begin
      drain_to_empty;
      check_settled_empty;
      writer(10, 100);              // leave 10 words inside
      winc = 0; rinc = 0;
      if (w_first) begin wrst_n = 0; #13; rrst_n = 0; end
      else         begin rrst_n = 0; #13; wrst_n = 0; end
      #60;
      if (w_first) begin wrst_n = 1; #29; rrst_n = 1; end
      else         begin rrst_n = 1; #29; wrst_n = 1; end
      repeat (4) @(posedge wclk);
      repeat (4) @(posedge rclk);
      #1;
      q.delete();                   // reset throws away stored data
      rd_cnt = wr_cnt;
      if (rempty !== 1'b1 || wfull !== 1'b0) begin
        $display("ERROR: bad flags after mid-run reset (rempty=%b wfull=%b)", rempty, wfull);
        errors++;
      end
      resets_mid++;
      random_phase(100, 60, 60);    // FIFO must still work after reset
    end
  endtask

  // ---------------- Main test ----------------
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(0, tb);

    do_reset;
    if (rempty !== 1'b1 || wfull !== 1'b0) begin
      $display("ERROR: bad flags after reset (rempty=%b wfull=%b)", rempty, wfull);
      errors++;
    end

    $display("[%0t] TEST 1: fill to full + overflow attempt", $time);
    fill_to_full;
    occ = wr_cnt - rd_cnt;
    if (occ !== DEPTH) begin
      $display("ERROR: expected %0d words when full, got %0d", DEPTH, occ);
      errors++;
    end
    overflow_attempt;

    $display("[%0t] TEST 2: drain to empty + underflow attempt", $time);
    drain_to_empty;
    underflow_attempt;
    check_settled_empty;

    $display("[%0t] TEST 3: clock ratio sweep with phase offset and jitter", $time);
    run_ratio(10.0, 27.0);
    run_ratio(27.0, 10.0);
    run_ratio(10.0, 10.7);
    run_ratio(10.7, 10.0);
    run_ratio(10.0, 10.0);
    run_ratio(10.0, 6.1);

    $display("[%0t] TEST 4: reset while FIFO holds data", $time);
    reset_midrun(1);
    reset_midrun(0);

    drain_to_empty;
    check_settled_empty;

    // ---------------- Report ----------------
    $display("============================================");
    $display("Words written / read        : %0d / %0d", wr_cnt, rd_cnt);
    $display("Clock ratios tested         : %0d", ratios_done);
    $display("Max occupancy               : %0d of %0d", max_occ, DEPTH);
    $display("Overflow attempts ignored   : %0d", ovf_att);
    $display("Underflow attempts ignored  : %0d", udf_att);
    $display("Gray-code steps checked     : %0d", gray_chg);
    $display("Gray-code violations        : %0d", gray_viol);
    $display("Mid-run resets              : %0d", resets_mid);
    $display("Saw full / saw empty        : %0b / %0b", saw_full, saw_empty);
    $display("Total errors                : %0d", errors);

    if (!saw_full)         $display("MISSING: full never reached");
    if (!saw_empty)        $display("MISSING: empty never reached after data");
    if (max_occ != DEPTH)  $display("MISSING: FIFO never completely filled");
    if (ovf_att == 0)      $display("MISSING: no overflow attempt");
    if (udf_att == 0)      $display("MISSING: no underflow attempt");
    if (gray_chg == 0)     $display("MISSING: Gray-code checker never ran");
    if (ratios_done != 6)  $display("MISSING: not all clock ratios ran");
    if (resets_mid != 2)   $display("MISSING: mid-run resets not done");

    pass = (errors == 0) && (wr_cnt == rd_cnt) && (q.size() == 0) &&
           saw_full && saw_empty && (max_occ == DEPTH) &&
           (ovf_att > 0) && (udf_att > 0) && (gray_chg > 0) &&
           (gray_viol == 0) && (ratios_done == 6) && (resets_mid == 2);

    if (pass) $display("RESULT: PASS");
    else      $display("RESULT: FAIL");
    $display("============================================");
    $finish;
  end

  // keep the waveform file small: record only the first 1.5 us
  initial begin
    #1500;
    $dumpoff;
  end

  // watchdog
  initial begin
    #10_000_000;
    $display("RESULT: FAIL (timeout)");
    $finish;
  end
endmodule
