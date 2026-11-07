`timescale 1ns/1ps

// Asynchronous FIFO with Gray-code pointers (Cummings style)
// 16 entries x 32 bits by default

// ---------------- FIFO memory ----------------
module fifo_mem #(parameter DATA_W = 32, parameter ADDR_W = 4) (
  input  logic              wclk,
  input  logic              wclken,
  input  logic              wfull,
  input  logic [ADDR_W-1:0] waddr,
  input  logic [ADDR_W-1:0] raddr,
  input  logic [DATA_W-1:0] wdata,
  output logic [DATA_W-1:0] rdata
);
  localparam DEPTH = 1 << ADDR_W;
  logic [DATA_W-1:0] mem [0:DEPTH-1];

  assign rdata = mem[raddr];

  always_ff @(posedge wclk)
    if (wclken && !wfull) mem[waddr] <= wdata;
endmodule

// ---------------- 2-FF synchronizers ----------------
// read pointer -> write clock domain
module sync_r2w #(parameter ADDR_W = 4) (
  input  logic              wclk,
  input  logic              wrst_n,
  input  logic [ADDR_W:0]   rptr,
  output logic [ADDR_W:0]   wq2_rptr
);
  logic [ADDR_W:0] wq1_rptr;
  always_ff @(posedge wclk or negedge wrst_n)
    if (!wrst_n) {wq2_rptr, wq1_rptr} <= '0;
    else         {wq2_rptr, wq1_rptr} <= {wq1_rptr, rptr};
endmodule

// write pointer -> read clock domain
module sync_w2r #(parameter ADDR_W = 4) (
  input  logic              rclk,
  input  logic              rrst_n,
  input  logic [ADDR_W:0]   wptr,
  output logic [ADDR_W:0]   rq2_wptr
);
  logic [ADDR_W:0] rq1_wptr;
  always_ff @(posedge rclk or negedge rrst_n)
    if (!rrst_n) {rq2_wptr, rq1_wptr} <= '0;
    else         {rq2_wptr, rq1_wptr} <= {rq1_wptr, wptr};
endmodule

// ---------------- Read pointer + empty flag (read domain) ----------------
module rptr_empty #(parameter ADDR_W = 4) (
  input  logic              rclk,
  input  logic              rrst_n,
  input  logic              rinc,
  input  logic [ADDR_W:0]   rq2_wptr,
  output logic              rempty,
  output logic [ADDR_W-1:0] raddr,
  output logic [ADDR_W:0]   rptr
);
  logic [ADDR_W:0] rbin;
  logic [ADDR_W:0] rbnext, rgnext;

  assign rbnext = rbin + (rinc & ~rempty);
  assign rgnext = (rbnext >> 1) ^ rbnext;      // binary -> Gray
  assign raddr  = rbin[ADDR_W-1:0];

  always_ff @(posedge rclk or negedge rrst_n)
    if (!rrst_n) {rbin, rptr} <= '0;
    else         {rbin, rptr} <= {rbnext, rgnext};

  // empty when next read Gray pointer equals synced write Gray pointer
  always_ff @(posedge rclk or negedge rrst_n)
    if (!rrst_n) rempty <= 1'b1;
    else         rempty <= (rgnext == rq2_wptr);
endmodule

// ---------------- Write pointer + full flag (write domain) ----------------
module wptr_full #(parameter ADDR_W = 4) (
  input  logic              wclk,
  input  logic              wrst_n,
  input  logic              winc,
  input  logic [ADDR_W:0]   wq2_rptr,
  output logic              wfull,
  output logic [ADDR_W-1:0] waddr,
  output logic [ADDR_W:0]   wptr
);
  logic [ADDR_W:0] wbin;
  logic [ADDR_W:0] wbnext, wgnext;

  assign wbnext = wbin + (winc & ~wfull);
  assign wgnext = (wbnext >> 1) ^ wbnext;      // binary -> Gray
  assign waddr  = wbin[ADDR_W-1:0];

  always_ff @(posedge wclk or negedge wrst_n)
    if (!wrst_n) {wbin, wptr} <= '0;
    else         {wbin, wptr} <= {wbnext, wgnext};

  // full when next write Gray pointer equals synced read Gray pointer
  // with the top two bits inverted
  always_ff @(posedge wclk or negedge wrst_n)
    if (!wrst_n) wfull <= 1'b0;
    else         wfull <= (wgnext == {~wq2_rptr[ADDR_W:ADDR_W-1],
                                       wq2_rptr[ADDR_W-2:0]});
endmodule

// ---------------- Top level ----------------
module async_fifo #(parameter DATA_W = 32, parameter ADDR_W = 4) (
  // write domain
  input  logic              wclk,
  input  logic              wrst_n,
  input  logic              winc,
  input  logic [DATA_W-1:0] wdata,
  output logic              wfull,
  // read domain
  input  logic              rclk,
  input  logic              rrst_n,
  input  logic              rinc,
  output logic [DATA_W-1:0] rdata,
  output logic              rempty
);
  logic [ADDR_W-1:0] waddr, raddr;
  logic [ADDR_W:0]   wptr, rptr, wq2_rptr, rq2_wptr;

  sync_r2w  #(ADDR_W) u_sync_r2w (.wclk(wclk), .wrst_n(wrst_n),
                                  .rptr(rptr), .wq2_rptr(wq2_rptr));
  sync_w2r  #(ADDR_W) u_sync_w2r (.rclk(rclk), .rrst_n(rrst_n),
                                  .wptr(wptr), .rq2_wptr(rq2_wptr));

  fifo_mem  #(DATA_W, ADDR_W) u_mem (.wclk(wclk), .wclken(winc), .wfull(wfull),
                                     .waddr(waddr), .raddr(raddr),
                                     .wdata(wdata), .rdata(rdata));

  rptr_empty #(ADDR_W) u_rptr_empty (.rclk(rclk), .rrst_n(rrst_n), .rinc(rinc),
                                     .rq2_wptr(rq2_wptr), .rempty(rempty),
                                     .raddr(raddr), .rptr(rptr));

  wptr_full  #(ADDR_W) u_wptr_full  (.wclk(wclk), .wrst_n(wrst_n), .winc(winc),
                                     .wq2_rptr(wq2_rptr), .wfull(wfull),
                                     .waddr(waddr), .wptr(wptr));
endmodule
