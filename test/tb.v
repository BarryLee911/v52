`default_nettype none
`timescale 1ns / 1ps
module tb;
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  reg external_drive0;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;
  tri pad0;
  wire [7:0] input_pins = {uio_in[7:1], pad0};
  assign pad0 = external_drive0 ? uio_in[0] : 1'bz;
  assign pad0 = uio_oe[0] ? uio_out[0] : 1'bz;
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(1, tb);
  end
`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif
  tt_um_sine_area_detector user_project (
`ifdef GL_TEST
      .VPWR(VPWR), .VGND(VGND),
`endif
      .ui_in(ui_in), .uo_out(uo_out), .uio_in(input_pins),
      .uio_out(uio_out), .uio_oe(uio_oe),
      .ena(ena), .clk(clk), .rst_n(rst_n)
  );
endmodule
