// Simulation driver: provides clk, reset and cyc_cnt like Makerchip does and
// reports PASS or FAIL for the wrapper module "top".
module tb;
  reg clk = 0;
  always #5 clk = ~clk;
  reg reset = 1;
  reg [31:0] cyc = 0;
  wire passed, failed;
  top dut (.clk(clk), .reset(reset), .cyc_cnt(cyc), .passed(passed), .failed(failed));
  always @(posedge clk) cyc <= cyc + 1;
  initial begin #22 reset = 0; end
  always @(posedge clk) begin
    if (failed)           begin $display("RESULT: FAIL at cycle %0d", cyc); $finish; end
    else if (passed)      begin $display("RESULT: PASS at cycle %0d", cyc); $finish; end
    else if (cyc > 6000)  begin $display("RESULT: FAIL (timeout)");         $finish; end
  end
`ifdef DUMP
  // one line per cycle for sim/check_dump.py
  always @(posedge clk) begin
    #1 $display("D %h %b %b %h %h", dut.tw_q, dut.en_q, dut.rst_q, dut.pose_out, dut.ref_pose);
  end
`endif
endmodule
