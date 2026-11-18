// Makerchip test wrapper for the Consensus kernel (module "kernel").
// Pose bus layout (128 bits, MSB first):
//   [127:112] rW  [111:96] rX  [95:80] rY  [79:64] rZ
//   [63:48]   dW  [47:32]  dX  [31:16] dY  [15:0]  dZ
// Test: feed an off-manifold pose (r = identity, d.w = 0.5, so r.d = 0.5).
// After projection, r.d must be ~0 and r must be unchanged.
module top(input wire clk, input wire reset, input wire [31:0] cyc_cnt,
           output wire passed, output wire failed);

  wire [127:0] pose_in  = {16'sh7FFF, 16'sh0000, 16'sh0000, 16'sh0000,
                           16'sh4000, 16'sh4000, 16'sh0000, 16'sh0000};
  wire [127:0] pose_out;

  kernel dut (.clk(clk), .rst(reset), .en(1'b1),
              .pose_in(pose_in), .pose_out(pose_out));

  wire signed [15:0] o_rW = pose_out[127:112];
  wire signed [15:0] o_rX = pose_out[111:96];
  wire signed [15:0] o_rY = pose_out[95:80];
  wire signed [15:0] o_rZ = pose_out[79:64];
  wire signed [15:0] o_dW = pose_out[63:48];
  wire signed [15:0] o_dX = pose_out[47:32];
  wire signed [15:0] o_dY = pose_out[31:16];
  wire signed [15:0] o_dZ = pose_out[15:0];

  wire signed [31:0] dot_full = o_rW*o_dW + o_rX*o_dX + o_rY*o_dY + o_rZ*o_dZ;
  wire signed [31:0] dot      = dot_full >>> 15;

  wire ok = (o_rW == 16'sh7FFF) && (dot >= -4) && (dot <= 4);

  assign passed = (cyc_cnt >= 32'd10) &&  ok;
  assign failed = (cyc_cnt >= 32'd10) && !ok;
endmodule
