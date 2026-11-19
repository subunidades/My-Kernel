// Makerchip test wrapper for the combined chip (module "chip").
// twist_in (96 bits, MSB first): wx wy wz vx vy vz, 16 bits each.
// pose_out (128 bits): rW rX rY rZ dW dX dY dZ, 16 bits each.
// Test: constant twist (wx=1000, vy=100) for 120+ cycles. The rotation must
// have moved, |r| must stay within 0.2% of 1, and |r.d| must stay <= 8 LSB.
module top(input wire clk, input wire reset, input wire [31:0] cyc_cnt,
           output wire passed, output wire failed);

  wire [95:0]  twist_in = {16'sh03E8, 16'sh0000, 16'sh0000,
                           16'sh0000, 16'sh0064, 16'sh0000};
  wire [127:0] pose_out;

  chip dut (.clk(clk), .rst(reset), .en(1'b1),
            .twist_in(twist_in), .pose_out(pose_out));

  wire signed [15:0] o_rW = pose_out[127:112];
  wire signed [15:0] o_rX = pose_out[111:96];
  wire signed [15:0] o_rY = pose_out[95:80];
  wire signed [15:0] o_rZ = pose_out[79:64];
  wire signed [15:0] o_dW = pose_out[63:48];
  wire signed [15:0] o_dX = pose_out[47:32];
  wire signed [15:0] o_dY = pose_out[31:16];
  wire signed [15:0] o_dZ = pose_out[15:0];

  wire signed [31:0] nsq  = o_rW*o_rW + o_rX*o_rX + o_rY*o_rY + o_rZ*o_rZ;
  wire signed [31:0] nerr = nsq - 32'sd1073741824;
  wire norm_ok = (nerr > -2147483) && (nerr < 2147483);

  wire signed [31:0] dot_full = o_rW*o_dW + o_rX*o_dX + o_rY*o_dY + o_rZ*o_dZ;
  wire signed [31:0] dot      = dot_full >>> 15;
  wire dot_ok = (dot >= -8) && (dot <= 8);

  wire moved = (o_rW != 16'sh7FFF);
  wire ok = moved && norm_ok && dot_ok;

  assign passed = (cyc_cnt >= 32'd120) &&  ok;
  assign failed = (cyc_cnt >= 32'd120) && !ok;
endmodule
