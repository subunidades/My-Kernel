// FPGA wrapper for the chip on the ULX3S (25 MHz board clock).
// * The chip is one long combinational path per update (about 242 ns on the
//   ECP5-85F), so it runs at 2.857 MHz, not 25 MHz. The PLL output must be
//   10 MHz or more and CLKDIVF only divides by 2 or 3.5, so the PLL makes
//   10 MHz and a CLKDIVF divides it by 3.5.
// * The chip has 227 I/O bits, more than the board has pins, so twist_in is
//   filled by a serial shift register and pose_out is folded to one observed
//   bit. Every input and output bit stays live, so nothing is optimized away.
module fpga_top(input wire clk_25mhz, input wire rst, input wire en,
                input wire ser_in, output wire ser_out);
  wire clk_pll, clk_chip, locked;
  pll u_pll (.clkin(clk_25mhz), .clkout0(clk_pll), .locked(locked));   // 10 MHz
  (* FREQUENCY_PIN_CDIVX="2.857" *)
  CLKDIVF #(.DIV("3.5")) u_div (.CLKI(clk_pll), .RST(1'b0), .ALIGNWD(1'b0),
                                .CDIVX(clk_chip));                        // 2.857 MHz
  wire rst_i = rst | ~locked;              // hold the chip in reset until the PLL locks

  reg [95:0] twist_sh = 96'd0;
  always @(posedge clk_chip) twist_sh <= {twist_sh[94:0], ser_in};

  wire [127:0] pose_out;
  chip u_chip (.clk(clk_chip), .rst(rst_i), .en(en),
               .twist_in(twist_sh), .pose_out(pose_out));

  reg fold = 1'b0;
  always @(posedge clk_chip) fold <= ^pose_out;
  assign ser_out = fold;
endmodule
