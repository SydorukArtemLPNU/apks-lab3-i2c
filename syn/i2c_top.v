// i2c_top.v — FPGA top for synthesis: I2C master with open-drain pads.
// The command interface is brought out to pins so nothing is optimised away.
`timescale 1ns / 1ps
module i2c_top (
    input  wire       clk,        // 12 MHz board oscillator
    input  wire       rst_n,
    input  wire       cmd_valid,
    output wire       cmd_ready,
    input  wire [2:0] cmd,
    input  wire [7:0] din,
    input  wire       read_nack,
    output wire [7:0] dout,
    output wire       ack,
    output wire       done,
    output wire       bus_busy,
    inout  wire       scl,        // open drain, external pull-up
    inout  wire       sda
);
    wire scl_oe, sda_oe;
    i2c_master #(.CLK_HZ(12_000_000), .I2C_HZ(400_000)) u_master (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready), .cmd(cmd), .din(din),
        .read_nack(read_nack), .dout(dout), .ack(ack), .done(done), .bus_busy(bus_busy),
        .scl_i(scl), .scl_oe(scl_oe), .sda_i(sda), .sda_oe(sda_oe));
    assign scl = scl_oe ? 1'b0 : 1'bz;
    assign sda = sda_oe ? 1'b0 : 1'bz;
endmodule
