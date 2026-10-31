// slave_top.v — FPGA top for synthesis: I2C slave (address 0x50) with open-drain SDA.
`timescale 1ns / 1ps
module slave_top (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       scl,
    inout  wire       sda,
    output wire [7:0] rx_data,
    output wire       rx_valid,
    output wire [3:0] ptr
);
    wire sda_oe;
    i2c_slave #(.ADDR(7'h50)) u_slave (
        .clk(clk), .rst_n(rst_n), .scl_i(scl), .sda_i(sda), .sda_oe(sda_oe),
        .rx_data(rx_data), .rx_valid(rx_valid), .ptr(ptr));
    assign sda = sda_oe ? 1'b0 : 1'bz;
endmodule
