// ============================================================================
// i2c_slave.v — I2C slave (receiver side) with a 16 x 8-bit register file
// ----------------------------------------------------------------------------
// Lab work #3, variant 20: I2C, Verilog.
//
// Protocol (like a small EEPROM / sensor):
//   write: S | ADDR+W | A | ptr | A | d0 | A | d1 | A ... | P
//          the first byte after the address sets the register pointer,
//          the following bytes are stored to mem[ptr], ptr auto-increments
//   read:  S | ADDR+W | A | ptr | A | Sr | ADDR+R | A | d0 | A | d1 | N | P
//          bytes mem[ptr], mem[ptr+1], ... are returned until master NACKs
//
// The slave oversamples SCL/SDA with the system clock (CLK must be at least
// ~10x faster than SCL): two-flop synchronisers + edge detectors produce
//   scl_rise / scl_fall — data is sampled on rise and changed after fall;
//   start_det          — SDA falls while SCL is high (START / repeated START);
//   stop_det           — SDA rises while SCL is high (STOP).
// sda_oe = 1 pulls SDA low (ACK or a transmitted '0').
// ============================================================================
`timescale 1ns / 1ps
module i2c_slave #(
    parameter [6:0] ADDR = 7'h50       // 7-bit slave address
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       scl_i,           // SCL line level
    input  wire       sda_i,           // SDA line level
    output reg        sda_oe,          // 1 = drive SDA low
    output reg  [7:0] rx_data,         // last data byte written by the master
    output reg        rx_valid,        // one-clock pulse with rx_data
    output reg  [3:0] ptr              // register pointer (for debug/tests)
);
    // FSM states
    localparam [2:0] S_IDLE    = 3'd0,  // not addressed, wait for START
                     S_ADDR    = 3'd1,  // receiving address byte
                     S_ACK_A   = 3'd2,  // driving ACK for the address
                     S_RX      = 3'd3,  // receiving a data byte
                     S_ACK_RX  = 3'd4,  // driving ACK for a data byte
                     S_TX      = 3'd5,  // transmitting a data byte
                     S_ACK_TX  = 3'd6;  // reading master ACK/NACK

    reg [7:0] mem [0:15];               // register file
    reg [2:0] state;
    reg [3:0] bitcnt;                   // bits received/sent in the byte (0..8)
    reg [7:0] sr;                       // shift register
    reg       rw;                       // 1 = master reads
    reg       first;                    // next RX byte is the register pointer
    reg       m_nack;                   // master answered NACK

    // synchronisers and edge detection
    reg [2:0] scl_d, sda_d;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            scl_d <= 3'b111;
            sda_d <= 3'b111;
        end else begin
            scl_d <= {scl_d[1:0], scl_i};
            sda_d <= {sda_d[1:0], sda_i};
        end
    wire scl_rise  = (scl_d[2:1] == 2'b01);
    wire scl_fall  = (scl_d[2:1] == 2'b10);
    wire scl_high  = scl_d[1] & scl_d[2];
    wire sda_s     = sda_d[1];
    wire start_det = scl_high && (sda_d[2:1] == 2'b10);
    wire stop_det  = scl_high && (sda_d[2:1] == 2'b01);

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            bitcnt   <= 4'd0;
            sr       <= 8'h00;
            rw       <= 1'b0;
            first    <= 1'b0;
            m_nack   <= 1'b0;
            sda_oe   <= 1'b0;
            rx_data  <= 8'h00;
            rx_valid <= 1'b0;
            ptr      <= 4'd0;
            for (i = 0; i < 16; i = i + 1) mem[i] <= 8'h00;
        end else begin
            rx_valid <= 1'b0;

            if (start_det) begin                 // START or repeated START
                state  <= S_ADDR;
                bitcnt <= 4'd0;
                sda_oe <= 1'b0;
            end else if (stop_det) begin         // STOP: release the bus
                state  <= S_IDLE;
                sda_oe <= 1'b0;
            end else begin
                case (state)
                S_IDLE: sda_oe <= 1'b0;
                // ---------------- address byte ------------------------
                S_ADDR: begin
                    if (scl_rise) begin
                        sr     <= {sr[6:0], sda_s};
                        bitcnt <= bitcnt + 1'b1;
                    end
                    // after the falling edge of the 8th bit decide on ACK
                    if (scl_fall && bitcnt == 4'd8) begin
                        if (sr[7:1] == ADDR) begin
                            sda_oe <= 1'b1;          // ACK
                            rw     <= sr[0];
                            state  <= S_ACK_A;
                        end else
                            state  <= S_IDLE;        // not our address
                    end
                end
                // ---------------- ACK after address -------------------
                S_ACK_A: if (scl_fall) begin
                    bitcnt <= 4'd0;
                    if (rw) begin                    // master reads: send mem[ptr]
                        sr     <= mem[ptr];
                        sda_oe <= ~mem[ptr][7];
                        state  <= S_TX;
                    end else begin                   // master writes
                        sda_oe <= 1'b0;
                        first  <= 1'b1;
                        state  <= S_RX;
                    end
                end
                // ---------------- receive data byte -------------------
                S_RX: begin
                    if (scl_rise) begin
                        sr     <= {sr[6:0], sda_s};
                        bitcnt <= bitcnt + 1'b1;
                    end
                    if (scl_fall && bitcnt == 4'd8) begin
                        sda_oe <= 1'b1;              // ACK
                        state  <= S_ACK_RX;
                        if (first) begin
                            ptr   <= sr[3:0];        // register pointer
                            first <= 1'b0;
                        end else begin
                            mem[ptr] <= sr;          // store data
                            ptr      <= ptr + 1'b1;
                            rx_data  <= sr;
                            rx_valid <= 1'b1;
                        end
                    end
                end
                S_ACK_RX: if (scl_fall) begin
                    sda_oe <= 1'b0;                  // release after ACK
                    bitcnt <= 4'd0;
                    state  <= S_RX;
                end
                // ---------------- transmit data byte ------------------
                S_TX: if (scl_fall) begin
                    bitcnt <= bitcnt + 1'b1;
                    if (bitcnt == 4'd7) begin
                        sda_oe <= 1'b0;              // release for master ACK
                        state  <= S_ACK_TX;
                    end else begin
                        sr     <= {sr[6:0], 1'b0};
                        sda_oe <= ~sr[6];            // next bit
                    end
                end
                S_ACK_TX: begin
                    if (scl_rise) m_nack <= sda_s;   // SDA high = NACK
                    if (scl_fall) begin
                        ptr <= ptr + 1'b1;
                        bitcnt <= 4'd0;
                        if (m_nack) begin
                            sda_oe <= 1'b0;
                            state  <= S_IDLE;        // wait for STOP / Sr
                        end else begin
                            sr     <= mem[ptr + 1'b1];
                            sda_oe <= ~mem[ptr + 1'b1][7];
                            state  <= S_TX;
                        end
                    end
                end
                default: state <= S_IDLE;
                endcase
            end
        end
    end
endmodule
