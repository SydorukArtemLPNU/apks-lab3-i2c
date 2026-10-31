// ============================================================================
// i2c_master.v — I2C master (transmitter side), byte-level command interface
// ----------------------------------------------------------------------------
// Lab work #3, variant 20: I2C, Verilog.
//
// The core executes one bus command at a time:
//   CMD_START — START condition (or repeated START if the bus is already busy)
//   CMD_WRITE — shift out `din` MSB first, then sample the ACK bit from slave
//   CMD_READ  — shift in 8 bits into `dout`, then send ACK (read_nack = 0)
//               or NACK (read_nack = 1, used for the last byte of a read)
//   CMD_STOP  — STOP condition, bus becomes free
//
// SCL generation: every bit is split into 4 quarter periods ("phases"):
//   phase 0: SCL low,  master puts the next data bit on SDA
//   phase 1: SCL low,  master releases SCL (it rises through the pull-up)
//   phase 2: SCL high, master samples SDA (read data / ACK)
//            — if SCL is still low here, a slave is stretching the clock and
//              the core waits (clock stretching support)
//   phase 3: SCL high, master pulls SCL low again (end of bit)
// f_SCL = CLK_HZ / (4 * QDIV), QDIV is rounded UP so that f_SCL never exceeds
// the requested I2C_HZ (12 MHz / 400 kHz -> QDIV = 8 -> f_SCL = 375 kHz).
//
// Open-drain outputs: *_oe = 1 pulls the line low, *_oe = 0 releases it
// (external pull-up makes it high). Board level: sda = sda_oe ? 1'b0 : 1'bz.
// ============================================================================
`timescale 1ns / 1ps
module i2c_master #(
    parameter integer CLK_HZ = 12_000_000,  // system clock frequency
    parameter integer I2C_HZ = 400_000      // target SCL frequency (Fast-mode)
) (
    input  wire       clk,        // system clock
    input  wire       rst_n,      // asynchronous reset, active low

    // ---- command interface -------------------------------------------------
    input  wire       cmd_valid,  // pulse/level: command present on `cmd`
    output wire       cmd_ready,  // core is idle and accepts a new command
    input  wire [2:0] cmd,        // CMD_* code, see localparams below
    input  wire [7:0] din,        // byte to transmit for CMD_WRITE
    input  wire       read_nack,  // CMD_READ: 1 = answer NACK (last byte)
    output reg  [7:0] dout,       // byte received by CMD_READ
    output reg        ack,        // CMD_WRITE: 1 = slave answered ACK
    output reg        done,       // one-clock pulse: command finished
    output reg        bus_busy,   // between START and STOP

    // ---- I2C bus (open drain) ---------------------------------------------
    input  wire       scl_i,      // SCL line level
    output reg        scl_oe,     // 1 = drive SCL low
    input  wire       sda_i,      // SDA line level
    output reg        sda_oe      // 1 = drive SDA low
);
    // command codes
    localparam [2:0] CMD_START = 3'd1,
                     CMD_WRITE = 3'd2,
                     CMD_READ  = 3'd3,
                     CMD_STOP  = 3'd4;

    // FSM states
    localparam [2:0] S_IDLE  = 3'd0,  // waiting for a command
                     S_START = 3'd1,  // generating (repeated) START
                     S_DATA  = 3'd2,  // 8 data bits (write or read)
                     S_ACK   = 3'd3,  // 9th bit: ACK/NACK
                     S_STOP  = 3'd4;  // generating STOP

    // quarter-period divider, rounded up
    localparam integer QDIV = (CLK_HZ + 4 * I2C_HZ - 1) / (4 * I2C_HZ);
    localparam integer CW   = (QDIV > 1) ? $clog2(QDIV) : 1;

    reg [2:0]    state;
    reg [1:0]    phase;      // quarter of the current bit
    reg [2:0]    bitcnt;     // data bit counter 0..7
    reg [7:0]    sr;         // shift register (TX data / RX data)
    reg          is_read;    // current byte is a read
    reg          nack_rd;    // send NACK after this read byte
    reg [CW-1:0] qcnt;       // divider counter

    // two-flop synchronisers for the bus inputs
    reg [1:0] scl_sync, sda_sync;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            scl_sync <= 2'b11;
            sda_sync <= 2'b11;
        end else begin
            scl_sync <= {scl_sync[0], scl_i};
            sda_sync <= {sda_sync[0], sda_i};
        end
    wire scl_s = scl_sync[1];
    wire sda_s = sda_sync[1];

    // quarter tick; frozen in phase 2 while a slave holds SCL low (stretching)
    wire stretch = (phase == 2'd2) && !scl_oe && !scl_s;
    wire tick    = (qcnt == 0) && !stretch;

    assign cmd_ready = (state == S_IDLE) && !done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            phase    <= 2'd0;
            bitcnt   <= 3'd0;
            sr       <= 8'h00;
            is_read  <= 1'b0;
            nack_rd  <= 1'b0;
            qcnt     <= 0;
            dout     <= 8'h00;
            ack      <= 1'b0;
            done     <= 1'b0;
            bus_busy <= 1'b0;
            scl_oe   <= 1'b0;   // bus released
            sda_oe   <= 1'b0;
        end else begin
            done <= 1'b0;

            // ---------------- divider ----------------
            if (state == S_IDLE || stretch)
                qcnt <= QDIV - 1;
            else if (qcnt == 0)
                qcnt <= QDIV - 1;
            else
                qcnt <= qcnt - 1'b1;

            case (state)
            // -------------------------------------------------------------
            S_IDLE: begin
                phase <= 2'd0;
                if (cmd_valid && cmd_ready) begin
                    case (cmd)
                    CMD_START: state <= S_START;
                    CMD_WRITE: begin
                        state <= S_DATA; sr <= din; is_read <= 1'b0;
                        bitcnt <= 3'd0;
                    end
                    CMD_READ: begin
                        state <= S_DATA; is_read <= 1'b1; nack_rd <= read_nack;
                        bitcnt <= 3'd0;
                    end
                    CMD_STOP: state <= S_STOP;
                    default: done <= 1'b1;  // unknown command: ignore
                    endcase
                end
            end
            // ------------- START / repeated START ------------------------
            S_START: if (tick) begin
                phase <= phase + 1'b1;
                case (phase)
                2'd0: sda_oe <= 1'b0;          // SDA high (needed for Sr)
                2'd1: scl_oe <= 1'b0;          // SCL high
                2'd2: sda_oe <= 1'b1;          // SDA falls while SCL high
                2'd3: begin                    // SCL low -> START done
                    scl_oe   <= 1'b1;
                    bus_busy <= 1'b1;
                    state    <= S_IDLE;
                    done     <= 1'b1;
                end
                endcase
            end
            // ------------- 8 data bits ------------------------------------
            S_DATA: if (tick) begin
                phase <= phase + 1'b1;
                case (phase)
                2'd0: sda_oe <= is_read ? 1'b0 : ~sr[7];   // drive bit / release
                2'd1: scl_oe <= 1'b0;                      // rising edge of SCL
                2'd2: if (is_read) sr <= {sr[6:0], sda_s}; // sample while SCL high
                2'd3: begin
                    scl_oe <= 1'b1;                        // falling edge
                    if (!is_read) sr <= {sr[6:0], 1'b0};
                    bitcnt <= bitcnt + 1'b1;
                    if (bitcnt == 3'd7) state <= S_ACK;
                end
                endcase
            end
            // ------------- ACK bit ----------------------------------------
            S_ACK: if (tick) begin
                phase <= phase + 1'b1;
                case (phase)
                2'd0: sda_oe <= is_read ? ~nack_rd : 1'b0; // master ACK / release
                2'd1: scl_oe <= 1'b0;
                2'd2: if (!is_read) ack <= ~sda_s;         // slave ACK = SDA low
                2'd3: begin
                    scl_oe <= 1'b1;
                    if (is_read) dout <= sr;
                    state <= S_IDLE;
                    done  <= 1'b1;
                end
                endcase
            end
            // ------------- STOP -------------------------------------------
            S_STOP: if (tick) begin
                phase <= phase + 1'b1;
                case (phase)
                2'd0: sda_oe <= 1'b1;          // SDA low while SCL low
                2'd1: scl_oe <= 1'b0;          // SCL high
                2'd2: sda_oe <= 1'b0;          // SDA rises while SCL high
                2'd3: begin
                    bus_busy <= 1'b0;
                    state    <= S_IDLE;
                    done     <= 1'b1;
                end
                endcase
            end
            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
