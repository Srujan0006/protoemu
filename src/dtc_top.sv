// DTC chip top level, level 2 (L2): core + Capture Unit + Trigger Unit.  RTL spike 4.
//
// Wiring only; every block was verified on its own against dtc_model.py.  This file adds:
//   * host-address routing for the two units  (0x10..0x13 Capture, 0x20..0x25 Trigger)
//   * the STATUS bits and sticky-flag pulses the units contribute (through the seam of dtc_core_top)
//   * the pin path:  machines -> shared pio_out / pio_oe -> TRIGGER OVERRIDE -> pad   (D23: the Trigger beats both machines)
//
// The host port is the parallel word interface of the model (one read or write per cycle).  The serial SPI config port
// (dtc_spi.sv) is the front end that produces exactly these signals; the chip top tt_um_srujan0006_dtc.sv connects them.
module dtc_top #(
    parameter NUM_SM = 1,
    parameter DBGW   = 180
) (
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire [7:0]             pad_in,
    output wire [7:0]             pad_out,
    output wire [7:0]             pad_oe,
    output wire                   trig_out,
    output wire                   irq_out,
    // host port
    input  wire                   h_wr,
    input  wire                   h_rd,
    input  wire [7:0]             h_addr,
    input  wire [31:0]            h_wdata,
    output wire [31:0]            h_rdata,
    input  wire                   frame_err,      // pulse from the serial config port (dtc_spi): STATUS.FRAME_ERR
    // co-simulation visibility
    output wire [NUM_SM*DBGW-1:0] dbg,
    output wire [51:0]            dbg_top,
    output wire [1:0]             trg_state,
    output wire [4:0]             cap_count
);
    wire        ext_wr, ext_rd, cap_en, trig_en, cap_clr;
    wire [7:0]  pin_sync, pio_out, pio_oe;
    wire [31:0] ext_rdata, ext_status, ext_flag_set;

    dtc_core_top #(.NUM_SM(NUM_SM), .DBGW(DBGW)) u_core (
        .clk(clk), .rst_n(rst_n), .pad_in(pad_in),
        .h_wr(h_wr), .h_rd(h_rd), .h_addr(h_addr), .h_wdata(h_wdata), .h_rdata(h_rdata),
        .ext_rdata(ext_rdata), .ext_status(ext_status), .ext_flag_set(ext_flag_set),
        .ext_wr(ext_wr), .ext_rd(ext_rd), .cap_en(cap_en), .trig_en(trig_en), .cap_clr(cap_clr),
        .pin_sync(pin_sync), .pio_out(pio_out), .pio_oe(pio_oe), .irq_out(irq_out),
        .dbg(dbg), .dbg_top(dbg_top)
    );

    // ---------------------------------------------------------------- Capture Unit  (0x10..0x13)
    wire        cap_sel_wr = ext_wr && (h_addr == 8'h10);
    wire        cap_sel_rd = ext_rd && (h_addr[7:4] == 4'h1);
    wire [31:0] cap_rdata;
    wire        cap_full, cap_empty, cap_ovf_pulse;
    wire [31:0] cycle_cnt;

    dtc_capture u_cap (
        .clk(clk), .rst_n(rst_n), .pin_sync(pin_sync),
        .cap_en(cap_en), .cap_clr(cap_clr),
        .wr_en(cap_sel_wr), .wr_data(h_wdata[7:0]),
        .rd_en(cap_sel_rd), .rd_sel(h_addr[1:0]), .rd_data(cap_rdata),
        .cap_count(cap_count), .cap_full(cap_full), .cap_empty(cap_empty),
        .ovf_pulse(cap_ovf_pulse), .cycle_cnt(cycle_cnt)
    );

    // ---------------------------------------------------------------- Trigger Unit  (0x20..0x25) and the pin override
    wire        trg_sel_wr = ext_wr && (h_addr[7:4] == 4'h2);
    wire [31:0] trg_rdata;
    wire        fired_pulse, trg_cmd_err;

    dtc_trigger u_trg (
        .clk(clk), .rst_n(rst_n), .pin_sync(pin_sync), .trig_en(trig_en),
        .wr_en(trg_sel_wr), .wr_sel(h_addr[2:0]), .wr_data(h_wdata),
        .rd_sel(h_addr[2:0]), .rd_data(trg_rdata),
        .pio_out(pio_out), .pio_oe(pio_oe), .pad_out(pad_out), .pad_oe(pad_oe), .trig_out(trig_out),
        .trg_state(trg_state), .fired_pulse(fired_pulse), .cmd_err_pulse(trg_cmd_err)
    );

    // ---------------------------------------------------------------- seam back into the core
    assign ext_rdata    = (h_addr[7:4] == 4'h1) ? cap_rdata : trg_rdata;
    assign ext_status   = {11'b0, cap_count, trg_state, cap_empty, cap_full, 12'b0};     // [20:16] [15:14] [13] [12]
    assign ext_flag_set = (cap_ovf_pulse ? 32'h8000_0000 : 32'b0) |      // CAP_OVF   bit 31
                          (fired_pulse   ? 32'h4000_0000 : 32'b0) |      // TRG_FIRED bit 30
                          (frame_err     ? 32'h0100_0000 : 32'b0) |      // FRAME_ERR bit 24
                          (trg_cmd_err   ? 32'h0400_0000 : 32'b0);       // CMD_ERR   bit 26
endmodule
