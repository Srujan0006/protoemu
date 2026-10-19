// Tiny Tapeout top level of the DTC: serial config port (dtc_spi) + whole chip (dtc_top).
//
// Pins (D2):  ui_in[0] CS_N   ui_in[1] SCK   ui_in[2] MOSI          (config port, SCK <= clk/8, A13)
//             uo_out[0] MISO  uo_out[1] interrupt  uo_out[2] TRIG_OUT
//             uio[7:0] the 8 protocol pads (uio_oe = pad_oe)
// The co-simulation buses of dtc_top (dbg, dbg_top) are left unconnected here, so synthesis removes them (CLAUDE.md rule 5).
`default_nettype none

module tt_um_srujan0006_dtc #(
    parameter NUM_SM = 2
) (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    wire        h_wr, h_rd, frame_err, miso, irq_out, trig_out;
    wire [7:0]  h_addr;
    wire [31:0] h_wdata, h_rdata;

    dtc_spi u_spi (
        .clk(clk), .rst_n(rst_n),
        .cs_n(ui_in[0]), .sck(ui_in[1]), .mosi(ui_in[2]), .miso(miso),
        .h_wr(h_wr), .h_rd(h_rd), .h_addr(h_addr), .h_wdata(h_wdata), .h_rdata(h_rdata),
        .frame_err(frame_err)
    );

    dtc_top #(.NUM_SM(NUM_SM)) u_top (
        .clk(clk), .rst_n(rst_n),
        .pad_in(uio_in), .pad_out(uio_out), .pad_oe(uio_oe),
        .trig_out(trig_out), .irq_out(irq_out),
        .h_wr(h_wr), .h_rd(h_rd), .h_addr(h_addr), .h_wdata(h_wdata), .h_rdata(h_rdata),
        .frame_err(frame_err),
        .dbg(), .dbg_top(), .trg_state(), .cap_count()
    );

    assign uo_out = {5'b0, trig_out, irq_out, miso};

    wire _unused = &{ena, ui_in[7:3], 1'b0};
endmodule

`default_nettype wire
