// DTC core wrapper, level 1 (L1): everything the state machines share.  RTL spike 3.
//
// Contains : IMEM (64 x 16 flops, not reset: D13), the 2-FF input synchronizer, the shared pin registers pio_out / pio_oe,
//            the shared IRQ flags and IRQ_EN, CTRL (SM_EN per machine, CAP_EN, TRIG_EN), the shared sticky flags,
//            the host-address decode, and NUM_SM copies of dtc_sm.
// Does NOT contain: the Capture Unit, the Trigger Unit (and its pin override) or the SPI config port.  They attach through the seam:
//     host addresses 0x10..0x13 and 0x20..0x25 are "external": this module raises ext_wr / ext_rd and returns ext_rdata;
//     ext_status supplies the Capture / Trigger bits of STATUS (bits 12..13 full / empty, 15:14 TRG_STATE, 20:16 CAP count);
//     ext_flag_set supplies the CAP_OVF / TRG_FIRED / CMD_ERR set pulses;
//     cap_en, trig_en and cap_clr are the CTRL bits those units need.
// pio_out / pio_oe are the shared registers BEFORE the Trigger override; the Trigger mux sits after them (D23).
//
// Pin write rule (D23): each machine exports per-pin write masks; SM0 is applied first and SM1 after it, so SM1 wins a
// same-edge write to the same pin.  A pin that no machine writes keeps its value.
//
// dbg_top layout (MSB..LSB): en[1] en[0] cap_en trig_en irq_en[7:0] irq_flags[7:0] flags[31:0]   (52 bits, co-simulation only)
module dtc_core_top #(
    parameter NUM_SM = 1,                // 1 or 2
    parameter DBGW   = 180
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [7:0]            pad_in,
    // host port: one operation per cycle
    input  wire                  h_wr,
    input  wire                  h_rd,
    input  wire [7:0]            h_addr,
    input  wire [31:0]           h_wdata,
    output reg  [31:0]           h_rdata,
    // seam to the other units
    input  wire [31:0]           ext_rdata,
    input  wire [31:0]           ext_status,
    input  wire [31:0]           ext_flag_set,
    output wire                  ext_wr,
    output wire                  ext_rd,
    output wire                  cap_en,
    output wire                  trig_en,
    output wire                  cap_clr,
    // pins
    output wire [7:0]            pin_sync,
    output wire [7:0]            pio_out,
    output wire [7:0]            pio_oe,
    output wire                  irq_out,
    // co-simulation visibility
    output wire [NUM_SM*DBGW-1:0] dbg,
    output wire [51:0]           dbg_top
);
    localparam [31:0] F_ADDR_ERR = 32'h0200_0000;   // bit 25
    localparam [31:0] F_CMD_ERR  = 32'h0400_0000;   // bit 26
    localparam [31:0] F_IMEM_ERR = 32'h0800_0000;   // bit 27
    localparam [31:0] SHARED_FLAGS = 32'hCF00_0000; // bits 24..27, 30, 31

    // ------------------------------------------------------------------ input synchronizer (2 FF)
    reg [7:0] sync1_q, sync2_q;
    always @(posedge clk) begin
        if (!rst_n) begin sync1_q <= 8'b0; sync2_q <= 8'b0; end
        else        begin sync1_q <= pad_in; sync2_q <= sync1_q; end
    end
    assign pin_sync = sync2_q;

    // ------------------------------------------------------------------ host address decode
    wire [7:0] a = h_addr;
    wire is_id    = (a == 8'h00);
    wire is_ctrl  = (a == 8'h01);
    wire is_stat0 = (a == 8'h02);
    wire is_stat1 = (a == 8'h03) && (NUM_SM > 1);
    wire is_irqf  = (a == 8'h0C);
    wire is_irqen = (a == 8'h0D);
    wire is_imem  = (a[7:6] == 2'b10);
    wire ext_w_ok = (a == 8'h10) || (a >= 8'h20 && a <= 8'h25);
    wire ext_r_ok = (a >= 8'h10 && a <= 8'h13) || (a >= 8'h20 && a <= 8'h25);
    wire [3:0] lo = a[3:0];
    wire lo_w_ok = (lo >= 4'd4 && lo <= 4'd8) || lo == 4'd10 || lo == 4'd14;
    wire lo_r_ok = (lo >= 4'd4 && lo <= 4'd7) || lo == 4'd9 || lo == 4'd11 || lo == 4'd14;

    wire [NUM_SM-1:0] sm_win;
    genvar gi;
    generate for (gi = 0; gi < NUM_SM; gi = gi + 1) begin : g_win
        localparam [3:0] WIN = gi * 4;                      // SM0 window 0x0X, SM1 window 0x4X
        assign sm_win[gi] = (a[7:4] == WIN);
    end endgenerate
    wire sm_reg_w = |sm_win && lo_w_ok;
    wire sm_reg_r = |sm_win && lo_r_ok;

    assign ext_wr = h_wr && ext_w_ok;
    assign ext_rd = h_rd && ext_r_ok;

    wire valid_w = is_ctrl || is_stat0 || is_stat1 || sm_reg_w || is_irqf || is_irqen || ext_w_ok || is_imem;
    wire valid_r = is_id || is_ctrl || is_stat0 || is_stat1 || sm_reg_r || is_irqf || is_irqen || ext_r_ok || is_imem;

    // ------------------------------------------------------------------ state
    reg [NUM_SM-1:0] en_q;
    reg              cap_en_q, trig_en_q;
    reg [31:0]       flags_q;                    // shared sticky flags only
    reg [7:0]        irq_flags_q, irq_en_q;
    reg [7:0]        pio_out_q, pio_oe_q;
    reg [15:0]       imem [0:63];

    wire locked = |en_q;                         // D26: host IMEM access needs every machine stopped
    assign cap_en  = cap_en_q;
    assign trig_en = trig_en_q;

    wire ctrl_wr = h_wr && is_ctrl;
    assign cap_clr = ctrl_wr && h_wdata[6];

    // ------------------------------------------------------------------ machines
    wire [NUM_SM*32-1:0] sm_hrdata, sm_status;
    wire [NUM_SM-1:0]    sm_cmd_err;
    wire [NUM_SM*8-1:0]  sm_out_we, sm_out_val, sm_oe_we, sm_oe_val, sm_irq_set, sm_irq_clr;
    wire [NUM_SM*6-1:0]  sm_raddr;
    reg  [NUM_SM*16-1:0] sm_rdata;
    wire [NUM_SM*DBGW-1:0] sm_dbg;

    generate for (gi = 0; gi < NUM_SM; gi = gi + 1) begin : g_sm
        localparam integer RS_B = (gi == 0) ? 3 : 9;
        localparam integer TC_B = (gi == 0) ? 4 : 10;
        localparam integer RC_B = (gi == 0) ? 5 : 11;
        localparam integer DR_B = (gi == 0) ? 7 : 12;

        wire sel_w = h_wr && sm_win[gi] && lo_w_ok;
        wire sel_r = h_rd && sm_win[gi] && lo_r_ok;
        wire stat_w = h_wr && (a == (gi == 0 ? 8'h02 : 8'h03));

        dtc_sm #(.DBGW(DBGW)) u_sm (
            .clk(clk), .rst_n(rst_n),
            .en(en_q[gi]),
            .restart    (ctrl_wr && h_wdata[RS_B]),
            .div_restart(ctrl_wr && h_wdata[DR_B]),
            .tx_clr     (ctrl_wr && h_wdata[TC_B]),
            .rx_clr     (ctrl_wr && h_wdata[RC_B]),
            .h_wr(sel_w), .h_rd(sel_r), .h_sel(a[3:0]), .h_wdata(h_wdata),
            .h_rdata(sm_hrdata[gi*32 +: 32]),
            .flag_w1c(stat_w ? h_wdata : 32'b0),
            .status(sm_status[gi*32 +: 32]),
            .cmd_err_pulse(sm_cmd_err[gi]),
            .pin_sync(sync2_q),
            .pin_out_we(sm_out_we[gi*8 +: 8]), .pin_out_val(sm_out_val[gi*8 +: 8]),
            .pin_oe_we(sm_oe_we[gi*8 +: 8]),   .pin_oe_val(sm_oe_val[gi*8 +: 8]),
            .imem_raddr(sm_raddr[gi*6 +: 6]), .imem_rdata(sm_rdata[gi*16 +: 16]),
            .irq_flags(irq_flags_q),
            .irq_set(sm_irq_set[gi*8 +: 8]), .irq_clr(sm_irq_clr[gi*8 +: 8]),
            .dbg(sm_dbg[gi*DBGW +: DBGW])
        );
    end endgenerate
    assign dbg = sm_dbg;

    // IMEM read ports (one per machine)
    integer ri;
    always @* begin
        for (ri = 0; ri < NUM_SM; ri = ri + 1) sm_rdata[ri*16 +: 16] = imem[sm_raddr[ri*6 +: 6]];
    end

    // ------------------------------------------------------------------ shared pin registers and IRQ flags
    reg [7:0] po_n, pe_n, irq_set_all, irq_clr_all;
    integer pi;
    always @* begin
        po_n = pio_out_q;  pe_n = pio_oe_q;  irq_set_all = 8'b0;  irq_clr_all = 8'b0;
        for (pi = 0; pi < NUM_SM; pi = pi + 1) begin           // machine order: SM1 is applied last and wins (D23)
            po_n = (po_n & ~sm_out_we[pi*8 +: 8]) | (sm_out_val[pi*8 +: 8] & sm_out_we[pi*8 +: 8]);
            pe_n = (pe_n & ~sm_oe_we[pi*8 +: 8])  | (sm_oe_val[pi*8 +: 8]  & sm_oe_we[pi*8 +: 8]);
            irq_set_all = irq_set_all | sm_irq_set[pi*8 +: 8];
            irq_clr_all = irq_clr_all | sm_irq_clr[pi*8 +: 8];
        end
    end
    assign pio_out = pio_out_q;
    assign pio_oe  = pio_oe_q;

    // ------------------------------------------------------------------ shared flags
    wire addr_err = (h_wr && !valid_w) || (h_rd && !valid_r);
    wire imem_err = (h_wr || h_rd) && is_imem && locked;
    wire [31:0] setf = (addr_err ? F_ADDR_ERR : 32'b0) | ((|sm_cmd_err) ? F_CMD_ERR : 32'b0) |
                       (imem_err ? F_IMEM_ERR : 32'b0) | ext_flag_set;
    wire [31:0] w1c  = (h_wr && is_stat0) ? (h_wdata & SHARED_FLAGS) : 32'b0;
    wire [7:0]  irq_w1c = (h_wr && is_irqf) ? h_wdata[7:0] : 8'b0;

    // ------------------------------------------------------------------ sequential
    always @(posedge clk) begin
        if (!rst_n) begin
            en_q <= {NUM_SM{1'b0}};
            cap_en_q <= 1'b0;  trig_en_q <= 1'b0;
            flags_q <= 32'b0;  irq_flags_q <= 8'b0;  irq_en_q <= 8'b0;
            pio_out_q <= 8'b0;  pio_oe_q <= 8'b0;
        end else begin
            if (ctrl_wr) begin
                cap_en_q <= h_wdata[1];
                trig_en_q <= h_wdata[2];
                en_q[0] <= h_wdata[0];
                if (NUM_SM > 1) en_q[NUM_SM-1] <= h_wdata[8];
            end
            flags_q     <= ((flags_q & ~w1c) | setf) & SHARED_FLAGS;
            irq_flags_q <= (irq_flags_q & ~(irq_w1c | irq_clr_all)) | irq_set_all;     // set wins
            if (h_wr && is_irqen) irq_en_q <= h_wdata[7:0];
            pio_out_q <= po_n;
            pio_oe_q  <= pe_n;
        end
        // IMEM: host writes only while every machine is stopped (D12 / D26); never reset (D13)
        if (h_wr && is_imem && !locked) imem[a[5:0]] <= h_wdata[15:0];
    end

    // ------------------------------------------------------------------ host reads
    wire [31:0] st0 = sm_status[31:0];
    wire [31:0] st1 = (NUM_SM > 1) ? sm_status[(NUM_SM-1)*32 +: 32] : 32'b0;
    wire [31:0] smr = sm_hrdata[(a[6] && NUM_SM > 1 ? NUM_SM - 1 : 0) * 32 +: 32];
    always @* begin
        h_rdata = 32'b0;
        if (is_id)                 h_rdata = {16'hD7C0, 8'(NUM_SM), 8'h01};
        else if (is_ctrl)          h_rdata = {23'b0, (NUM_SM > 1) ? en_q[NUM_SM-1] : 1'b0, 5'b0, trig_en_q, cap_en_q, en_q[0]};
        else if (is_stat0)         h_rdata = st0 | ext_status | flags_q;
        else if (is_stat1)         h_rdata = st1;
        else if (sm_reg_r)         h_rdata = smr;
        else if (is_irqf)          h_rdata = {24'b0, irq_flags_q};
        else if (is_irqen)         h_rdata = {24'b0, irq_en_q};
        else if (ext_r_ok)         h_rdata = ext_rdata;
        else if (is_imem && !locked) h_rdata = {16'b0, imem[a[5:0]]};
    end

    // IRQ output: IRQ_EN & {any IRQ flag, TRG_FIRED, CAP_OVF, capture FIFO non-empty}
    assign irq_out = |(irq_en_q[3:0] & {(irq_flags_q != 8'b0), flags_q[30], flags_q[31], !ext_status[13]});

    // ------------------------------------------------------------------ co-simulation visibility
    wire [1:0] en_v = {(NUM_SM > 1) ? en_q[NUM_SM-1] : 1'b0, en_q[0]};
    assign dbg_top = {en_v[1], en_v[0], cap_en_q, trig_en_q, irq_en_q, irq_flags_q, flags_q};
endmodule
