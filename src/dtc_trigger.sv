// -----------------------------------------------------------------------------
// dtc_trigger.sv : DTC Autonomous Trigger Unit (RTL spike)
//
// Cycle-accurate to dtc_model.py  DtcModel._trigger() and the TRG_* host writes of _hwrite()
// (decisions D6-D8, A1, A3, A8).  Written in a Verilog-2001 style subset of SystemVerilog so that
// Icarus Verilog, Verilator and Yosys/SymbiYosys all accept it.
//
// Timing contract (A1): if the external pin changes in cycle n, pin_sync shows it in cycle n+2 (the
// 2-FF synchronizer is OUTSIDE this module), the match cycle is m = n+2, and `override` is high in
// cycles m+D+1 .. m+D+H  (H = max(HOLD,1)),  i.e. LAT_TOTAL = D + 2 clock edges.
//
// Reset is synchronous, active low.  All state is one-hot / plain flops.
// -----------------------------------------------------------------------------
module dtc_trigger (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  pin_sync,        // synchronized pins
    input  wire        trig_en,         // CTRL.TRIG_EN (a flop outside this module)

    // host register port.  sel: 0 TRG_CTRL(0x20) 1 MATCH_MASK(0x21) 2 MATCH_VAL(0x22)
    //                          3 DELAY(0x23)    4 ACT_MASK(0x24)   5 ACT_VAL(0x25)
    input  wire        wr_en,
    input  wire [2:0]  wr_sel,
    input  wire [31:0] wr_data,
    input  wire [2:0]  rd_sel,
    output reg  [31:0] rd_data,

    // pin path: the override sits AFTER the state machines' pin registers
    input  wire [7:0]  pio_out,
    input  wire [7:0]  pio_oe,
    output wire [7:0]  pad_out,
    output wire [7:0]  pad_oe,
    output wire        trig_out,        // = override (scope trigger pulse)

    // status
    output wire [1:0]  trg_state,       // 0 DISARMED, 1 ARMED, 2 COUNTING, 3 FIRING
    output wire        fired_pulse,     // 1 cycle when a firing completes   (sets STATUS.TRG_FIRED)
    output wire        cmd_err_pulse    // 1 cycle when a write is rejected   (sets STATUS.CMD_ERR)
);

    // ---------------------------------------------------------------- state
    reg         st_dis, st_armed, st_cnt, st_fire;      // one-hot FSM
    reg         rearm;
    reg [15:0]  hold_cfg;
    reg [7:0]   match_mask, match_val;
    reg [31:0]  delay_cfg;
    reg [7:0]   act_mask, act_val;
    reg [31:0]  trg_cnt;
    reg [15:0]  trg_hold;
    reg [7:0]   act_mask_s, act_val_s;                  // snapshot taken at the match
    reg         match_prev;

    // ---------------------------------------------------------------- combinational
    // MATCH_MASK = 0 never matches.  The comparison is on the masked bits only.
    wire        match_now  = (match_mask != 8'h00) &&
                             ((pin_sync & match_mask) == (match_val & match_mask));
    wire        match_edge = match_now && !match_prev;  // fire on the transition INTO a match (D6)

    wire [15:0] hold_m1    = (hold_cfg == 16'h0000) ? 16'h0000 : (hold_cfg - 16'h0001);   // max(H,1)-1

    wire        is_trg_wr  = wr_en && (wr_sel <= 3'd5);
    wire        is_ctrl_wr = wr_en && (wr_sel == 3'd0);
    wire        arm_bit    = is_ctrl_wr && wr_data[0];
    wire        arm_strobe = arm_bit && st_dis && trig_en;                   // A8: ARMED at this same edge

    // A3: a rejected write (any TRG_* register while not DISARMED, or ARM while TRIG_EN=0)
    assign cmd_err_pulse = (is_trg_wr && !st_dis) || (arm_bit && st_dis && !trig_en);

    wire        override   = st_fire && trig_en;       // aborts the cycle TRIG_EN drops (D8)

    assign pad_out  = override ? ((pio_out & ~act_mask_s) | (act_val_s & act_mask_s)) : pio_out;
    assign pad_oe   = override ? (pio_oe | act_mask_s) : pio_oe;
    assign trig_out = override;

    assign trg_state   = st_armed ? 2'd1 : st_cnt ? 2'd2 : st_fire ? 2'd3 : 2'd0;
    assign fired_pulse = rst_n && trig_en && st_fire && (trg_hold == 16'h0000);

    always @* begin
        case (rd_sel)
            3'd0:    rd_data = {hold_cfg, 14'b0, rearm, 1'b0};   // ARM reads as 0
            3'd1:    rd_data = {24'b0, match_mask};
            3'd2:    rd_data = {24'b0, match_val};
            3'd3:    rd_data = delay_cfg;
            3'd4:    rd_data = {24'b0, act_mask};
            3'd5:    rd_data = {24'b0, act_val};
            default: rd_data = 32'h0000_0000;
        endcase
    end

    // ---------------------------------------------------------------- sequential
    always @(posedge clk) begin
        if (!rst_n) begin
            st_dis     <= 1'b1;
            st_armed   <= 1'b0;
            st_cnt     <= 1'b0;
            st_fire    <= 1'b0;
            rearm      <= 1'b0;
            hold_cfg   <= 16'd1;
            match_mask <= 8'h00;
            match_val  <= 8'h00;
            delay_cfg  <= 32'd0;
            act_mask   <= 8'h00;
            act_val    <= 8'h00;
            trg_cnt    <= 32'd0;
            trg_hold   <= 16'd0;
            act_mask_s <= 8'h00;
            act_val_s  <= 8'h00;
            match_prev <= 1'b0;
        end else begin
            match_prev <= match_now;                    // tracked every cycle, in every state

            // host configuration: writable ONLY while DISARMED (else rejected, see cmd_err_pulse)
            if (wr_en && st_dis) begin
                case (wr_sel)
                    3'd0: begin rearm <= wr_data[1]; hold_cfg <= wr_data[31:16]; end
                    3'd1: match_mask <= wr_data[7:0];
                    3'd2: match_val  <= wr_data[7:0];
                    3'd3: delay_cfg  <= wr_data;
                    3'd4: act_mask   <= wr_data[7:0];
                    3'd5: act_val    <= wr_data[7:0];
                    default: ;
                endcase
            end

            // state machine
            if (!trig_en) begin                         // abort wins over everything (D8)
                st_dis   <= 1'b1;
                st_armed <= 1'b0;
                st_cnt   <= 1'b0;
                st_fire  <= 1'b0;
            end else if (st_dis) begin
                if (arm_strobe) begin
                    st_dis   <= 1'b0;
                    st_armed <= 1'b1;
                end
            end else if (st_armed) begin
                if (match_edge) begin
                    act_mask_s <= act_mask;             // snapshot: later host writes cannot change a firing
                    act_val_s  <= act_val;
                    st_armed   <= 1'b0;
                    if (delay_cfg == 32'd0) begin
                        st_fire  <= 1'b1;
                        trg_hold <= hold_m1;
                    end else begin
                        st_cnt   <= 1'b1;
                        trg_cnt  <= delay_cfg - 32'd1;
                    end
                end
            end else if (st_cnt) begin
                if (trg_cnt != 32'd0) begin
                    trg_cnt <= trg_cnt - 32'd1;
                end else begin
                    st_cnt   <= 1'b0;
                    st_fire  <= 1'b1;
                    trg_hold <= hold_m1;
                end
            end else begin                              // FIRING
                if (trg_hold != 16'd0) begin
                    trg_hold <= trg_hold - 16'd1;
                end else begin
                    st_fire <= 1'b0;
                    if (rearm) st_armed <= 1'b1;        // re-arm: needs a NEW match edge to fire again
                    else       st_dis   <= 1'b1;        // one-shot
                end
            end
        end
    end

`ifdef FORMAL
    // =========================================================================
    // Formal properties P1-P7 (Step 5.9).  Clocked immediate assertions, checked against an
    // INDEPENDENT shadow counter (chk_*) so a counter bug in the design cannot hide itself.
    //   cd formal
    //   sby -f dtc_trigger.sby bmc      (-D BOUNDED: DELAY <= 6, HOLD <= 4, depth 40)
    //   sby -f dtc_trigger.sby cover
    //   sby -f dtc_trigger.sby prove    (k-induction, NO bounds on DELAY/HOLD, helper invariants below)
    // =========================================================================
    reg f_past_valid = 1'b0;
    always @(posedge clk) f_past_valid <= 1'b1;
    always @* if (!f_past_valid) assume (!rst_n);       // start in reset

`ifdef BOUNDED
    always @* begin                                     // keeps BMC fast; the proof task does not use this
        if (wr_en && wr_sel == 3'd3) assume (wr_data <= 32'd6);
        if (wr_en && wr_sel == 3'd0) assume (wr_data[31:16] <= 16'd4);
    end
`endif

    // ---- independent shadow of one firing sequence (40-bit arithmetic: no wrap for any DELAY/HOLD) ----
    reg         chk_prev;
    wire        chk_now    = (match_mask != 8'h00) && ((pin_sync & match_mask) == (match_val & match_mask));
    wire        chk_edge   = chk_now && !chk_prev;
    wire        chk_accept = st_armed && trig_en && chk_edge;     // a match accepted in cycle m
    reg         chk_active;
    reg [39:0]  chk_cnt;                                          // = k in cycle m+1+k
    reg [31:0]  chk_D;
    reg [15:0]  chk_Hh;                                           // max(HOLD,1)
    reg [7:0]   chk_am, chk_av;
    wire [39:0] chk_end    = {8'd0, chk_D} + {24'd0, chk_Hh};     // first cycle AFTER the override window

    always @(posedge clk) begin
        if (!rst_n) begin
            chk_prev   <= 1'b0;
            chk_active <= 1'b0;
        end else begin
            chk_prev <= chk_now;
            if (!trig_en) begin
                chk_active <= 1'b0;
            end else if (chk_accept) begin
                chk_active <= 1'b1;
                chk_cnt    <= 40'd0;
                chk_D      <= delay_cfg;
                chk_Hh     <= (hold_cfg == 16'd0) ? 16'd1 : hold_cfg;
                chk_am     <= act_mask;
                chk_av     <= act_val;
            end else if (chk_active) begin
                chk_cnt <= chk_cnt + 40'd1;
                if (chk_cnt + 40'd1 == chk_end) chk_active <= 1'b0;
            end
        end
    end

    always @(posedge clk) if (f_past_valid) begin
        // ---------------- the properties ----------------
        // P7: the state is always one-hot
        assert ((st_dis + st_armed + st_cnt + st_fire) == 1);

        // P1 never early / P2 never missed / P3 exact width: override is high exactly in
        // cycles m+D+1 .. m+D+H after an accepted match in cycle m
        if (trig_en && chk_active)
            assert (override == (chk_cnt >= {8'd0, chk_D} && chk_cnt < chk_end));
        if (!chk_active)
            assert (!override);                          // no spurious override outside a sequence

        // P4: pass-through outside the window, exact override value inside it
        if (!override) begin
            assert (pad_out == pio_out);
            assert (pad_oe  == pio_oe);
        end else begin
            assert (pad_out == ((pio_out & ~chk_am) | (chk_av & chk_am)));
            assert (pad_oe  == (pio_oe | chk_am));
        end

        // P5: abort. TRIG_EN=0 kills the override in the same cycle and the unit is DISARMED next cycle
        if (!trig_en) assert (!override);
        if ($past(rst_n) && !$past(trig_en)) assert (st_dis);

        // P6: one-shot ends DISARMED, re-arm ends ARMED (and needs a new match edge: see P1/P2 above)
        if ($past(rst_n) && $past(st_fire) && $past(trig_en) && ($past(trg_hold) == 16'd0)) begin
            if ($past(rearm)) assert (st_armed);
            else              assert (st_dis);
        end

        // configuration is frozen whenever the unit was not DISARMED in the previous cycle
        if ($past(rst_n) && !$past(st_dis))
            assert ($stable({match_mask, match_val, delay_cfg, act_mask, act_val, hold_cfg, rearm}));

        // A3: every rejected write is reported
        if (wr_en && wr_sel <= 3'd5 && !st_dis) assert (cmd_err_pulse);

        // ---------------- helper invariants (needed only so k-induction can start from ANY state) ----------------
        // they tie the shadow checker to the design: same history, same snapshot, same counters
        assert (chk_prev == match_prev);
        assert (chk_active == (st_cnt || st_fire));
        if (st_cnt || st_fire) begin
            assert (chk_am == act_mask_s && chk_av == act_val_s);
            assert (chk_D == delay_cfg);
            assert (chk_Hh == ((hold_cfg == 16'd0) ? 16'd1 : hold_cfg));
        end
        if (st_cnt)
            assert (chk_D != 32'd0 && {8'd0, trg_cnt} + chk_cnt + 40'd1 == {8'd0, chk_D});
        if (st_fire)
            assert (chk_cnt >= {8'd0, chk_D} && chk_cnt < chk_end &&
                    {24'd0, trg_hold} + chk_cnt + 40'd1 == chk_end);
    end

    // non-vacuity: these situations must be reachable
    always @(posedge clk) if (f_past_valid) begin
        cover (override && chk_D == 32'd3 && chk_Hh == 16'd2);
        cover (fired_pulse && rearm);
        cover (fired_pulse && !rearm);
        cover (cmd_err_pulse);
        cover (st_cnt && !trig_en);
    end
`endif

endmodule
