// DTC PIO-class state machine (one machine).  RTL spike 3.  Verilog-2001 style SystemVerilog.
//
// Behaviour is defined by dtc_model.py (_core / _exec / _hwrite / _hread / the per-machine part of step()).
// What lives here  : PC, X, Y, OSR, ISR, shift counters, delay counter, divider, forced-instruction slot,
//                    EXECCTRL / SHIFTCTRL / PINCTRL / CLKDIV / WAIT_CFG, TX and RX FIFOs (4 x 32), per-machine sticky flags.
// What lives OUTSIDE (dtc_core_top): IMEM, the shared IRQ flags, the shared pin registers, the input synchronizer, CTRL.
//
// Three seams that make NUM_SM=2 a parameter flip:
//   1. PIN WRITES  are exported as per-pin write masks (we) and values (val), NOT as held registers.
//      Side-set is merged here and wins per pin over OUT / SET / MOV PINS.  The shared pio_out / pio_oe flops sit outside;
//      with two machines SM1 is applied after SM0, so SM1 wins a same-edge write to the same pin (D23).
//   2. IMEM        is read through imem_raddr / imem_rdata (one port per machine).
//   3. IRQ FLAGS   are read through irq_flags and written through one-cycle irq_set / irq_clr strobes.
//
// Corner cases carried over from the model (see comments at the places they matter):
//   * a count of 0 in OUT / IN means 32 bits; shifts use 64-bit intermediates so a 32-bit shift is exact
//   * a stalled autopush IN shifts the ISR on its FIRST evaluation only (push_pending); later evaluations only retry the push
//   * a stalled IRQ-wait raises the flag on its FIRST evaluation only (irq_wait); later evaluations only watch for the clear
//   * side-set is stateless (A7): it is applied on EVERY evaluation of an instruction, including stalled ones
//   * pin indices wrap modulo 8; WAIT GPIO idx and WAIT PIN (IN_BASE + idx) wrap the same way (A9)
//
// FIFO flags used by the datapath are the values at the START of the cycle (A5); push and pop in the same cycle both happen.
//
// dbg layout (LSB first), used by the co-simulation only; it is unconnected (and optimised away) in the chip:
//   pc[5:0] x[31:0] y[31:0] osr[31:0] isr[31:0] osr_cnt[5:0] isr_cnt[5:0] delay[4:0] stalled push_pend irq_wait forced_v
//   div_cnt[15:0] tx_n[2:0] rx_n[2:0] flags[2:0]={rxf_und,txf_ovr,rx_lost}                      -> 180 bits
module dtc_sm #(
    parameter DBGW = 180
) (
    input  wire         clk,
    input  wire         rst_n,            // synchronous, active low
    // control (from the CTRL register in the top level; restart / clr / div_restart are one-cycle strobes)
    input  wire         en,
    input  wire         restart,
    input  wire         div_restart,
    input  wire         tx_clr,
    input  wire         rx_clr,
    // host port for this machine's registers.  h_sel = address[3:0]: 4 CLKDIV 5 EXECCTRL 6 SHIFTCTRL 7 PINCTRL
    //                                                  8 TXF (wr) 9 RXF (rd, pops) 10 INSTR (wr) 11 PC (rd) 14 WAIT_CFG
    input  wire         h_wr,
    input  wire         h_rd,
    input  wire [3:0]   h_sel,
    input  wire [31:0]  h_wdata,
    output reg  [31:0]  h_rdata,
    input  wire [31:0]  flag_w1c,         // write-1-to-clear for the per-machine flags (bits 23, 28, 29 are used)
    output wire [31:0]  status,           // STATUS / STATUS1 contribution of this machine
    output wire         cmd_err_pulse,    // INSTR written while a forced instruction is still pending
    // pins
    input  wire [7:0]   pin_sync,
    output wire [7:0]   pin_out_we,
    output wire [7:0]   pin_out_val,
    output wire [7:0]   pin_oe_we,
    output wire [7:0]   pin_oe_val,
    // instruction memory read port
    output wire [5:0]   imem_raddr,
    input  wire [15:0]  imem_rdata,
    // shared IRQ flags
    input  wire [7:0]   irq_flags,
    output wire [7:0]   irq_set,
    output wire [7:0]   irq_clr,
    // co-simulation visibility
    output wire [DBGW-1:0] dbg
);
    // ------------------------------------------------------------------ state
    reg [15:0] clkdiv_q, div_cnt_q;
    reg [17:0] execctrl_q;
    reg [13:0] shiftctrl_q;
    reg [26:0] pinctrl_q;
    reg [31:0] wait_cfg_q;
    reg [5:0]  pc_q;
    reg [31:0] x_q, y_q, osr_q, isr_q;
    reg [5:0]  osr_cnt_q, isr_cnt_q;      // 0..32
    reg [4:0]  delay_q;
    reg        stalled_q, push_pend_q, irq_wait_q, forced_v_q;
    reg [15:0] forced_q;
    reg        f_rx_lost_q, f_txf_ovr_q, f_rxf_und_q;

    reg [31:0] txm [0:3];
    reg [31:0] rxm [0:3];
    reg [1:0]  tx_wp, tx_rp, rx_wp, rx_rp;
    reg [2:0]  tx_cnt, rx_cnt;

    // ------------------------------------------------------------------ register fields
    wire [5:0] wrap_bottom = execctrl_q[5:0];
    wire [5:0] wrap_top    = execctrl_q[11:6];
    wire [2:0] jmp_pin     = execctrl_q[14:12];
    wire       side_en     = execctrl_q[16];
    wire       side_pindir = execctrl_q[17];
    wire       autopush    = shiftctrl_q[0];
    wire       autopull    = shiftctrl_q[1];
    wire       in_right    = shiftctrl_q[2];
    wire       out_right   = shiftctrl_q[3];
    wire [5:0] push_thresh = (shiftctrl_q[8:4]  == 5'd0) ? 6'd32 : {1'b0, shiftctrl_q[8:4]};
    wire [5:0] pull_thresh = (shiftctrl_q[13:9] == 5'd0) ? 6'd32 : {1'b0, shiftctrl_q[13:9]};
    wire [2:0] out_base    = pinctrl_q[2:0];
    wire [3:0] out_count   = (pinctrl_q[7:4] > 4'd8) ? 4'd8 : pinctrl_q[7:4];
    wire [2:0] set_base    = pinctrl_q[10:8];
    wire [3:0] set_count   = (pinctrl_q[14:12] > 3'd5) ? 4'd5 : {1'b0, pinctrl_q[14:12]};
    wire [2:0] side_base   = pinctrl_q[18:16];
    wire [2:0] side_count  = (pinctrl_q[22:20] > 3'd5) ? 3'd5 : pinctrl_q[22:20];
    wire [2:0] in_base     = pinctrl_q[26:24];

    wire [15:0] div_lim = (clkdiv_q == 16'd0) ? 16'd1 : clkdiv_q;
    wire        tick    = (div_cnt_q >= (div_lim - 16'd1));

    wire tx_full     = (tx_cnt == 3'd4);
    wire tx_nonempty = (tx_cnt != 3'd0);
    wire rx_full     = (rx_cnt == 3'd4);
    wire rx_nonempty = (rx_cnt != 3'd0);
    wire [31:0] tx_head = txm[tx_rp];
    wire [31:0] rx_head = rxm[rx_rp];

    // ------------------------------------------------------------------ what executes this cycle
    wire exec_forced = !restart && tick && forced_v_q;
    wire exec_fetch  = !restart && tick && !forced_v_q && en && (delay_q == 5'd0);
    wire exec        = exec_forced || exec_fetch;
    wire dec_delay   = !restart && tick && !forced_v_q && en && (delay_q != 5'd0);

    assign imem_raddr = pc_q;
    wire [15:0] instr = forced_v_q ? forced_q : imem_rdata;
    wire [15:0] pin_rot   = {pin_sync, pin_sync} >> in_base;             // rotate right by IN_BASE
    wire [31:0] pin_val32 = {24'b0, pin_rot[7:0]};                        // zero-extended

    // ------------------------------------------------------------------ helpers
    function [7:0] pmask;                // pins touched when `cnt` bits are written from `base` (wraps modulo 8)
        input [2:0] base;
        input [3:0] cnt;
        integer k;
        reg [2:0] p;
        begin
            pmask = 8'b0;
            for (k = 0; k < 8; k = k + 1) begin
                p = base + k[2:0];
                if (k < cnt) pmask[p] = 1'b1;
            end
        end
    endfunction

    function [7:0] pvals;                // value bit k lands on pin (base + k) % 8
        input [2:0] base;
        input [3:0] cnt;
        input [7:0] v;
        integer k;
        reg [2:0] p;
        begin
            pvals = 8'b0;
            for (k = 0; k < 8; k = k + 1) begin
                p = base + k[2:0];
                if (k < cnt) pvals[p] = v[k];
            end
        end
    endfunction

    function [31:0] mask32;              // low n bits set, n = 1..32
        input [5:0] n;
        reg [63:0] t;
        begin
            t = (64'd1 << n) - 64'd1;
            mask32 = t[31:0];
        end
    endfunction

    function [31:0] bitrev;
        input [31:0] v;
        integer k;
        begin
            for (k = 0; k < 32; k = k + 1) bitrev[k] = v[31 - k];
        end
    endfunction

    // ------------------------------------------------------------------ combinational execute
    reg [31:0] n_x, n_y, n_osr, n_isr;
    reg [5:0]  n_osr_cnt, n_isr_cnt;
    reg        n_push_pend, n_irq_wait;
    reg        done, jump_v, tx_pop, rx_push, rx_lost_set;
    reg [5:0]  jump_a;
    reg [31:0] rx_pdata;
    reg [7:0]  om, ov, em, ev;            // pin writes of the instruction body
    reg [7:0]  sm_, sv_;                  // side-set mask / value
    reg [7:0]  irq_set_c, irq_clr_c;
    reg [4:0]  dly;

    reg [2:0]  op, cond_, dest, src, idx;
    reg [1:0]  mop;
    reg [7:0]  ops;
    reg        take, pol;
    reg [5:0]  n;
    reg [31:0] val, data, osr_c, mk, vl;
    reg [5:0]  cnt_c;
    reg [6:0]  cnt_sum;
    reg [63:0] w64;
    reg [31:0] isr_sh;
    reg [3:0]  pc_cnt;
    reg [5:0]  dbits;
    reg [4:0]  area;
    reg [3:0]  nb;
    reg        apply;

    always @* begin
        // defaults: nothing changes
        n_x = x_q;  n_y = y_q;  n_osr = osr_q;  n_isr = isr_q;
        n_osr_cnt = osr_cnt_q;  n_isr_cnt = isr_cnt_q;
        n_push_pend = push_pend_q;  n_irq_wait = irq_wait_q;
        done = 1'b1;  jump_v = 1'b0;  jump_a = 6'd0;
        tx_pop = 1'b0;  rx_push = 1'b0;  rx_pdata = 32'd0;  rx_lost_set = 1'b0;
        om = 8'b0;  ov = 8'b0;  em = 8'b0;  ev = 8'b0;
        sm_ = 8'b0; sv_ = 8'b0;
        irq_set_c = 8'b0;  irq_clr_c = 8'b0;
        dly = 5'd0;
        op = instr[15:13];  ops = instr[7:0];
        cond_ = 3'd0; dest = 3'd0; mop = 2'd0; src = 3'd0; idx = 3'd0;
        take = 1'b0; pol = 1'b0; n = 6'd1;
        val = 32'd0; data = 32'd0; osr_c = 32'd0; mk = 32'd0; vl = 32'd0;
        cnt_c = 6'd0; cnt_sum = 7'd0; w64 = 64'd0; isr_sh = 32'd0; pc_cnt = 4'd0;
        dbits = 6'd0; area = 5'd0; nb = 4'd0; apply = 1'b0;

        case (op)
        // ---------------------------------------------------------------- JMP
        3'b000: begin
            cond_ = ops[7:5];
            case (cond_)
                3'd0: take = 1'b1;
                3'd1: take = (x_q == 32'd0);
                3'd2: begin take = (x_q != 32'd0); n_x = x_q - 32'd1; end   // always decrements
                3'd3: take = (y_q == 32'd0);
                3'd4: begin take = (y_q != 32'd0); n_y = y_q - 32'd1; end
                3'd5: take = (x_q != y_q);
                3'd6: take = pin_sync[jmp_pin];
                default: take = (osr_cnt_q < pull_thresh);                  // !OSRE
            endcase
            if (take) begin
                jump_v = 1'b1;
                jump_a = forced_v_q ? {instr[8], ops[4:0]} : {pc_q[5], ops[4:0]};
            end
        end
        // ---------------------------------------------------------------- WAIT
        3'b001: begin
            pol = ops[7];
            src = {1'b0, ops[6:5]};
            case (ops[6:5])
                2'd0: done = (pin_sync[ops[2:0]] == pol);                    // GPIO, idx % 8
                2'd1: done = (pin_sync[in_base + ops[2:0]] == pol);          // PIN, (IN_BASE + idx) % 8
                2'd2: if (ops[4:3] == 2'b00) begin                           // IRQ flag (idx[4:3] != 0 is a NOP)
                    done = (irq_flags[ops[2:0]] == pol);
                    if (done && pol) irq_clr_c[ops[2:0]] = 1'b1;
                end
                default: if (ops[4:1] == 4'd0) begin                         // masked pattern, slot = idx[0]
                    mk = ops[0] ? {24'b0, wait_cfg_q[23:16]} : {24'b0, wait_cfg_q[7:0]};
                    vl = ops[0] ? {24'b0, wait_cfg_q[31:24]} : {24'b0, wait_cfg_q[15:8]};
                    done = (((pin_sync & mk[7:0]) == (vl[7:0] & mk[7:0])) == pol);
                end
            endcase
        end
        // ---------------------------------------------------------------- IN
        3'b010: begin
            n   = (ops[4:0] == 5'd0) ? 6'd32 : {1'b0, ops[4:0]};
            src = ops[7:5];
            if (push_pend_q) begin                                           // shift already done: only retry the push
                if (!rx_full) begin
                    rx_push = 1'b1;  rx_pdata = isr_q;
                    n_isr = 32'd0;  n_isr_cnt = 6'd0;  n_push_pend = 1'b0;
                end else done = 1'b0;
            end else if (src == 3'd4 || src == 3'd5) begin
                // reserved source: NOP
            end else begin
                case (src)
                    3'd0:    val = pin_val32;
                    3'd1:    val = x_q;
                    3'd2:    val = y_q;
                    3'd3:    val = 32'd0;
                    3'd6:    val = isr_q;
                    default: val = osr_q;
                endcase
                val = val & mask32(n);
                if (in_right) begin
                    w64 = ({32'b0, val} << (6'd32 - n)) | ({32'b0, isr_q} >> n);
                    isr_sh = w64[31:0];
                end else begin
                    w64 = ({32'b0, isr_q} << n) | {32'b0, val};
                    isr_sh = w64[31:0];
                end
                cnt_sum = {1'b0, isr_cnt_q} + {1'b0, n};
                n_isr = isr_sh;
                n_isr_cnt = (cnt_sum > 7'd32) ? 6'd32 : cnt_sum[5:0];
                if (autopush && n_isr_cnt >= push_thresh) begin
                    if (!rx_full) begin
                        rx_push = 1'b1;  rx_pdata = isr_sh;
                        n_isr = 32'd0;  n_isr_cnt = 6'd0;
                    end else begin
                        n_push_pend = 1'b1;                                  // keep the shifted ISR, never shift twice
                        done = 1'b0;
                    end
                end
            end
        end
        // ---------------------------------------------------------------- OUT
        3'b011: begin
            n    = (ops[4:0] == 5'd0) ? 6'd32 : {1'b0, ops[4:0]};
            dest = ops[7:5];
            osr_c = osr_q;  cnt_c = osr_cnt_q;
            if (autopull && cnt_c >= pull_thresh && dest != 3'd7) begin
                if (tx_nonempty) begin
                    tx_pop = 1'b1;  osr_c = tx_head;  cnt_c = 6'd0;
                end else done = 1'b0;
            end
            if (done && dest != 3'd7) begin                                  // D27: dest 7 (OUT EXEC) is a reserved NOP
                if (out_right) begin
                    data = osr_c & mask32(n);
                    w64 = {32'b0, osr_c} >> n;
                    n_osr = w64[31:0];
                end else begin
                    w64 = {32'b0, osr_c} >> (6'd32 - n);
                    data = w64[31:0];
                    w64 = {32'b0, osr_c} << n;
                    n_osr = w64[31:0];
                end
                cnt_sum = {1'b0, cnt_c} + {1'b0, n};
                n_osr_cnt = (cnt_sum > 7'd32) ? 6'd32 : cnt_sum[5:0];
                case (dest)
                    3'd0: begin
                        pc_cnt = (n > {2'b0, out_count}) ? out_count : n[3:0];
                        om = pmask(out_base, pc_cnt);  ov = pvals(out_base, pc_cnt, data[7:0]);
                    end
                    3'd1: n_x = data;
                    3'd2: n_y = data;
                    3'd4: begin
                        pc_cnt = (n > {2'b0, out_count}) ? out_count : n[3:0];
                        em = pmask(out_base, pc_cnt);  ev = pvals(out_base, pc_cnt, data[7:0]);
                    end
                    3'd5: begin jump_v = 1'b1; jump_a = data[5:0]; end
                    3'd6: begin n_isr = data; n_isr_cnt = n; end
                    default: ;                                               // NULL: shifted out and discarded
                endcase
            end
        end
        // ---------------------------------------------------------------- PUSH / PULL
        3'b100: begin
            if (ops[7]) begin                                                // PULL
                if (ops[6] && osr_cnt_q < pull_thresh) begin
                    // IfEmpty and not empty: no-op
                end else if (tx_nonempty) begin
                    tx_pop = 1'b1;  n_osr = tx_head;  n_osr_cnt = 6'd0;
                end else if (ops[5]) done = 1'b0;                            // blocking: stall
                else begin n_osr = x_q;  n_osr_cnt = 6'd0; end               // non-blocking: copy X
            end else begin                                                   // PUSH
                if (ops[6] && isr_cnt_q < push_thresh) begin
                    // IfFull and not full: no-op
                end else if (!rx_full) begin
                    rx_push = 1'b1;  rx_pdata = isr_q;
                    n_isr = 32'd0;  n_isr_cnt = 6'd0;
                end else if (ops[5]) done = 1'b0;                            // blocking: stall
                else begin                                                   // non-blocking: drop, clear, flag
                    n_isr = 32'd0;  n_isr_cnt = 6'd0;  rx_lost_set = 1'b1;
                end
            end
        end
        // ---------------------------------------------------------------- MOV
        3'b101: begin
            dest = ops[7:5];  mop = ops[4:3];  src = ops[2:0];
            if (dest != 3'd3 && mop != 2'd3 && src != 3'd4 && src != 3'd5) begin    // reserved codes: NOP
                case (src)
                    3'd0:    val = pin_val32;
                    3'd1:    val = x_q;
                    3'd2:    val = y_q;
                    3'd3:    val = 32'd0;
                    3'd6:    val = isr_q;
                    default: val = osr_q;
                endcase
                if (mop == 2'd1)      val = ~val;
                else if (mop == 2'd2) val = bitrev(val);
                case (dest)
                    3'd0: begin om = pmask(out_base, out_count);  ov = pvals(out_base, out_count, val[7:0]); end
                    3'd1: n_x = val;
                    3'd2: n_y = val;
                    3'd4: ;                                                  // D27: MOV EXEC is a reserved NOP
                    3'd5: begin jump_v = 1'b1; jump_a = val[5:0]; end
                    3'd6: begin n_isr = val; n_isr_cnt = 6'd0; end
                    default: begin n_osr = val; n_osr_cnt = 6'd0; end
                endcase
            end
        end
        // ---------------------------------------------------------------- IRQ / LJMP / PJMP
        3'b110: begin
            idx  = ops[2:0];
            case (ops[4:3])
                2'd1: begin jump_v = 1'b1; jump_a = {ops[7:5], ops[2:0]}; end        // D19 LJMP
                2'd0: begin
                    if (ops[7]) irq_clr_c[idx] = 1'b1;                               // clear
                    else if (irq_wait_q) begin                                       // flag already raised: wait for the clear
                        if (irq_flags[idx]) done = 1'b0;
                        else n_irq_wait = 1'b0;
                    end else begin
                        irq_set_c[idx] = 1'b1;
                        if (ops[6]) begin n_irq_wait = 1'b1; done = 1'b0; end        // wait variant: stall after raising
                    end
                end
                2'd2: begin                                                          // D30 PJMP, non-blocking
                    mk = ops[7] ? {24'b0, wait_cfg_q[23:16]} : {24'b0, wait_cfg_q[7:0]};
                    vl = ops[7] ? {24'b0, wait_cfg_q[31:24]} : {24'b0, wait_cfg_q[15:8]};
                    if ((pin_sync & mk[7:0]) == (vl[7:0] & mk[7:0])) begin
                        jump_v = 1'b1;
                        jump_a = {pc_q[5], ops[6:5], ops[2:0]};
                    end
                end
                default: ;                                                           // mode 3: reserved NOP
            endcase
        end
        // ---------------------------------------------------------------- SET
        default: begin
            dest = ops[7:5];
            case (dest)
                3'd0: begin om = pmask(set_base, set_count);  ov = pvals(set_base, set_count, {3'b0, ops[4:0]}); end
                3'd1: n_x = {27'b0, ops[4:0]};
                3'd2: n_y = {27'b0, ops[4:0]};
                3'd4: begin em = pmask(set_base, set_count);  ev = pvals(set_base, set_count, {3'b0, ops[4:0]}); end
                default: ;                                                           // reserved: NOP
            endcase
        end
        endcase

        // side-set and delay (A7: stateless; applied on every evaluation; forced instructions have neither, D18)
        if (!forced_v_q) begin
            dbits = 6'd5 - {3'b0, side_count};
            dly   = instr[12:8] & ((5'd1 << dbits) - 5'd1);
            area  = instr[12:8] >> dbits;
            if (side_count != 3'd0) begin
                if (side_en) begin
                    apply = area[side_count - 3'd1];
                    nb    = {1'b0, side_count} - 4'd1;
                end else begin
                    apply = 1'b1;
                    nb    = {1'b0, side_count};
                end
                if (apply && nb != 4'd0) begin
                    sm_ = pmask(side_base, nb);
                    sv_ = pvals(side_base, nb, {3'b0, area});
                end
            end
        end
    end

    // side-set is applied last, so it wins per pin over the instruction body
    wire [7:0] o_we  = om | (side_pindir ? 8'b0 : sm_);
    wire [7:0] o_val = (ov & ~(side_pindir ? 8'b0 : sm_)) | (sv_ & (side_pindir ? 8'b0 : sm_));
    wire [7:0] e_we  = em | (side_pindir ? sm_ : 8'b0);
    wire [7:0] e_val = (ev & ~(side_pindir ? sm_ : 8'b0)) | (sv_ & (side_pindir ? sm_ : 8'b0));

    assign pin_out_we  = exec ? o_we  : 8'b0;
    assign pin_out_val = o_val;
    assign pin_oe_we   = exec ? e_we  : 8'b0;
    assign pin_oe_val  = e_val;
    assign irq_set     = exec ? irq_set_c : 8'b0;
    assign irq_clr     = exec ? irq_clr_c : 8'b0;

    // ------------------------------------------------------------------ host side
    wire force_wr     = h_wr && (h_sel == 4'd10) && !forced_v_q;
    assign cmd_err_pulse = h_wr && (h_sel == 4'd10) && forced_v_q;
    wire tx_push      = h_wr && (h_sel == 4'd8) && !tx_full;
    wire txf_ovr_set  = h_wr && (h_sel == 4'd8) && tx_full;
    wire rx_pop       = h_rd && (h_sel == 4'd9) && rx_nonempty;
    wire rxf_und_set  = h_rd && (h_sel == 4'd9) && !rx_nonempty;
    wire do_rx_push   = exec && rx_push;
    wire do_tx_pop    = exec && tx_pop;

    always @* begin
        case (h_sel)
            4'd4:    h_rdata = {16'b0, clkdiv_q};
            4'd5:    h_rdata = {14'b0, execctrl_q};
            4'd6:    h_rdata = {18'b0, shiftctrl_q};
            4'd7:    h_rdata = {5'b0, pinctrl_q};
            4'd9:    h_rdata = rx_nonempty ? rx_head : 32'd0;
            4'd11:   h_rdata = {26'b0, pc_q};
            4'd14:   h_rdata = wait_cfg_q;
            default: h_rdata = 32'd0;
        endcase
    end

    assign status = {2'b0, f_rxf_und_q, f_txf_ovr_q, 4'b0, f_rx_lost_q, 12'b0,
                     rx_cnt, stalled_q, tx_cnt, !rx_nonempty, rx_full, !tx_nonempty, tx_full};

    // ------------------------------------------------------------------ sequential
    always @(posedge clk) begin
        if (!rst_n) begin
            clkdiv_q    <= 16'd1;
            div_cnt_q   <= 16'd0;
            execctrl_q  <= 18'd63 << 6;
            shiftctrl_q <= 14'h0C;
            pinctrl_q   <= 27'd0;
            wait_cfg_q  <= 32'd0;
            pc_q        <= 6'd0;
            x_q <= 32'd0;  y_q <= 32'd0;  osr_q <= 32'd0;  isr_q <= 32'd0;
            osr_cnt_q   <= 6'd32;
            isr_cnt_q   <= 6'd0;
            delay_q     <= 5'd0;
            stalled_q <= 1'b0;  push_pend_q <= 1'b0;  irq_wait_q <= 1'b0;  forced_v_q <= 1'b0;
            f_rx_lost_q <= 1'b0;  f_txf_ovr_q <= 1'b0;  f_rxf_und_q <= 1'b0;
            tx_wp <= 2'd0;  tx_rp <= 2'd0;  tx_cnt <= 3'd0;
            rx_wp <= 2'd0;  rx_rp <= 2'd0;  rx_cnt <= 3'd0;
        end else begin
            // divider (runs regardless of restart)
            div_cnt_q <= (div_restart || tick) ? 16'd0 : div_cnt_q + 16'd1;

            // configuration registers
            if (h_wr) begin
                case (h_sel)
                    4'd4:  clkdiv_q    <= h_wdata[15:0];
                    4'd5:  execctrl_q  <= h_wdata[17:0] & 18'h37FFF;
                    4'd6:  shiftctrl_q <= h_wdata[13:0];
                    4'd7:  pinctrl_q   <= h_wdata[26:0] & 27'h77777F7;
                    4'd14: wait_cfg_q  <= h_wdata;
                    default: ;
                endcase
            end

            // sticky per-machine flags: set wins over write-1-to-clear
            f_rx_lost_q <= (f_rx_lost_q & ~flag_w1c[23]) | (exec && rx_lost_set);
            f_txf_ovr_q <= (f_txf_ovr_q & ~flag_w1c[28]) | txf_ovr_set;
            f_rxf_und_q <= (f_rxf_und_q & ~flag_w1c[29]) | rxf_und_set;

            // FIFOs.  TX_CLR / RX_CLR win over a same-edge push or pop.
            if (tx_clr) begin
                tx_wp <= 2'd0;  tx_rp <= 2'd0;  tx_cnt <= 3'd0;
            end else begin
                if (tx_push) begin txm[tx_wp] <= h_wdata;  tx_wp <= tx_wp + 2'd1; end
                if (do_tx_pop) tx_rp <= tx_rp + 2'd1;
                tx_cnt <= tx_cnt + {2'b0, tx_push} - {2'b0, do_tx_pop};
            end
            if (rx_clr) begin
                rx_wp <= 2'd0;  rx_rp <= 2'd0;  rx_cnt <= 3'd0;
            end else begin
                if (do_rx_push) begin rxm[rx_wp] <= rx_pdata;  rx_wp <= rx_wp + 2'd1; end
                if (rx_pop) rx_rp <= rx_rp + 2'd1;
                rx_cnt <= rx_cnt + {2'b0, do_rx_push} - {2'b0, rx_pop};
            end

            // core
            if (restart) begin                                               // restart wins over execute
                osr_q <= 32'd0;  osr_cnt_q <= 6'd32;  isr_q <= 32'd0;  isr_cnt_q <= 6'd0;
                delay_q <= 5'd0;  stalled_q <= 1'b0;
                push_pend_q <= 1'b0;  irq_wait_q <= 1'b0;  forced_v_q <= 1'b0;
            end else begin
                if (force_wr) begin forced_q <= h_wdata[15:0];  forced_v_q <= 1'b1; end
                if (dec_delay) delay_q <= delay_q - 5'd1;
                if (exec) begin
                    x_q <= n_x;  y_q <= n_y;  osr_q <= n_osr;  isr_q <= n_isr;
                    osr_cnt_q <= n_osr_cnt;  isr_cnt_q <= n_isr_cnt;
                    push_pend_q <= n_push_pend;  irq_wait_q <= n_irq_wait;
                    stalled_q <= !done;
                    if (done) begin
                        if (forced_v_q) begin
                            forced_v_q <= 1'b0;
                            if (jump_v) pc_q <= jump_a;
                        end else begin
                            if (jump_v)                 pc_q <= jump_a;
                            else if (pc_q == wrap_top)  pc_q <= wrap_bottom;
                            else                        pc_q <= pc_q + 6'd1;
                            delay_q <= dly;
                        end
                    end
                end
            end
        end
    end

    assign dbg = {f_rxf_und_q, f_txf_ovr_q, f_rx_lost_q, rx_cnt, tx_cnt, div_cnt_q,
                  forced_v_q, irq_wait_q, push_pend_q, stalled_q, delay_q, isr_cnt_q, osr_cnt_q,
                  isr_q, osr_q, y_q, x_q, pc_q};
endmodule
