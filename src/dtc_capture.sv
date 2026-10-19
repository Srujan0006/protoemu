// DTC Timestamp Capture Unit  (RTL spike 2).  Verilog-2001 style SystemVerilog.
//
// Behaviour is defined by dtc_model.py (_capture, _hread 0x10..0x13, CTRL.CAP_EN / CAP_CLR):
//   * free-running 32-bit cycle counter (CYCLE, host address 0x13), reset to 0
//   * every cycle:  changed = (pin_sync ^ pin_prev) & cap_mask
//       - if cap_en && changed:  push {cycle_cnt, pin_sync} into the 16-entry FIFO,
//         unless the FIFO was FULL at the start of the cycle (pre-cycle flag, A5):
//         then DROP THE NEWEST entry and pulse ovf_pulse (sticky flag lives in the shared flag register).
//       - cap_clr (CTRL write with CAP_CLR) empties the FIFO; clear wins over a same-edge push,
//         but a same-edge overflow is still reported.
//   * host reads (rd_en + rd_sel):  0 = CAP_MASK (0x10)   1 = CAP_LO (0x11, timestamp, no pop)
//                                   2 = CAP_HI (0x12, {23'b0, VALID, pins}, POPS if non-empty)   3 = CYCLE (0x13)
//     CAP_LO / CAP_HI read 0 when the FIFO is empty.  A pop and a push in the same cycle both happen
//     (the pop removes the old head; the new entry goes to the tail).
//   * timestamp = value of cycle_cnt in the cycle in which the registered edge is first visible on pin_sync,
//     i.e. the model's "edge cycle + 2" (the 2 synchronizer stages are outside this module).
//
// cap_en is a REGISTER in the top-level CTRL word (like trig_en for the Trigger Unit): this module reads it, it does not store it.
// The FIFO is a plain register array.  D29: if the 8x4 flow gives us an SRAM macro, this array is the one thing that moves into it
// (one read port at the head, one write port at the tail: it already has exactly that shape).
module dtc_capture (
    input  wire        clk,
    input  wire        rst_n,        // synchronous, active low
    input  wire [7:0]  pin_sync,     // output of the 2-FF input synchronizer
    input  wire        cap_en,       // CTRL.CAP_EN (registered)
    input  wire        cap_clr,      // strobe: CTRL write with CAP_CLR=1, same cycle as the write
    input  wire        wr_en,        // strobe: write to CAP_MASK (0x10)
    input  wire [7:0]  wr_data,
    input  wire        rd_en,        // strobe: host read of 0x10..0x13
    input  wire [1:0]  rd_sel,       // 0..3 = 0x10..0x13
    output reg  [31:0] rd_data,      // combinational, valid in the cycle of rd_en
    output wire [4:0]  cap_count,    // STATUS[20:16]
    output wire        cap_full,     // STATUS[12]  (pre-cycle value)
    output wire        cap_empty,    // STATUS[13]
    output wire        ovf_pulse,    // one cycle: a push was dropped (sets the sticky CAP_OVF flag)
    output wire [31:0] cycle_cnt
);
    localparam DEPTH = 16;

    reg [31:0] cycle_q;
    reg [7:0]  mask_q;
    reg [7:0]  prev_q;
    reg [39:0] mem [0:DEPTH-1];     // {timestamp[31:0], pins[7:0]}
    reg [3:0]  wptr, rptr;
    reg [4:0]  count;

    assign cycle_cnt = cycle_q;
    assign cap_count = count;
    assign cap_full  = (count == 5'd16);
    assign cap_empty = (count == 5'd0);

    wire [7:0] changed = (pin_sync ^ prev_q) & mask_q;
    wire       event_s = cap_en && (changed != 8'h00);
    wire       push    = event_s && !cap_full;
    assign     ovf_pulse = event_s && cap_full;

    wire [39:0] head    = mem[rptr];
    wire        pop     = rd_en && (rd_sel == 2'd2) && !cap_empty;

    always @* begin
        case (rd_sel)
            2'd0:    rd_data = {24'b0, mask_q};
            2'd1:    rd_data = cap_empty ? 32'b0 : head[39:8];
            2'd2:    rd_data = cap_empty ? 32'b0 : {23'b0, 1'b1, head[7:0]};
            default: rd_data = cycle_q;
        endcase
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            cycle_q <= 32'b0;
            mask_q  <= 8'b0;
            prev_q  <= 8'b0;
            wptr    <= 4'b0;
            rptr    <= 4'b0;
            count   <= 5'b0;
        end else begin
            cycle_q <= cycle_q + 32'd1;
            prev_q  <= pin_sync;
            if (wr_en) mask_q <= wr_data;

            if (cap_clr) begin                      // clear wins over push; pop is moot
                wptr  <= 4'b0;
                rptr  <= 4'b0;
                count <= 5'b0;
            end else begin
                if (push) begin
                    mem[wptr] <= {cycle_q, pin_sync};
                    wptr      <= wptr + 4'd1;
                end
                if (pop) rptr <= rptr + 4'd1;
                count <= count + {4'b0, push} - {4'b0, pop};
            end
        end
    end

`ifdef FORMAL
    // ---------------------------------------------------------------- formal (SymbiYosys)
    // C1  count never exceeds 16 and always equals wptr - rptr (mod 16), with 16 only when wptr == rptr
    // C2  a push never happens when full (drop-newest), and a dropped event always raises ovf_pulse
    // C3  clear leaves the FIFO empty
    // C4  DATA INTEGRITY: the first entry pushed after reset/clear is tracked.  When it reaches the head,
    //     CAP_LO and CAP_HI return exactly the timestamp and pins that were pushed (nothing lost, reordered or corrupted).
    reg f_past = 1'b0;
    always @(posedge clk) f_past <= 1'b1;
    always @* if (!f_past) assume(!rst_n);          // start in reset

    always @(posedge clk) if (rst_n && f_past) begin
        assert(count <= 5'd16);
        assert((wptr - rptr) == count[3:0]);
        assert(count != 5'd16 || wptr == rptr);
        assert(!(push && cap_full));
        assert(!(event_s && cap_full) || ovf_pulse);
        assert(!(event_s && !cap_full) || !ovf_pulse);
        assert(!(count == 5'd0 && pop));
    end

    // C3, one cycle later
    reg f_clr_q = 1'b0;
    always @(posedge clk) f_clr_q <= rst_n && cap_clr;
    always @(posedge clk) if (f_past && f_clr_q) assert(count == 5'd0);

    // C4: tracked entry
    reg        t_seen = 1'b0;
    reg [4:0]  t_pos  = 5'd0;                       // entries ahead of the tracked one
    reg [31:0] t_ts   = 32'd0;
    reg [7:0]  t_pins = 8'd0;
    wire       head_is_t = t_seen && (t_pos == 5'd0);

    always @(posedge clk) begin
        if (!rst_n || cap_clr) begin
            t_seen <= 1'b0;
        end else begin
            if (t_seen) begin
                if (pop) begin
                    if (t_pos == 5'd0) t_seen <= 1'b0;      // consumed
                    else               t_pos  <= t_pos - 5'd1;
                end
            end else if (push) begin
                t_seen <= 1'b1;
                t_pos  <= count - {4'b0, pop};
                t_ts   <= cycle_q;
                t_pins <= pin_sync;
            end
        end
    end

    // helper invariants (so that k-induction can close C4)
    always @(posedge clk) if (rst_n && f_past && t_seen) begin
        assert(t_pos < count);
        assert(mem[rptr + t_pos[3:0]] == {t_ts, t_pins});
    end
    always @(posedge clk) if (rst_n && f_past && head_is_t && rd_en && !cap_empty) begin
        if (rd_sel == 2'd1) assert(rd_data == t_ts);
        if (rd_sel == 2'd2) assert(rd_data == {23'b0, 1'b1, t_pins});
    end

    // non-vacuity
    always @(posedge clk) if (rst_n && f_past) begin
        cover(cap_full);
        cover(ovf_pulse);
        cover(push && pop);
        cover(head_is_t && pop && rd_data[8]);
        cover(cap_clr && count != 5'd0);
    end
`endif
endmodule
