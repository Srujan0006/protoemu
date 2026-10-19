// DTC serial config port: SPI target (mode 0, MSB first) that drives the parallel host bus of dtc_top.
//
// Specification: dtc_spi.py (SpiPort is the cycle model of this file) and DTC_project_summary.md section 20.
//   header  RW[15] (1 = read) AINC[14] reserved[13:8] addr[7:0]
//   write   32-bit words, one h_wr per complete word; a partial header or partial word at CS_N high -> frame_err
//   read    per word: 8 turnaround clocks (h_rd issued at the first one: pop side effects), then 32 data bits
//   A13     CS_N / SCK / MOSI through 2-FF synchronizers in the clk domain; MISO is updated on the DETECTED rising
//           SCK edge with the next bit (pre-load); SCK <= clk/8.
// All outputs are registered.  h_wdata is the shift register itself (stable for a whole SCK period after the word).
module dtc_spi (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        cs_n,
    input  wire        sck,
    input  wire        mosi,
    output reg         miso,
    output reg         h_wr,
    output reg         h_rd,
    output wire [7:0]  h_addr,
    output wire [31:0] h_wdata,
    input  wire [31:0] h_rdata,
    output reg         frame_err
);
    localparam [1:0] HDR = 2'd0, WR = 2'd1, RD = 2'd2;

    reg        cs1, cs2, sck1, sck2, sck_q, mo1, mo2;
    reg [1:0]  st;
    reg [5:0]  cnt;
    reg [31:0] sr;
    reg [7:0]  addr;
    reg        ainc;

    wire        rise    = sck2 & ~sck_q;
    wire [31:0] shifted = {sr[30:0], mo2};

    assign h_addr  = addr;
    assign h_wdata = sr;

    always @(posedge clk) begin
        if (!rst_n) begin
            cs1 <= 1'b1;  cs2 <= 1'b1;  sck1 <= 1'b0;  sck2 <= 1'b0;  sck_q <= 1'b0;  mo1 <= 1'b0;  mo2 <= 1'b0;
            st <= HDR;  cnt <= 6'd0;  sr <= 32'b0;  addr <= 8'b0;  ainc <= 1'b0;
            h_wr <= 1'b0;  h_rd <= 1'b0;  frame_err <= 1'b0;  miso <= 1'b0;
        end else begin
            cs1 <= cs_n;  cs2 <= cs1;  sck1 <= sck;  sck2 <= sck1;  sck_q <= sck2;  mo1 <= mosi;  mo2 <= mo1;
            h_wr <= 1'b0;  h_rd <= 1'b0;  frame_err <= 1'b0;

            if (h_rd)        sr   <= h_rdata;                    // fetch
            if (h_wr | h_rd) addr <= addr + {7'b0, ainc};

            if (cs2) begin                                       // CS_N high: idle / end of frame
                if ((st != RD) && (cnt != 6'd0)) frame_err <= 1'b1;   // partial header or partial write word
                st <= HDR;  cnt <= 6'd0;  miso <= 1'b0;
            end else if (rise) begin
                case (st)
                    HDR: begin
                        sr <= shifted;
                        if (cnt == 6'd15) begin
                            st   <= sr[14] ? RD : WR;
                            ainc <= sr[13];
                            addr <= shifted[7:0];
                            cnt  <= 6'd0;
                        end else cnt <= cnt + 6'd1;
                    end
                    WR: begin
                        sr <= shifted;
                        if (cnt == 6'd31) begin h_wr <= 1'b1;  cnt <= 6'd0; end
                        else cnt <= cnt + 6'd1;
                    end
                    default: begin                               // RD: 40 clocks per word
                        if (cnt == 6'd0) h_rd <= 1'b1;
                        if (cnt >= 6'd7 && cnt <= 6'd38) begin miso <= sr[31];  sr <= {sr[30:0], 1'b0}; end
                        else if (cnt == 6'd39) miso <= 1'b0;
                        cnt <= (cnt == 6'd39) ? 6'd0 : cnt + 6'd1;
                    end
                endcase
            end
        end
    end
endmodule
