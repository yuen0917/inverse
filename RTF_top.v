`timescale 1ns / 1ps
module RTF_top #(
    parameter MIC_NUM              = 8,
    parameter SOR_NUM              = 2,
    parameter FREQ_NUM             = 257,
    parameter DATA_WIDTH           = 16,
    parameter LATENCY              = 2,
    parameter BRAM_RD_ADDR_WIDTH   = 32,
    parameter BRAM_WR_ADDR_WIDTH   = 32,
    parameter BRAM_RD_ADDR_BASE    = 0,
    parameter BRAM_WR_ADDR_BASE    = 0,
    parameter BRAM_RD_INCREASE     = 2, // 16 / 8 = 2
    parameter BRAM_WR_INCREASE     = 6, // 48 / 8 = 6
    parameter BRAM_WR_WE_WIDTH     = 6,
    parameter DIVOUT_TDATA_WIDTH   = 64,
    parameter DIVOUT_F_WIDTH       = 32,
    parameter DIVISOR_TDATA_WIDTH  = 32,
    parameter DIVIDEND_TDATA_WIDTH = 32,
    parameter LAMBDA               = 0
)(
    input                                        clk,
    input                                        rst_n,
    input                                        start,
    output reg                                   done,
    output reg                                   all_freq_finish,

    // read bram data
    input      signed [DATA_WIDTH-1:0]           af_bram_rd_real,
    input      signed [DATA_WIDTH-1:0]           af_bram_rd_imag,
    output reg        [BRAM_RD_ADDR_WIDTH-1:0]   bram_rd_addr,

    // write bram data
    output reg signed [DATA_WIDTH*4-1:0]         result_bram_wr_real,
    output reg signed [DATA_WIDTH*4-1:0]         result_bram_wr_imag,
    output reg        [BRAM_WR_ADDR_WIDTH-1:0]   bram_wr_addr,
    output            [BRAM_WR_WE_WIDTH-1:0]     bram_wr_we,
    output                                       bram_wr_en,

    // from divider
    input      signed [DIVOUT_TDATA_WIDTH-1:0]   m_axis_dout_tdata,
    input                                        m_axis_dout_tvalid,

    // to divider
    output reg signed [DIVIDEND_TDATA_WIDTH-1:0] s_axis_dividend_tdata,
    output reg                                   s_axis_dividend_tvalid,
    output reg signed [DIVISOR_TDATA_WIDTH-1:0]  s_axis_divisor_tdata,
    output reg                                   s_axis_divisor_tvalid
);

    localparam S_IDLE           = 0;  // wait start
    localparam S_RD             = 1;  // read bram data
    localparam S_UPDATE_RD_ADDR = 2;  // update bram read address
    localparam S_PLUS           = 3;  // plus lambda * I to G
    localparam S_CALDET1        = 4;  // calculate g11 * g22
    localparam S_CALDET2        = 5;  // calculate det - (g12_real_acc_sqr + g12_imag_acc_sqr)
    localparam S_INVDET         = 6;  // set dividend and divisor to divider
    localparam S_SETDIV         = 7;  // set dividend and divisor valid to divider
    localparam S_WAITDIV        = 8;  // wait divider result
    localparam S_CALINVG        = 9;  // calculate inverse of G
    localparam S_CALRESULT      = 10; // calculate result elements
    localparam S_WR             = 11; // write result to bram
    localparam S_UPDATE_WR_ADDR = 12; // update bram write address
    localparam S_DONE           = 13; // done

    localparam TOTAL_NUM = MIC_NUM * SOR_NUM * FREQ_NUM;
    localparam PER_FREQ  = MIC_NUM * SOR_NUM;

    // ==============================
    // bram start delay
    // ==============================
    reg [LATENCY:0] start_delay;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_delay <= 0;
        end else begin
            start_delay <= {start_delay[LATENCY-1:0], start};
        end
    end

    // ==============================
    // FSM
    // ==============================
    reg [3:0] state;
    reg [3:0] next_state;
    reg [2:0] sor_cnt;
    reg [3:0] rd_cnt;
    reg [3:0] wr_cnt;
    reg       flag_rd_sor1;
    reg       result_row1;
    reg [8:0] freq_sample_cnt;

    assign bram_wr_we = (state == S_WR) ? {BRAM_WR_WE_WIDTH{1'b1}} : {BRAM_WR_WE_WIDTH{1'b0}};
    assign bram_wr_en = (state == S_WR);

    reg signed [DATA_WIDTH-1:0] sor0_temp_real [0:MIC_NUM-1];
    reg signed [DATA_WIDTH-1:0] sor0_temp_imag [0:MIC_NUM-1];
    reg signed [DATA_WIDTH-1:0] sor1_temp_real [0:MIC_NUM-1];
    reg signed [DATA_WIDTH-1:0] sor1_temp_imag [0:MIC_NUM-1];

    // G = a(f)h * a(f) + lambda * I register
    // note1: g21 = g12 conjugate, so we only need store g11, g12, g22.
    // note2: g11 and g22 only have real part, so we only need store real part of g11 and g22
    reg signed [DATA_WIDTH*2-1:0] g11_real_acc;
    reg signed [DATA_WIDTH*2-1:0] g12_real_acc;
    reg signed [DATA_WIDTH*2-1:0] g12_imag_acc;
    reg signed [DATA_WIDTH*2-1:0] g22_real_acc;

    // square of g12 (avoid timing violation)
    wire signed [DATA_WIDTH*2-1:0] g12_real_acc_sqr;
    wire signed [DATA_WIDTH*2-1:0] g12_imag_acc_sqr;

    assign g12_real_acc_sqr = g12_real_acc * g12_real_acc;
    assign g12_imag_acc_sqr = g12_imag_acc * g12_imag_acc;

    // det
    reg  signed [DATA_WIDTH*2-1:0]         det;
    reg  signed [DIVIDEND_TDATA_WIDTH-1:0] inv_det_q;
    reg  signed [DIVOUT_F_WIDTH-1:0]       inv_det_f;
    wire signed [DIVOUT_TDATA_WIDTH-1:0]   inv_det;

    // inverse G register
    reg signed [DATA_WIDTH*3-1:0] inv_g11_real;
    reg signed [DATA_WIDTH*3-1:0] inv_g12_real;
    reg signed [DATA_WIDTH*3-1:0] inv_g12_imag;
    reg signed [DATA_WIDTH*3-1:0] inv_g22_real;

    // result elements (avoid timing violation)
    reg signed [DATA_WIDTH*3-1:0] result_real_element0;
    reg signed [DATA_WIDTH*3-1:0] result_real_element1;
    reg signed [DATA_WIDTH*3-1:0] result_real_element2;
    reg signed [DATA_WIDTH*3-1:0] result_imag_element0;
    reg signed [DATA_WIDTH*3-1:0] result_imag_element1;
    reg signed [DATA_WIDTH*3-1:0] result_imag_element2;


    assign inv_det = ($signed(inv_det_q) <<< (DIVOUT_F_WIDTH - 1)) + $signed(inv_det_f);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
        end else begin
            state <= next_state;
        end
    end

    always @(*) begin
        case (state)
            S_IDLE:           next_state = (start_delay[LATENCY]) ? S_RD : S_IDLE;
            S_RD:             next_state = (rd_cnt == PER_FREQ - 1) ? S_PLUS : S_UPDATE_RD_ADDR;
            S_UPDATE_RD_ADDR: next_state = S_RD;
            S_PLUS:           next_state = S_CALDET1;
            S_CALDET1:        next_state = S_CALDET2;
            S_CALDET2:        next_state = S_INVDET;
            S_INVDET:         next_state = S_SETDIV;
            S_SETDIV:         next_state = S_WAITDIV;
            S_WAITDIV:        next_state = (m_axis_dout_tvalid) ? S_CALINVG : S_WAITDIV;
            S_CALINVG:        next_state = S_CALRESULT;
            S_CALRESULT:      next_state = S_WR;
            S_WR:             next_state = (wr_cnt == PER_FREQ - 1) ? S_DONE : S_UPDATE_WR_ADDR;
            S_UPDATE_WR_ADDR: next_state = S_CALRESULT;
            S_DONE:           next_state = S_IDLE;
            default:          next_state = S_IDLE;
        endcase
    end

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // bram addr
            bram_rd_addr <= BRAM_RD_ADDR_BASE;
            bram_wr_addr <= BRAM_WR_ADDR_BASE;

            // sor0 and sor1 temp register
            for (i = 0; i < MIC_NUM; i = i + 1) begin
                sor0_temp_real[i] <= 0;
                sor0_temp_imag[i] <= 0;
                sor1_temp_real[i] <= 0;
                sor1_temp_imag[i] <= 0;
            end
            flag_rd_sor1 <= 0;
            result_row1  <= 0;

            // G register
            g11_real_acc <= 0;
            g12_real_acc <= 0;
            g12_imag_acc <= 0;
            g22_real_acc <= 0;

            // det register
            det          <= 0;
            inv_det_q    <= 0;
            inv_det_f    <= 0;

            // to divider
            s_axis_dividend_tdata  <= 0;
            s_axis_dividend_tvalid <= 0;
            s_axis_divisor_tdata   <= 0;
            s_axis_divisor_tvalid  <= 0;

            // counter
            sor_cnt         <= 3'd0;
            rd_cnt          <= 4'd0;
            wr_cnt          <= 4'd0;
            freq_sample_cnt <= 9'd0;

            // result
            result_real_element0 <= 0;
            result_real_element1 <= 0;
            result_real_element2 <= 0;
            result_imag_element0 <= 0;
            result_imag_element1 <= 0;
            result_imag_element2 <= 0;
            result_bram_wr_real  <= 0;
            result_bram_wr_imag  <= 0;
            all_freq_finish      <= 0;
            done                 <= 0;
        end else begin
            case (state)
                S_IDLE: begin // state 0
                    if (start_delay[LATENCY]) begin
                        sor_cnt    <= 0;
                        rd_cnt     <= 0;
                        for (i = 0; i < MIC_NUM; i = i + 1) begin
                            sor0_temp_real[i] <= 0;
                            sor0_temp_imag[i] <= 0;
                            sor1_temp_real[i] <= 0;
                            sor1_temp_imag[i] <= 0;
                        end
                        g11_real_acc <= 0;
                        g12_real_acc <= 0;
                        g12_imag_acc <= 0;
                        g22_real_acc <= 0;
                        inv_g11_real <= 0;
                        inv_g12_real <= 0;
                        inv_g12_imag <= 0;
                        inv_g22_real <= 0;
                        det          <= 0;
                        inv_det_q    <= 0;
                        inv_det_f    <= 0;
                        done         <= 0;
                        all_freq_finish      <= 0;
                        result_real_element0 <= 0;
                        result_real_element1 <= 0;
                        result_real_element2 <= 0;
                        result_imag_element0 <= 0;
                        result_imag_element1 <= 0;
                        result_imag_element2 <= 0;
                        result_bram_wr_real  <= 0;
                        result_bram_wr_imag  <= 0;
                        flag_rd_sor1         <= 0;
                        result_row1          <= 0;
                    end
                end
                S_RD: begin // state 1
                    rd_cnt <= (rd_cnt == PER_FREQ - 1) ? rd_cnt : rd_cnt + 1;
                    if (flag_rd_sor1) begin
                        sor1_temp_real[sor_cnt] <= af_bram_rd_real;
                        sor1_temp_imag[sor_cnt] <= af_bram_rd_imag;
                        g22_real_acc            <= g22_real_acc + $signed(af_bram_rd_real) * $signed(af_bram_rd_real)
                                                                + $signed(af_bram_rd_imag) * $signed(af_bram_rd_imag);
                        g12_real_acc            <= g12_real_acc + $signed(sor0_temp_real[sor_cnt]) * $signed(af_bram_rd_real)
                                                                + $signed(sor0_temp_imag[sor_cnt]) * $signed(af_bram_rd_imag);
                        g12_imag_acc            <= g12_imag_acc + $signed(sor0_temp_real[sor_cnt]) * $signed(af_bram_rd_imag)
                                                                - $signed(sor0_temp_imag[sor_cnt]) * $signed(af_bram_rd_real);
                    end else begin
                        sor0_temp_real[sor_cnt] <= af_bram_rd_real;
                        sor0_temp_imag[sor_cnt] <= af_bram_rd_imag;
                        g11_real_acc            <= g11_real_acc + $signed(af_bram_rd_real) * $signed(af_bram_rd_real)
                                                                + $signed(af_bram_rd_imag) * $signed(af_bram_rd_imag);
                    end
                end
                S_UPDATE_RD_ADDR: begin // state 2
                    sor_cnt      <= (sor_cnt == MIC_NUM - 1) ? 0 : sor_cnt + 1;
                    flag_rd_sor1 <= (sor_cnt == MIC_NUM - 1) ? ~flag_rd_sor1 : flag_rd_sor1;
                    bram_rd_addr <= bram_rd_addr + BRAM_RD_INCREASE;
                end
                S_PLUS: begin // state 3
                    bram_rd_addr <= bram_rd_addr + BRAM_RD_INCREASE;
                    flag_rd_sor1 <= 0;
                    rd_cnt       <= 0;
                    sor_cnt      <= 0;
                    g11_real_acc <= g11_real_acc + $signed(LAMBDA);
                    g22_real_acc <= g22_real_acc + $signed(LAMBDA);
                end
                S_CALDET1: begin // state 4
                    det <= g11_real_acc * g22_real_acc;
                end
                S_CALDET2: begin // state 5
                    det <= det - (g12_real_acc_sqr + g12_imag_acc_sqr);
                end
                S_INVDET: begin // state 6
                    s_axis_divisor_tdata  <= det;
                    s_axis_dividend_tdata <= 1;
                end
                S_SETDIV: begin // state 7
                    s_axis_divisor_tvalid  <= 1;
                    s_axis_dividend_tvalid <= 1;
                end
                S_WAITDIV: begin // state 8
                    s_axis_divisor_tvalid  <= 0;
                    s_axis_dividend_tvalid <= 0;
                    if (m_axis_dout_tvalid) begin
                        inv_det_q <= m_axis_dout_tdata[DIVOUT_TDATA_WIDTH-1:DIVOUT_F_WIDTH];
                        inv_det_f <= m_axis_dout_tdata[DIVOUT_F_WIDTH-1:0];
                    end
                end
                S_CALINVG: begin // state 9
                    inv_g11_real <=  g22_real_acc * inv_det;
                    inv_g12_real <= -g12_real_acc * inv_det;
                    inv_g12_imag <= -g12_imag_acc * inv_det;
                    inv_g22_real <=  g11_real_acc * inv_det;
                end
                S_CALRESULT: begin // state 10
                    if (result_row1) begin
                        result_real_element0 <=  inv_g12_real * sor0_temp_real[sor_cnt];
                        result_real_element1 <= -inv_g12_imag * sor0_temp_imag[sor_cnt];
                        result_real_element2 <=  inv_g22_real * sor1_temp_real[sor_cnt];
                        result_imag_element0 <= -inv_g12_real * sor0_temp_imag[sor_cnt];
                        result_imag_element1 <= -inv_g12_imag * sor0_temp_real[sor_cnt];
                        result_imag_element2 <= -inv_g22_real * sor1_temp_imag[sor_cnt];
                    end else begin
                        result_real_element0 <=  inv_g11_real * sor0_temp_real[sor_cnt];
                        result_real_element1 <=  inv_g12_real * sor1_temp_real[sor_cnt];
                        result_real_element2 <=  inv_g12_imag * sor1_temp_imag[sor_cnt];
                        result_imag_element0 <= -inv_g11_real * sor0_temp_imag[sor_cnt];
                        result_imag_element1 <= -inv_g12_real * sor1_temp_imag[sor_cnt];
                        result_imag_element2 <=  inv_g12_imag * sor1_temp_real[sor_cnt];
                    end
                end
                S_WR: begin // state 11
                    wr_cnt              <= (wr_cnt == PER_FREQ - 1) ? wr_cnt : wr_cnt + 1;
                    result_bram_wr_real <= result_real_element0 + result_real_element1 + result_real_element2;
                    result_bram_wr_imag <= result_imag_element0 + result_imag_element1 + result_imag_element2;
                end
                S_UPDATE_WR_ADDR: begin // state 12
                    sor_cnt      <= (sor_cnt == MIC_NUM - 1) ? 0 : sor_cnt + 1;
                    result_row1  <= (sor_cnt == MIC_NUM - 1) ? ~result_row1 : result_row1;
                    bram_wr_addr <= bram_wr_addr + BRAM_WR_INCREASE;
                end
                S_DONE: begin // state 13
                    result_row1     <= 0;
                    freq_sample_cnt <= (freq_sample_cnt == FREQ_NUM - 1) ? 0 : freq_sample_cnt + 1;
                    all_freq_finish <= (freq_sample_cnt == FREQ_NUM - 1) ? 1 : 0;
                    sor_cnt         <= 0;
                    wr_cnt          <= 0;
                    done            <= 1;
                    bram_wr_addr    <= bram_wr_addr + BRAM_WR_INCREASE; // for next bram write start
                end
                default: begin

                end
            endcase
        end
    end
endmodule