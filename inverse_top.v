`timescale 1ns / 1ps
module inverse_top #(
    parameter DATA_WIDTH           = 16,
    parameter LATENCY              = 2,
    parameter BRAM_RD_ADDR_WIDTH   = 10,
    parameter BRAM_WR_ADDR_WIDTH   = 10,
    parameter BRAM_RD_INCREASE     = 4,
    parameter MIC_NUM              = 8,
    parameter SOR_NUM              = 2,
    parameter FREQ_NUM             = 257,
    parameter signed [DATA_WIDTH-1:0] LAMBDA = 16'sh0000A4 // signed 164, s10.14
)(
    input                                      clk,
    input                                      rst_n,
    input                                      start,

    // read bram data
    input      signed [DATA_WIDTH-1:0]         af_bram_rd_real,
    input      signed [DATA_WIDTH-1:0]         af_bram_rd_imag,
    output reg        [BRAM_RD_ADDR_WIDTH-1:0] bram_rd_addr,

    // write bram data
    output reg signed [DATA_WIDTH-1:0]         result_bram_wr_real,
    output reg signed [DATA_WIDTH-1:0]         result_bram_wr_imag,
    output reg        [BRAM_WR_ADDR_WIDTH-1:0] bram_wr_addr,

    // connect to divider
    input      signed [] s_axis_din_tdata,
    input                s_axis_din_tvalid,
    output reg signed [] m_axis_dividend_tdata,
    output reg           m_axis_dividend_tvalid,
    output reg signed [] m_axis_divisor_tdata,
    output reg           m_axis_divisor_tvalid,

    output reg                                 done
);

    localparam S_IDLE        = 0;
    localparam S_RD          = 1; // AH * A
    localparam S_UPDATE_ADDR = 2; 
    localparam S_PLUS        = 3; // G = AH * A + lambda * I
    localparam S_CALDET1     = 4; // det
    localparam S_CALDET2     = 5; // G -> G^-1
    localparam S_INVDET      = 6; // W = G^-1 * AH
    localparam S_DONE        = 7;

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
    reg [2:0] sor0_cnt;
    reg [3:0] rd_cnt;
    reg       flag_rd_sor1;

    reg signed [DATA_WIDTH-1:0] sor0_temp_real [0:MIC_NUM-1];
    reg signed [DATA_WIDTH-1:0] sor0_temp_imag [0:MIC_NUM-1];

    reg signed [DATA_WIDTH*2-1:0] g11_real_acc;
    reg signed [DATA_WIDTH*2-1:0] g12_real_acc;
    reg signed [DATA_WIDTH*2-1:0] g12_imag_acc;
    reg signed [DATA_WIDTH*2-1:0] g21_real_acc;
    reg signed [DATA_WIDTH*2-1:0] g22_real_acc;

    wire signed [DATA_WIDTH*2-1:0] g12_real_acc_sqr;
    wire signed [DATA_WIDTH*2-1:0] g12_imag_acc_sqr;

    assign g12_real_acc_sqr = g12_real_acc * g12_real_acc;
    assign g12_imag_acc_sqr = g12_imag_acc * g12_imag_acc;
    
    reg signed [DATA_WIDTH*2-1:0] det;
    reg signed [DATA_WIDTH*2-1:0] inv_det;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
        end else begin
            state <= next_state;
        end
    end

    always @(*) begin
        case (state)
            S_IDLE:        next_state = (start_delay[LATENCY]) ? S_RD : S_IDLE; 
            S_RD:          next_state = (rd_cnt == PER_FREQ - 1) ? S_PLUS : S_UPDATE_ADDR;
            S_UPDATE_ADDR: next_state = S_RD;
            S_PLUS:        next_state = S_CALDET1;
            S_CALDET1:     next_state = S_CALDET2;
            S_CALDET2:     next_state = S_INVDET;
            S_INVDET:      next_state = 
            S_DONE: 
            default: next_state = S_IDLE; 
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bram_rd_addr <= 0;
            flag_rd_sor1 <= 1'b0;
            sor0_cnt     <= 3'd0;
            rd_cnt       <= 4'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start_delay[LATENCY]) begin
                        
                    end
                end
                S_RD: begin
                    rd_cnt <= (rd_cnt == PER_FREQ - 1) ? rd_cnt : rd_cnt + 1;
                    if (flag_rd_sor1) begin
                        g22_real_acc <= g22_real_acc + af_bram_rd_real * af_bram_rd_real 
                                                     + af_bram_rd_imag * af_bram_rd_imag;
                        g12_real_acc <= g12_real_acc + sor0_temp_real[sor0_cnt] * af_bram_rd_real 
                                                     + sor0_temp_imag[sor0_cnt] * af_bram_rd_imag;
                        g12_imag_acc <= g12_imag_acc - sor0_temp_real[sor0_cnt] * af_bram_rd_imag 
                                                     - sor0_temp_imag[sor0_cnt] * af_bram_rd_real;
                    end else begin
                        sor0_temp_real[sor0_cnt] <= af_bram_rd_real;
                        sor0_temp_imag[sor0_cnt] <= af_bram_rd_imag;
                        g11_real_acc             <= g11_real_acc + sor0_temp_real[sor0_cnt] * sor0_temp_real[sor0_cnt] 
                                                                 + sor0_temp_imag[sor0_cnt] * sor0_temp_imag[sor0_cnt];
                    end
                end
                S_UPDATE_ADDR: begin
                    sor0_cnt     <= (sor0_cnt == MIC_NUM - 1) ? 0 : sor0_cnt + 1;
                    flag_rd_sor1 <= (sor0_cnt == MIC_NUM - 1) ? ~flag_rd_sor1 : flag_rd_sor1; 
                    bram_rd_addr <= bram_rd_addr + BRAM_RD_INCREASE;
                end 
                S_PLUS: begin
                    rd_cnt       <= 0;
                    g11_real_acc <= g11_real_acc + $signed(LAMBDA);
                    g22_real_acc <= g22_real_acc + $signed(LAMBDA);
                end 
                S_CALDET1: begin
                    det <= g11_real_acc * g22_real_acc;
                end 
                S_CALDET2: begin
                    det <= det - (g12_real_acc_sqr + g12_imag_acc_sqr);
                end
                S_INVDET: begin
                    inv_det <= 1/det;
                end
                S_DONE: begin
                    done <= 1;
                end
                default: begin
                    
                end
            endcase
        end
    end
endmodule