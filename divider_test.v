`timescale 1ns / 1ps
module divider_test #(
    parameter DOUT_TDATA_WIDTH     = 48,
    parameter DIVISOR_TDATA_WIDTH  = 32, 
    parameter DIVIDEND_TDATA_WIDTH = 32
)(
    input clk,
    input rst_n,
    input start,
    input      signed [DIVISOR_TDATA_WIDTH-1:0]  divisor_data,
    input      signed [DIVIDEND_TDATA_WIDTH-1:0] dividend_data,
    input      signed [DOUT_TDATA_WIDTH-1:0]     m_axis_dout_tdata,
    input                                        m_axis_dout_tvalid,
    output reg signed [DIVISOR_TDATA_WIDTH-1:0]  s_axis_divisor_tdata,
    output reg                                   s_axis_divisor_tvalid,
    output reg signed [DIVIDEND_TDATA_WIDTH-1:0] s_axis_dividend_tdata,
    output reg                                   s_axis_dividend_tvalid,

    output reg signed [DOUT_TDATA_WIDTH-1:0]     result_data,
    output reg                                   result_valid
);

    localparam S_IDLE = 0;
    localparam S_SET  = 1;
    localparam S_WAIT = 2;

    reg [1:0] state;
    reg [1:0] next_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
        end else begin
            state <= next_state;
        end
    end

    always @(*) begin
        case(state)
            S_IDLE:  next_state <= (start) ? S_SET : S_IDLE;
            S_SET:   next_state <= S_WAIT;
            S_WAIT:  next_state <= (m_axis_dout_tvalid) ? S_IDLE : S_WAIT;
            default: next_state <= S_IDLE;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axis_divisor_tdata   <= 0;
            s_axis_divisor_tvalid  <= 0;
            s_axis_dividend_tdata  <= 0;
            s_axis_dividend_tvalid <= 0;
            result_data            <= 0;
            result_valid           <= 0;
        end else begin
            result_valid           <= 0; // default for 0
            case(state)
                S_IDLE: begin
                    s_axis_divisor_tvalid  <= 0;
                    s_axis_dividend_tvalid <= 0;
                    if (start) begin
                        s_axis_divisor_tdata  <= divisor_data;
                        s_axis_dividend_tdata <= dividend_data;
                    end
                end
                S_SET: begin
                    s_axis_divisor_tvalid  <= 1;
                    s_axis_dividend_tvalid <= 1; 
                end
                S_WAIT: begin
                    s_axis_divisor_tvalid  <= 0;
                    s_axis_dividend_tvalid <= 0; 
                    if (m_axis_dout_tvalid) begin
                        result_data  <= m_axis_dout_tdata;
                        result_valid <= 1;
                    end
                end
                default: begin
                    s_axis_divisor_tdata   <= 0;
                    s_axis_divisor_tvalid  <= 0;
                    s_axis_dividend_tdata  <= 0;
                    s_axis_dividend_tvalid <= 0;
                    result_data            <= 0;
                    result_valid           <= 0;
                end
            endcase
        end
    end

endmodule