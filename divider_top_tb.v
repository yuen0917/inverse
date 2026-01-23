`timescale 1ns / 1ps
module divider_top_tb ();
    reg                clk;
    reg                rst_n;
    reg                start;
    reg  signed [31:0] divisor_data;
    reg  signed [31:0] dividend_data;

    // result_data: {32-bit signed quotient, 16-bit signed fractional}
    wire signed [47:0] result_data;
    wire               result_valid;

    design_1 u_design_1 (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .divisor_data(divisor_data),
        .dividend_data(dividend_data),
        .result_data(result_data),
        .result_valid(result_valid)
    );

    localparam PERIOD = 10;

    always #(PERIOD/2) clk = ~clk;

    // Test one division: value = dividend / divisor
    // t_expected is {32-bit signed quotient, 16-bit signed fractional}
    task automatic do_test;
        input signed [31:0]  t_divisor;
        input signed [31:0]  t_dividend;
        input signed [47:0]  t_expected;
        real r_result, r_expected;
    begin
        @(negedge clk);
        divisor_data  <= t_divisor;
        dividend_data <= t_dividend;

        start <= 1'b1;
        @(negedge clk);
        start <= 1'b0;

        wait (result_valid == 1'b1);

        // Decode {quotient, fractional} to real
        r_result   = $itor(result_data[47:16]) +
                     $itor(result_data[15:0]) / 65536.0;
        r_expected = $itor(t_expected[47:16]) +
                     $itor(t_expected[15:0]) / 65536.0;

        if (result_data === t_expected) begin
            $display("[%0t] PASS: %0d / %0d",
                     $time, t_dividend, t_divisor);
            $display("  result   = %b (q=%0d, f=%0d) = %0.6f",
                     result_data, result_data[47:16], result_data[15:0], r_result);
            $display("  expected = %b (q=%0d, f=%0d) = %0.6f",
                     t_expected,  t_expected[47:16],  t_expected[15:0],  r_expected);
        end else begin
            $display("[%0t] FAIL: %0d / %0d",
                     $time, t_dividend, t_divisor);
            $display("  result   = %b (q=%0d, f=%0d) = %0.6f",
                     result_data, result_data[47:16], result_data[15:0], r_result);
            $display("  expected = %b (q=%0d, f=%0d) = %0.6f",
                     t_expected,  t_expected[47:16],  t_expected[15:0],  r_expected);
        end

        @(negedge clk);
    end
    endtask

    initial begin
        clk          = 0;
        rst_n        = 0;
        start        = 0;
        divisor_data = 0;
        dividend_data= 0;

        #(PERIOD*2) rst_n = 1;

        // Test 1: 100 / 5 = 20.0 -> quotient=20, fractional=0
        #(PERIOD*2);
        do_test(32'sd5,   32'sd100, 48'h00000014_0000);

        // Test 2: 7 / 3 ≈ 2.3333 -> quotient=2, fractional≈0.3333 -> 0x5555
        do_test(32'sd3,   32'sd7,   48'h00000002_5555);

        // Test 3: -50 / 4 = -12.5 -> quotient=-12 (0xFFFF_FFF4), fractional=-0.5 (0x8000)
        do_test(32'sd4,  -32'sd50,  48'hFFFF_FFF4_8000);

        // Test 4: -9 / -3 = 3.0 -> quotient=3, fractional=0
        do_test(-32'sd3, -32'sd9,   48'h00000003_0000);

        $display("All tests finished");
        $finish;
    end

endmodule