`timescale 1ns / 1ps

module limit_switch(
    input clk,        // 時鐘信號
    input reset,      // 重置信號，高有效
    input btn_in,     // 按鈕輸入
    output reg btn_out // 穩定的按鈕輸出
);
    reg [2:0] shift_reg; // 移位暫存器，記錄最近 3 次按鈕狀態
    reg stable_state;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // 初始化所有暫存器
            shift_reg <= 3'b111;    // 預設為高（未按下狀態）
            stable_state <= 1'b1;   // 按鈕穩定狀態設為未按下
            btn_out <= 1'b1;        // 輸出初始化為高
        end else begin
            // 將按鈕輸入存入移位暫存器
            shift_reg <= {shift_reg[1:0], btn_in};

            // 判斷最近 3 次輸入是否一致
            if (shift_reg == 3'b000 || shift_reg == 3'b111) begin
                stable_state <= shift_reg[0];
            end

            // 更新輸出
            btn_out <= stable_state;
        end
    end
endmodule