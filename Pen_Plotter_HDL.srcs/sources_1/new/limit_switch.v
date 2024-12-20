`timescale 1ns / 1ps

// 按下去是 0
module limit_switch(clk, btn_in, btn_out);
input clk, btn_in;
output btn_out;

assign btn_out = btn_in;
endmodule