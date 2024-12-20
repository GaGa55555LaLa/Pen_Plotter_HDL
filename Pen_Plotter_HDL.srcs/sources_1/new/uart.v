module uart_top (clk, rx, rst, send_data_button, tx, data_ready, tell_py_start_to_send_data, rx_data);
    input wire clk;
    input wire rx;
    input wire rst;
    input wire send_data_button;
    input wire tell_py_start_to_send_data;
    output wire tx;
    output wire data_ready;
    output reg [7:0] rx_data;

    wire tx_ready;
    
    uart_receiver uart_rx (
        .clk(clk),
        .rx(rx),
        .rst(rst),
        .rx_data(rx_data),
        .data_ready(data_ready),
    );

    uart_transmitter uart_tx (
        .clk(clk),
        .rst(rst),
        .send(send_data_button),
        .tx(tx),
        .tx_ready(tx_ready),
        .tell_py_start_to_send_data(tell_py_start_to_send_data)
    );

endmodule

module uart_receiver (clk, rx, rst, rx_data, data_ready);

    input wire clk;        
    input wire rx;         
    input wire rst;        
    output reg [7:0] rx_data ;
    output reg data_ready;  

    parameter BAUD_RATE = 230400;
    parameter CLOCK_FREQ = 100000000;
    localparam BAUD_TICK_COUNT = 433;

    reg [15:0] baud_counter;
    reg [3:0] bit_counter;
    reg [7:0] shift_reg;
    reg receiving;
    reg rx_sync1, rx_sync2;    // 保留雙級同步器以防止亞穩態
    
    // 同步rx信號
    always @(posedge clk) begin
        if (rst) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
        end else begin
            rx_sync1 <= rx;
            rx_sync2 <= rx_sync1;
        end
    end

    // 主要接收邏輯
    always @(posedge clk) begin
        if (rst) begin
            baud_counter <= 0;
            bit_counter <= 0;
            shift_reg <= 8'b0;
            receiving <= 0;
            data_ready <= 0;
            rx_data <= 8'b0;
        end else begin
            data_ready <= 0;  // 默認清除data_ready
            if (!receiving && rx_sync2 == 1'b0) begin  // 檢測起始位
                receiving <= 1;
                baud_counter <= BAUD_TICK_COUNT/2;  // 設為半個波特週期
                bit_counter <= 0;
            end else if (receiving) begin
                if (baud_counter == 0) begin
                    baud_counter <= BAUD_TICK_COUNT;
                    
                    if (bit_counter < 9) begin  // 接收8位數據
                        shift_reg <= {rx_sync2, shift_reg[7:1]};
                        bit_counter <= bit_counter + 1;
                    end 
                    else begin  // 接收完成
                        if (rx_sync2 == 1'b1) begin  // 簡單檢查停止位
                            rx_data <= shift_reg;
                            data_ready <= 1;
                        end
                        receiving <= 0;
                        bit_counter <= 0;
                    end
                end else begin
                    baud_counter <= baud_counter - 1;
                end
            end
        end
    end
endmodule

module uart_transmitter (clk, rst, send, tx, tx_ready, tell_py_start_to_send_data);
    input wire clk;
    input wire rst;
    input wire send;
    input tell_py_start_to_send_data;
    output reg tx;
    output reg tx_ready;

    parameter BAUD_RATE = 230400;
    parameter CLOCK_FREQ = 100000000;
    localparam BAUD_TICK_COUNT = 433;

    reg [15:0] baud_counter;
    reg [3:0] bit_counter;
    reg [7:0] shift_reg;
    reg transmitting;

    always @(posedge clk) begin
        if (rst) begin
            baud_counter <= 0;
            bit_counter <= 0;
            tx <= 1'b1;
            transmitting <= 0;
            tx_ready <= 1;
        end else begin
            if (tx_ready && tell_py_start_to_send_data) begin
                transmitting <= 1;
                shift_reg <= 1; // 傳送執行完畢的訊號給電腦
                bit_counter <= 0;
                baud_counter <= BAUD_TICK_COUNT;
                tx <= 0;  // 起始位
                tx_ready <= 0;
            end else if (transmitting) begin
                if (baud_counter == 0) begin
                    baud_counter <= BAUD_TICK_COUNT;
                    if (bit_counter < 8) begin
                        tx <= shift_reg[bit_counter];
                        bit_counter <= bit_counter + 1;
                    end else if (bit_counter == 8) begin
                        tx <= 1;  // 停止位
                        bit_counter <= bit_counter + 1;
                    end else begin
                        transmitting <= 0;
                        tx_ready <= 1;
                    end
                end else begin
                    baud_counter <= baud_counter - 1;
                end
            end
        end
    end
endmodule