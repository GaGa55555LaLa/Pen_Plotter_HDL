
module pen_plotter(clk, reset, send_data_button, collision_in, btm_stop_motor, btm_angle_15,
btm_start, btm_dir, top_stop_motor, top_angle_15, top_start, top_dir, servo_pwm, collision_out, btm_IN1, 
btm_IN2, btm_IN3, btm_IN4, btm_ENA, btm_ENB, btm_done, top_IN1, top_IN2, top_IN3, top_IN4, top_ENA, top_ENB, top_done);

// inputs
input             clk;
input             reset;
// input             rx;
// input [7:0]       switches;
input             start;
input             send_data_button;
input             collision_in;
input             btm_stop_motor;
input [3:0]       btm_angle_15;
input             btm_start;
input             btm_dir;
input             top_stop_motor;
input [3:0]       top_angle_15;
input             top_start;
input             top_dir;

// outputs
output wire       servo_pwm;
// output wire  [7:0] led;
// output wire       tx;
output wire        collision_out;
output wire        btm_IN1;
output wire        btm_IN2;
output wire        btm_IN3;
output wire        btm_IN4;
output wire        btm_ENA;
output wire        btm_ENB;
output wire        btm_done;
output wire        top_IN1;
output wire        top_IN2;
output wire        top_IN3;
output wire        top_IN4;
output wire        top_ENA;
output wire        top_ENB;
output wire        top_done;

// wire
wire               rst_pb;
wire               rst_op;
wire               top_start_pb;
wire               top_start_op;
wire               btm_start_pb;
wire               btm_start_op;
wire               start_pb;
wire               start_op;
wire [7:0]         led;
wire               tx;
wire               rx;
wire [7:0]         switches;
wire               data_ready;

// reg

debounce d0(rst_pb, reset, clk);
onepulse d1(rst_pb, clk, rst_op);
debounce d2(top_start_pb, top_start, clk);
onepulse d3(top_start, clk, top_start_op);
debounce d4(btm_start_pb, btm_start, clk);
onepulse d5(btm_start, clk, btm_start_op);
debounce d6(start_pb, start, clk);
onepulse d7(start, clk, start_op);

servo servo_motor(clk, rst_op, servo_pwm);
uart_top uart(clk, rx, rst_op, switches, send_data_button, led, tx, data_ready, tell_py_start_to_send_data);
limit_switch limit_sw(clk, collision_in, collision_out);
stepper_motor_bottom motor_btm(clk, btm_stop_motor, btm_angle_15, btm_start_op, btm_dir, btm_IN1, btm_IN2, btm_IN3, btm_IN4, btm_ENA, btm_done);
stepper_motor_top motor_top(clk, top_stop_motor, top_angle_15, top_start_op, top_dir, top_IN1, top_IN2, top_IN3, top_IN4, top_ENA, top_done);
fsm fsm();
endmodule

module fsm (clk, start_op);
input              clk;
input              start_op;
parameter IDLE                  = 4'd0;   // 閒置
parameter RECEIVE_X_DIR         = 4'd1;   // 接收 X 軸方向
parameter SEND_X_DIR_COMPLETE   = 4'd2;   // 發送 X 軸方向接收完畢
parameter RECEIVE_X_STEP        = 4'd3;   // 接收 X 軸步數
parameter SEND_X_STEP_COMPLETE  = 4'd4;   // 發送 X 軸步數接收完畢
parameter RECEIVE_Y_DIR         = 4'd5;   // 接收 Y 軸方向
parameter SEND_Y_DIR_COMPLETE   = 4'd6;   // 發送 Y 軸方向接收完畢
parameter RECEIVE_Y_STEP        = 4'd7;   // 接收 Y 軸步數
parameter SEND_Y_STEP_COMPLETE  = 4'd8;   // 發送 Y 軸步數接收完畢
parameter RECEIVE_Z             = 4'd9;   // 接收 Z 軸狀態
parameter SEND_Z_COMPLETE       = 4'd10;  // 發送 Z 軸狀態接收完畢
parameter WAIT_TO_STABLE        = 4'd11;  // 等待訊號穩定
parameter EXECUTION             = 4'd12;  // 執行指令 
parameter EXECUTION_COMPLETE    = 4'd13;  // 完成指令並回傳

wire [3:0]         state;
wire [3:0]         next_state;

always@(posedge clk) begin
    if(rst_op) begin
        state <= IDLE:
    end
    else begin
        state <= next_state;
    end
end

assign tell_py_start_to_send_data = start_op;

always@(*) begin
    case(state)
        IDLE: begin
            // TODO: 歸零

            if(start_op) next_state = RECEIVE_X_DIR;
            else next_state = IDLE;
        end                
        RECEIVE_X_DIR: begin
            
        end       

        SEND_X_DIR_COMPLETE: begin
        end 
        RECEIVE_X_STEP: begin
        end      
        SEND_X_STEP_COMPLETE: begin
        end
        RECEIVE_Y_DIR: begin
        end       
        SEND_Y_DIR_COMPLETE: begin
        end 
        RECEIVE_Y_STEP: begin
        end      
        SEND_Y_STEP_COMPLETE: begin
        end
        RECEIVE_Z: begin
        end           
        SEND_Z_COMPLETE: begin
        end     
        WAIT_TO_STABLE: begin
        end      
        EXECUTION: begin
            // TODO: 處理碰撞邊界(直接下一步)
            // TODO: 一般執行:
        end           
        EXECUTION_COMPLETE: begin
            // TODO: 等待幾個 cycle 回到 RECEIVE_X_DIR
        end  
    endcase
end

endmodule

module debounce (pb_debounced, pb, clk);
    output pb_debounced;
    input pb;
    input clk;
    reg [4:0] DFF;

    always @(posedge clk) begin
        DFF[4:1] <= DFF[3:0];
        DFF[0] <= pb; 
    end
    assign pb_debounced = (&(DFF)); 
endmodule

module onepulse (PB_debounced, clk, PB_one_pulse);
    input PB_debounced;
    input clk;
    output reg PB_one_pulse;
    reg PB_debounced_delay;

    always @(posedge clk) begin
        PB_one_pulse <= PB_debounced & (!PB_debounced_delay);
        PB_debounced_delay <= PB_debounced;
    end 
endmodule