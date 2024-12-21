
module pen_plotter(clk, reset, rx, start, collision_in, servo_pwm, tx, collision_out, btm_IN1, 
btm_IN2, btm_IN3, btm_IN4, btm_ENA, btm_ENB, top_IN1, top_IN2, top_IN3, top_IN4, top_ENA, top_ENB, AN, seg, start_signal);

// inputs
input             clk;
input             reset;
input             rx;
input             start;
input             collision_in;
// input             btm_stop_motor;
// input [3:0]       btm_angle_15;
input [15:0]      btm_step;
// input             btm_start;
input             btm_dir;
// input             top_stop_motor;
// input [3:0]       top_angle_15;
input [15:0]      top_step;
// input             top_start;
input             top_dir;

// outputs
output wire        servo_pwm;
output wire        tx;
output wire        collision_out;
output wire        btm_IN1;
output wire        btm_IN2;
output wire        btm_IN3;
output wire        btm_IN4;
output wire        btm_ENA;
output wire        btm_ENB;
output wire        top_IN1;
output wire        top_IN2;
output wire        top_IN3;
output wire        top_IN4;
output wire        top_ENA;
output wire        top_ENB;

// for test
output reg [3:0]   AN;
output reg [6:0]   seg;
wire [3:0] state;

// wire
wire               rst_pb;
wire               rst_op;
wire               top_start;
wire               btm_start;
// wire               top_start_pb;
// wire               top_start_op;
// wire               btm_start_pb;
// wire               btm_start_op;
wire               start_pb;
wire               start_op;
wire               data_ready;
wire               tell_py_start_to_send_data;
wire [7:0]         rx_data;
wire        btm_done;
wire        top_done;

// reg

debounce d0(rst_pb, reset, clk);
onepulse d1(rst_pb, clk, rst_op);
// debounce d2(top_start_pb, top_start, clk);
// onepulse d3(top_start, clk, top_start_op);
// debounce d4(btm_start_pb, btm_start, clk);
// onepulse d5(btm_start, clk, btm_start_op);
debounce d6(start_pb, start, clk);
onepulse d7(start_pb, clk, start_op);

servo servo_motor(clk, rst_op, servo_pwm);
uart_top uart(clk, rx, rst_op, send_data_button, tx, data_ready, tell_py_start_to_send_data, rx_data);
limit_switch limit_sw(clk, collision_in, collision_out);
stepper_motor_bottom motor_btm(clk, btm_step, btm_start, btm_dir, btm_IN1, btm_IN2, btm_IN3, btm_IN4, btm_ENA, btm_done);
stepper_motor_top motor_top(clk, top_step, top_start, top_dir, top_IN1, top_IN2, top_IN3, top_IN4, top_ENA, top_done);
fsm fsm(clk, start_op, rst_op, rx_data, tell_py_start_to_send_data, data_ready, state);


reg seg_slow;
reg [1:0] seg_sel;
reg [15:0] seg_counter;

always @(posedge clk) begin
    if (rst_op) begin
        seg_counter <= 16'b0;
        seg_slow <= 1'b0;
    end else begin
        if (seg_counter == 16'b1111111111111111) begin
            seg_counter <= 16'b0;
            seg_slow <= 1'b1;
        end else begin
            seg_counter <= seg_counter + 1'b1;
            seg_slow <= 1'b0;
        end
    end
end

always @(posedge clk) begin
    if (rst_op) begin
        seg_sel <= 2'b00;
    end else if (seg_slow) begin
        seg_sel <= seg_sel + 1'b1;
    end else begin
        seg_sel <= seg_sel;
    end
end

always @(*) begin
    case (seg_sel)
        2'b00: AN = 4'b0111;
        2'b01: AN = 4'b1011;
        2'b10: AN = 4'b1101;
        2'b11: AN = 4'b1110;
        default: AN = 4'b1111;
    endcase
end

always @(*) begin
    case (state)
            4'b0000: seg = 7'b0000001;  // Display 0
            4'b0001: seg = 7'b1001111;  // Display 1
            4'b0010: seg = 7'b0010010;  // Display 2
            4'b0011: seg = 7'b0000110;  // Display 3
            4'b0100: seg = 7'b1001100;  // Display 4
            4'b0101: seg = 7'b0100100;  // Display 5
            4'b0110: seg = 7'b0100000;  // Display 6
            4'b0111: seg = 7'b0001111;  // Display 7
            4'b1000: seg = 7'b0000000;  // Display 8zzzzzzzzz
            4'b1001: seg = 7'b0000100;  // Display 9
            4'b1010: seg = 7'b0001000;  // Display A
            4'b1011: seg = 7'b1100000;  // Display B
            4'b1100: seg = 7'b0110001;  // Display C
            4'b1101: seg = 7'b1000010;  // Display D
            4'b1110: seg = 7'b0110000;  // Display E
            4'b1111: seg = 7'b0111000;  // Display F
            default: seg = 7'b1111111;  // Turn off all segments (blank display)
    endcase
end

output start_signal;
assign start_signal = start;

endmodule

module fsm (clk, start_op, rst_op, btm_step, btm_start, btm_dir, btm_done, top_step, top_start, top_dir, top_done, rx_data, tell_py_start_to_send_data, data_ready, state);

input              clk;
input              start_op;
input              rst_op;
input [7:0]        rx_data;
input              data_ready;
input              btm_done, top_done;

output reg  [3:0]         state;
reg  [3:0]         next_state;
reg  [6:0]         stable_counter;
reg  [7:0]         X_step;
reg                X_dir;
reg  [7:0]         Y_step;
reg                Y_dir; 
reg                Z_axis; // 1  上升 / 0 下降
reg  [7:0]         next_X_step;
reg                next_X_dir;
reg  [7:0]         next_Y_step;
reg                next_Y_dir; 
reg                next_Z_axis;

output reg         tell_py_start_to_send_data;
output [15:0]      btm_step, top_step;
output             btm_start, top_start;
output             btm_dir, top_dir;
//for test

parameter IDLE                  = 4'd0;   // 閒置
parameter RESETTING_L           = 4'd14;  // 往回走 未接觸 limit switch
parameter RESETTING_R           = 4'd15;  // 往中心走 已接觸 limit switch
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
wire [6:0]         counter;
wire [6:0]         next_counter;

always@(posedge clk) begin
    if(rst_op) begin
        state <= IDLE:
        counter <= 7'b0;
    end else begin
        state <= next_state;
        counter <= next_counter;
        X_step <= next_X_step;
        Y_step <= next_Y_step;
        Z_axis <= next_Z_axis;
        X_dir <= next_X_dir;
        Y_dir <= next_Y_dir;
        tell_py_start_to_send_data <= (start_op || state == SEND_X_DIR_COMPLETE || state == SEND_X_STEP_COMPLETE || state == SEND_Y_DIR_COMPLETE || state == SEND_Y_STEP_COMPLETE);
    end
end


always@(posedge clk) begin
    if(rst_op) begin
        stable_counter <= 0;
    end
    else if(state == WAIT_TO_STABLE)begin
        if(stable_counter >= 127) begin
            stable_counter <= 0;
        end
        else begin
            stable_counter <= stable_counter + 1;
        end 
    end
    else begin
        stable_counter <= 0;
    end
end

always@(*) begin
    case(state)
        IDLE: begin
            // TODO: 歸零
            if (rst_op) begin
                top_start = 1'b0;
                btm_start = 1'b0;
                next_state = RESETTING_L;
            end
            if(start_op) next_state = RECEIVE_X_DIR;
            else next_state = IDLE;
        end    
        RESETTING_L: begin
            top_start = 1'b1;
            top_dir = 1'b0;  //spin left (CW)
            top_step = 16'd438;   //大約7公分
            if (collision_out == 1'b0) begin
                top_start = 1'b0;
                next_state = RESETTING_R;
            end
        end    
        RESETTING_R: begin
            top_start = 1'b1;
            top_dir = 1'b1;  //spin right (CCE)
            top_step = 16'd313;   //大約5公分
            if (top_done) begin
                top_start = 1'b0;
                next_state = IDLE;
            end
        end        
        RECEIVE_X_DIR: begin
            if(data_ready) next_state = SEND_X_DIR_COMPLETE;
            else next_state = RECEIVE_X_DIR;
            next_X_step = 0;
            next_X_dir = rx_data;
            next_Y_step = 0;
            next_Y_dir = 0; 
            next_Z_axis = 0;
        end       
        SEND_X_DIR_COMPLETE: begin
            if(tell_py_start_to_send_data) next_state = RECEIVE_X_STEP;
            else next_state = SEND_X_DIR_COMPLETE;
        end 
        RECEIVE_X_STEP: begin
            if(data_ready) next_state = SEND_X_STEP_COMPLETE;
            else next_state = RECEIVE_X_STEP;
            next_X_step = rx_data;
            next_Y_step = 0;
            next_Y_dir = 0; 
            next_Z_axis = 0;
        end      
        SEND_X_STEP_COMPLETE: begin
            if(tell_py_start_to_send_data) next_state = RECEIVE_Y_DIR;
            else next_state = SEND_X_STEP_COMPLETE;
        end
        RECEIVE_Y_DIR: begin
            if(data_ready) next_state = SEND_Y_DIR_COMPLETE;
            else next_state = RECEIVE_Y_DIR;
            next_Y_step = 0;
            next_Y_dir = rx_data; 
            next_Z_axis = 0;
        end       
        SEND_Y_DIR_COMPLETE: begin
            if(tell_py_start_to_send_data) next_state = RECEIVE_Y_STEP;
            else next_state = SEND_Y_DIR_COMPLETE;
        end 
        RECEIVE_Y_STEP: begin
            if(data_ready) next_state = SEND_Y_STEP_COMPLETE;
            else next_state = RECEIVE_Y_STEP;
            next_Y_step = rx_data;
            next_Z_axis = 0;
        end      
        SEND_Y_STEP_COMPLETE: begin
            if(tell_py_start_to_send_data) next_state = RECEIVE_Z;
            else next_state = SEND_Y_STEP_COMPLETE;
        end
        RECEIVE_Z: begin
            if(data_ready) next_state = SEND_Z_COMPLETE;
            else next_state = RECEIVE_Z;
            next_Z_axis = rx_data;
        end           
        SEND_Z_COMPLETE: begin
            next_state = WAIT_TO_STABLE;
        end     
        WAIT_TO_STABLE: begin
            if(stable_counter == 19) next_state = EXECUTION;
            else next_state = WAIT_TO_STABLE;
        end      
        EXECUTION: begin
            // TODO: 處理碰撞邊界(直接下一步)
            if (collision_out == 1'b0) begin
                top_start = 1'b0;
                btm_start = 1'b0;
                next_state = RECEIVE_X_DIR;
            end
            // TODO: 一般執行:
            top_start = 1'b1;
            top_dir = X_dir;
            top_step = X_step;
            btm_start = 1'b1;
            btm_dir = Y_dir;
            btm_step = Y_dir;
            if (top_done && btm_done) begin
                top_start = 1'b0;
                btm_start = 1'b0;
                next_state = EXECUTION_COMPLETE;
            end
        end           
        EXECUTION_COMPLETE: begin
            // TODO: 等待幾個 cycle 回到 RECEIVE_X_DIR
            if (counter == ~7'b0) begin
                next_counter = counter + 1'b1;
            end else begin
                next_state = RECEIVE_X_DIR;
                next_counter = 7'b0;
            end
        end  
        default: next_state = IDLE;
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

