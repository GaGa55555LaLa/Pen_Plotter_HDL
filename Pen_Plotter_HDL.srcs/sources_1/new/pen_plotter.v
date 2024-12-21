
module pen_plotter(clk, reset, rx, start, collision_in, return_to_0, servo_pwm, tx, collision_out, btm_IN1, 
btm_IN2, btm_IN3, btm_IN4, btm_ENA, btm_ENB, top_IN1, top_IN2, top_IN3, top_IN4, top_ENA, top_ENB, AN, seg, start_signal);

// inputs
input             clk;
input             reset;
input             rx;
input             start;
input             collision_in;
input             return_to_0;

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
wire               start_pb;
wire               start_op;
wire               data_ready;
wire               tell_py_start_to_send_data;
wire [7:0]         rx_data;
wire               btm_done;
wire               top_done;
wire [7:0]         btm_step;
wire               btm_dir;
wire [7:0]         top_step;
wire               top_dir;
wire               return_to_0_db;
wire               return_to_0_op;

// reg

debounce d0(rst_pb, reset, clk);
onepulse d1(rst_pb, clk, rst_op);
debounce d2(start_pb, start, clk);
onepulse d3(start_pb, clk, start_op);
debounce d4(return_to_0_pb, return_to_0, clk);
onepulse d5(return_to_0_pb, clk, return_to_0_op);

servo servo_motor(.clk(clk), .reset(rst_op), .servo(servo_pwm), .done(/* ? */));
uart_top uart(.clk(clk), .rx(rx), .rst(rst_op), .tx(tx), .data_ready(data_ready), .tell_py_start_to_send_data(tell_py_start_to_send_data), .rx_data(rx_data));
limit_switch limit_sw(.clk(clk), .btn_in(collision_in), .btn_out(collision_out));
stepper_motor_bottom motor_btm(.clk(clk), .step(btm_step), .start(btm_start), .dir(btm_dir), .IN1(btm_IN1), .IN2(btm_IN2), .IN3(btm_IN3), .IN4(btm_IN4), .ENA(btm_ENA), .ENB(btm_ENB), .done(btm_done));
stepper_motor_top motor_top(.clk(clk), .step(top_step), .start(top_start), .dir(top_dir), .IN1(top_IN1), .IN2(top_IN2), .IN3(top_IN3), .IN4(top_IN4), .ENA(top_ENA), .ENB(top_ENB), .done(top_done));
fsm fsm(.clk(clk), .start_op(start_op), .rst_op(rst_op), .btm_step(btm_step), .btm_start(btm_start), .btm_dir(btm_dir), .btm_done(btm_done), .top_step(top_step), .top_start(top_start), .top_dir(top_dir), .top_done(top_done), .rx_data(rx_data), .tell_py_start_to_send_data(tell_py_start_to_send_data), .data_ready(data_ready), .state(state), .collision_out(collision_out), .return_to_0_op(return_to_0_op));


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
            4'b1000: seg = 7'b0000000;  // Display 8
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

module fsm (clk, start_op, rst_op, btm_step, btm_start, btm_dir, btm_done, top_step, top_start, top_dir, top_done, rx_data, tell_py_start_to_send_data, data_ready, state, collision_out, return_to_0_op);

input              clk;
input              start_op;
input              rst_op;
input [7:0]        rx_data;
input              data_ready;
input              btm_done, top_done;
input              collision_out;
input              return_to_0_op;

output reg [3:0]  state;
reg [3:0]         next_state;
reg [6:0]         stable_counter;
reg               Z_axis; // 1  上升 / 0 下降
reg [7:0]         next_top_step;
reg               next_top_dir;
reg [7:0]         next_btm_step;
reg               next_btm_dir; 
reg               next_Z_axis;
reg               next_top_start;
reg               next_btm_start;

output reg        tell_py_start_to_send_data;
output reg [7:0]  btm_step, top_step;
output reg        btm_start, top_start;
output reg        btm_dir, top_dir;
//for test

parameter IDLE                    = 4'd0;   // 閒置
parameter RESETTING_L             = 4'd1;  // 往回走 未接觸 limit switch
parameter RESETTING_R             = 4'd2;  // 往中心走 已接觸 limit switch
parameter RECEIVE_TOP_DIR         = 4'd3;   // 接收 X 軸方向
parameter SEND_TOP_DIR_COMPLETE   = 4'd4;   // 發送 X 軸方向接收完畢
parameter RECEIVE_TOP_STEP        = 4'd5;   // 接收 X 軸步數
parameter SEND_TOP_STEP_COMPLETE  = 4'd6;   // 發送 X 軸步數接收完畢
parameter RECEIVE_BTM_DIR         = 4'd7;   // 接收 Y 軸方向
parameter SEND_BTM_DIR_COMPLETE   = 4'd8;   // 發送 Y 軸方向接收完畢
parameter RECEIVE_BTM_STEP        = 4'd9;   // 接收 Y 軸步數
parameter SEND_BTM_STEP_COMPLETE  = 4'd10;   // 發送 Y 軸步數接收完畢
parameter RECEIVE_Z               = 4'd11;   // 接收 Z 軸狀態
parameter SEND_Z_COMPLETE         = 4'd12;  // 發送 Z 軸狀態接收完畢
parameter WAIT_TO_STABLE          = 4'd13;  // 等待訊號穩定
parameter EXECUTION               = 4'd14;  // 執行指令 
parameter EXECUTION_COMPLETE      = 4'd15;  // 完成指令並回傳

reg [6:0]         counter;
reg [6:0]         next_counter;

always@(posedge clk) begin
    if(rst_op) begin
        state <= IDLE;
        counter <= 7'b0;
        top_step <= 0;
        btm_step <= 0;
        Z_axis <= 1;
        top_dir <= 0;
        btm_dir <= 0;
        btm_start <= 0;
        top_start <= 0;
        tell_py_start_to_send_data <= 0;
    end else begin
        state <= next_state;
        counter <= next_counter;
        top_step <= next_top_step;
        btm_step <= next_btm_step;
        Z_axis <= next_Z_axis;
        top_dir <= next_top_dir;
        btm_dir <= next_btm_dir;
        btm_start <= next_btm_start;
        top_start <= next_top_start;
        tell_py_start_to_send_data <= (start_op || state == SEND_TOP_DIR_COMPLETE || state == SEND_TOP_STEP_COMPLETE || state == SEND_BTM_DIR_COMPLETE || state == SEND_BTM_STEP_COMPLETE);
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
            if (return_to_0_op) begin
                next_state = RESETTING_L;
            end
            if(start_op) next_state = RECEIVE_TOP_DIR;
            else next_state = IDLE;
        end    
        RESETTING_L: begin
            next_top_start = 1'b1;
            next_top_dir = 1'b1;  //spin left?
            next_top_step = 16'd438;   //大約7公分
            if (collision_out == 1'b0) begin
                next_top_start = 1'b0;
                next_state = RESETTING_R;
            end else next_state = RESETTING_L;
        end    
        RESETTING_R: begin
            next_top_start = 1'b1;
            next_top_dir = 1'b0;  //spin right?
            next_top_step = 16'd313;   //大約5公分
            if (top_done) begin
                next_top_start = 1'b0;
                next_state = IDLE;
            end else next_state = RESETTING_R;
        end        
        RECEIVE_TOP_DIR: begin
            if(data_ready) next_state = SEND_TOP_DIR_COMPLETE;
            else next_state = RECEIVE_TOP_DIR;
            next_top_step = 0;
            next_top_dir = rx_data;
            next_top_step = 0;
            next_btm_dir = 0; 
            next_Z_axis = 0;
        end       
        SEND_TOP_DIR_COMPLETE: begin
            if(tell_py_start_to_send_data) next_state = RECEIVE_TOP_STEP;
            else next_state = SEND_TOP_DIR_COMPLETE;
        end 
        RECEIVE_TOP_STEP: begin
            if(data_ready) next_state = SEND_TOP_STEP_COMPLETE;
            else next_state = RECEIVE_TOP_STEP;
            next_top_step = rx_data;
            next_btm_step = 0;
            next_btm_dir = 0; 
            next_Z_axis = 0;
        end      
        SEND_TOP_STEP_COMPLETE: begin
            if(tell_py_start_to_send_data) next_state = RECEIVE_BTM_DIR;
            else next_state = SEND_TOP_STEP_COMPLETE;
        end
        RECEIVE_BTM_DIR: begin
            if(data_ready) next_state = SEND_BTM_DIR_COMPLETE;
            else next_state = RECEIVE_BTM_DIR;
            next_btm_step = 0;
            next_btm_dir = rx_data; 
            next_Z_axis = 0;
        end       
        SEND_BTM_DIR_COMPLETE: begin
            if(tell_py_start_to_send_data) next_state = RECEIVE_BTM_STEP;
            else next_state = SEND_BTM_DIR_COMPLETE;
        end 
        RECEIVE_BTM_STEP: begin
            if(data_ready) next_state = SEND_BTM_STEP_COMPLETE;
            else next_state = RECEIVE_BTM_STEP;
            next_btm_step = rx_data;
            next_Z_axis = 0;
        end      
        SEND_BTM_STEP_COMPLETE: begin
            if(tell_py_start_to_send_data) next_state = RECEIVE_Z;
            else next_state = SEND_BTM_STEP_COMPLETE;
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
                next_top_start = 1'b0;
                next_btm_start = 1'b0;
                next_state = EXECUTION_COMPLETE;
            end
            else begin
                next_top_start = 1'b1;
                next_btm_start = 1'b1;
                if (top_done && btm_done) begin
                    next_top_start = 1'b0;
                    next_btm_start = 1'b0;
                    next_state = EXECUTION_COMPLETE;
                end
            end
        end           
        EXECUTION_COMPLETE: begin
            // TODO: 等待幾個 cycle 回到 RECEIVE_X_DIR
            if (counter == ~7'b0) begin
                next_state = RECEIVE_TOP_DIR;
                next_counter = 7'b0;
            end else begin
                next_state = EXECUTION_COMPLETE;
                next_counter = counter + 1'b1;
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

