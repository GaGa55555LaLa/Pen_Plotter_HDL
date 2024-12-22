//---------------------------------------------------
//  PWM 週期：                                    20ms
//  伺服馬達最小脈衝寬度(0)：                     0.54ms
//  伺服馬達最大脈衝寬度(180)：                   2.54ms
//  這裡取 0.54 ms - 1.54 ms (0-90)
//  通常 1ms -> 0 度, 2ms -> 180度
//  但 Tower Pro SG90 這台他媽的是 0.54ms -> 2.54ms TMD
//---------------------------------------------------

module servo(clk, reset, dir, servo, done);

input clk;
input reset;
input dir;
output wire servo;
output reg done;

localparam PWM_PERIOD = 2000000;  // 20ms (週期對應的計數)
localparam MIN_WIDTH  = 54000;   // 1ms (伺服最小脈衝寬度)
localparam MAX_WIDTH  = 154000;   // 2ms (伺服最大脈衝寬度)
localparam STEP       = 5000;     // 每次調整的步長(使轉動平滑)
//  從 0 度到 180 度的時間為 20 * (154000-54000) / 5000 = 400 ms = 0.4s

reg [20:0] counter;
reg        servo_reg;
reg [18:0] control;
// reg        toggle;
// reg        up;


always @(posedge clk) begin

    if(reset) begin
        counter <= 0;
        servo_reg <= 0;
        control <= MIN_WIDTH;
        done <= 0;
        // toggle <= 1;
    end
    else begin
        
        if (counter == PWM_PERIOD - 1)
            counter <= 0;
        else
            counter <= counter + 1;

        if (counter < control)
            servo_reg <= 1;
        else
            servo_reg <= 0;

        if(counter == 0) begin
            if (dir)
                control <= (control + STEP >= MAX_WIDTH)? MAX_WIDTH : control + STEP;
            else
                control <= (control - STEP <= MIN_WIDTH)? MIN_WIDTH : control - STEP;

            if (control >= MAX_WIDTH) begin
                // toggle <= 0;
                done <= 1'b1; // Signal completion of forward rotation
            end else if (control <= MIN_WIDTH) begin
                // toggle <= 1;
                done <= 1'b1; // Signal completion of backward rotation
            end else begin
                done <= 1'b0;
            end
        end
    end
end

assign servo = servo_reg;

endmodule
