
// `timescale 1ns / 1ps

// module stepper_motor_top(clk, stop_motor, step, start, dir, IN1, IN2, IN3, IN4, ENA, ENB, done);
// input wire clk;
// input wire stop_motor;
// // input wire [15:0] angle_15; // Desired angle (0-360 degrees)
// input wire [15:0] step; // Desired step
// input wire start;        // Start rotation signal
// input wire dir;          // Spin direction if 1 left, 0 right
// output reg IN1;
// output reg IN2;
// output reg IN3;
// output reg IN4;
// output reg ENA;
// output reg ENB;
// output reg done;          // Signals when rotation is complete


// // GT2 齒距 2mm，18齒，其齒根直徑為 2 * 18 / pi = 11.459mm
// // 馬達轉一圈(200步)會帶動 belt 走同步帶輪的周長(齒數*齒根直徑)
// // 0.18mm/step
// // 校正後 0.16mm/step

// localparam [1:0] STEP1 = 2'b00,
//                  STEP2 = 2'b01,
//                  STEP3 = 2'b10,
//                  STEP4 = 2'b11;

// // Calculate the divider value for desired RPM
// // For 100MHz clock: CLK_DIV_MAX = (100MHz) / (Steps_per_rev * Desired_RPM / 60)
// // Example: For 200 steps/rev at 60 RPM: 100MHz / (200 * 60/60) = 500,000
// parameter STEPS_PER_REV = 200;    // Typical for 1.8?X per step motor
// parameter CLK_DIV_MAX = 500000;   // Speed control
// reg [1:0] step_state;
// reg [31:0] clk_div;
// reg [15:0] step_count;
// reg [15:0] target_steps;
// reg running;

// // Formula: steps = (angle * STEPS_PER_REV) / 360
// always @(posedge clk) begin
//     if (stop_motor) begin
//         target_steps <= 16'd0;
//     end else if (start && !running) begin
//         // target_steps <= ((angle_15 * 45)* STEPS_PER_REV) / 360;
//         // target_steps <= 100;
//         target_steps <= step;
//     end
// end

// always @(posedge clk) begin
//     if (stop_motor) begin
//         clk_div <= 32'b0;
//         step_state <= STEP1;
//         step_count <= 16'd0;
//         running <= 1'b0;
//         done <= 1'b1;
//         IN1 <= 1'b0;
//         IN2 <= 1'b0;
//         IN3 <= 1'b0;
//         IN4 <= 1'b0;
//         ENA <= 1'b1;
//         ENB <= 1'b1;
//     end else begin
//         if (start && !running) begin
//             running <= 1'b1;
//             done <= 1'b0;
//             step_count <= 16'd0;
//         end
//         if (running) begin
//             if (step_count >= target_steps) begin
//                 running <= 1'b0;
//                 done <= 1'b1;
//                 IN1 <= 1'b0;
//                 IN2 <= 1'b0;
//                 IN3 <= 1'b0;
//                 IN4 <= 1'b0;
//             end else if (clk_div >= CLK_DIV_MAX - 1) begin
//                 clk_div <= 32'b0;
//                 step_count <= step_count + 1'b1;
                
//                 case (step_state)
//                     STEP1: begin
//                         IN1 <= 1'b1;
//                         IN2 <= 1'b0;
//                         IN3 <= 1'b1;
//                         IN4 <= 1'b0;
//                         step_state <= STEP2;
//                     end
//                     STEP2: begin
//                         if (dir==0) begin
//                             IN1 <= 1'b1;
//                             IN2 <= 1'b0;
//                             IN3 <= 1'b0;
//                             IN4 <= 1'b1;
//                         end else begin
//                             IN1 <= 1'b0;
//                             IN2 <= 1'b1;
//                             IN3 <= 1'b1;
//                             IN4 <= 1'b0;
//                         end
//                         step_state <= STEP3;
//                     end
//                     STEP3: begin
//                         IN1 <= 1'b0;
//                         IN2 <= 1'b1;
//                         IN3 <= 1'b0;
//                         IN4 <= 1'b1;
//                         step_state <= STEP4;
//                     end
//                     STEP4: begin
//                         if (dir==0) begin
//                             IN1 <= 1'b0;
//                             IN2 <= 1'b1;
//                             IN3 <= 1'b1;
//                             IN4 <= 1'b0;
//                         end else begin
//                             IN1 <= 1'b1;
//                             IN2 <= 1'b0;
//                             IN3 <= 1'b0;
//                             IN4 <= 1'b1;
//                         end
//                         step_state <= STEP1;
//                     end
//                 endcase
//             end else begin
//                 clk_div <= clk_div + 1'b1;
//             end
//         end
//     end
// end
// endmodule

`timescale 1ns / 1ps

module stepper_motor_top(clk, step, start, dir, IN1, IN2, IN3, IN4, ENA, ENB, done);
input wire clk;
// input wire stop_motor;
// input wire [15:0] angle_15; // Desired angle (0-360 degrees)
input wire [15:0] step; // Desired step
input wire start;        // Start rotation signal
input wire dir;          // Spin direction if 1 left, 0 right
output reg IN1;
output reg IN2;
output reg IN3;
output reg IN4;
output reg ENA;
output reg ENB;
output reg done;          // Signals when rotation is complete


// GT2 齒距 2mm，18齒，其齒根直徑為 2 * 18 / pi = 11.459mm
// 馬達轉一圈(200步)會帶動 belt 走同步帶輪的周長(齒數*齒根直徑)
// 0.18mm/step
// 校正後 0.16mm/step

localparam [1:0] STEP1 = 2'b00,
                 STEP2 = 2'b01,
                 STEP3 = 2'b10,
                 STEP4 = 2'b11;

// Calculate the divider value for desired RPM
// For 100MHz clock: CLK_DIV_MAX = (100MHz) / (Steps_per_rev * Desired_RPM / 60)
// Example: For 200 steps/rev at 60 RPM: 100MHz / (200 * 60/60) = 500,000
parameter STEPS_PER_REV = 200;    // Typical for 1.8?X per step motor
// parameter CLK_DIV_MAX = 500000;   // Speed control
reg [31:0] CLK_DIV_MAX;
reg [1:0] step_state;
reg [31:0] clk_div;
reg [15:0] step_count;
reg [15:0] target_steps;
reg running;

// Formula: steps = (angle * STEPS_PER_REV) / 360
always @(posedge clk) begin
    if (!start) begin
        target_steps <= 16'd0;
        CLK_DIV_MAX <= 32'b0;
    end else begin
        // target_steps <= ((angle_15 * 45)* STEPS_PER_REV) / 360;
        // target_steps <= 100;
        target_steps <= step;
        CLK_DIV_MAX <= 32'd100000000 / step;
    end
end

always @(posedge clk) begin
    if (!start) begin
        clk_div <= 32'b0;
        step_state <= STEP1;
        step_count <= 16'd0;
        // running <= 1'b0;
        done <= 1'b1;
        IN1 <= 1'b0;
        IN2 <= 1'b0;
        IN3 <= 1'b0;
        IN4 <= 1'b0;
        ENA <= 1'b1;
        ENB <= 1'b1;
    end else begin
        if (step_count >= target_steps) begin
            // running <= 1'b0;
            done <= 1'b1;
            IN1 <= 1'b0;
            IN2 <= 1'b0;
            IN3 <= 1'b0;
            IN4 <= 1'b0;
        end else if (clk_div >= CLK_DIV_MAX - 1) begin
            clk_div <= 32'b0;
            step_count <= step_count + 1'b1;
            
            case (step_state)
                STEP1: begin
                    IN1 <= 1'b1;
                    IN2 <= 1'b0;
                    IN3 <= 1'b1;
                    IN4 <= 1'b0;
                    step_state <= STEP2;
                end
                STEP2: begin
                    if (dir==0) begin
                        IN1 <= 1'b1;
                        IN2 <= 1'b0;
                        IN3 <= 1'b0;
                        IN4 <= 1'b1;
                    end else begin
                        IN1 <= 1'b0;
                        IN2 <= 1'b1;
                        IN3 <= 1'b1;
                        IN4 <= 1'b0;
                    end
                    step_state <= STEP3;
                end
                STEP3: begin
                    IN1 <= 1'b0;
                    IN2 <= 1'b1;
                    IN3 <= 1'b0;
                    IN4 <= 1'b1;
                    step_state <= STEP4;
                end
                STEP4: begin
                    if (dir==0) begin
                        IN1 <= 1'b0;
                        IN2 <= 1'b1;
                        IN3 <= 1'b1;
                        IN4 <= 1'b0;
                    end else begin
                        IN1 <= 1'b1;
                        IN2 <= 1'b0;
                        IN3 <= 1'b0;
                        IN4 <= 1'b1;
                    end
                    step_state <= STEP1;
                end
            endcase
        end else begin
            clk_div <= clk_div + 1'b1;
        end
    end
end
endmodule