import serial
import time

ser = serial.Serial(
    port='COM10',  # 看裝置管理員      
    baudrate=230400,   
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
)

def read_motor_steps_from_file(filename):
    """從 motor_steps_output.txt 讀取步數資料並扁平化"""
    motor_steps = []
    with open(filename, "r", encoding="utf-8") as file:
        for line in file.readlines():
            # 解析每行中的步數資料 (X, C, Z)
            parts = line.strip().split()
            if len(parts) == 5:  # 確保每行有正確的 5 個資料
                if parts[0] == '-':
                    dirX = 1;
                else:
                    dirX = 0;
                stepsX = int(parts[1])
                if parts[2] == '+':
                    dirC = 1;
                else:
                    dirC = 0;
                stepsC = int(parts[3])
                z = int(parts[4])
                motor_steps.extend([dirX, stepsX, dirC, stepsC, z])  # 將資料展開到平鋪的列表中
    return motor_steps

def send_data_to_fpga(data):
    """發送資料到 FPGA"""
    if ser.is_open:
        ser.write(bytes([data]))  
        print(f"Sent: {data} (Binary: {bin(data)[2:].zfill(8)})")
    else:
        print("Serial port not open")

def receive_data_from_fpga():
    """從 FPGA 接收資料"""
    if ser.in_waiting > 0:
        data = ser.read(1)
        value = int.from_bytes(data, byteorder='big')
        print(f"Received from FPGA: {value} (Binary: {bin(value)[2:].zfill(8)})")
        ser.reset_input_buffer()  # 清空緩衝區
        return value
    return None  # 若無資料，回傳 None


motor_steps = read_motor_steps_from_file("motor_steps_output.txt")
step_iterator = iter(motor_steps)  # 將清單轉換為迭代器

try:
    print("開始與 FPGA 通信...")
    value_received = 1  # 初始化為 1，確保可以發送第一筆資料

    while True:
        # 接收 FPGA 的回傳資料
        if ser.in_waiting > 0:
            received_value = receive_data_from_fpga()
            if received_value == 1:  # 檢查是否接收到 1
                try:
                    # 發送下一筆資料
                    value_to_send = next(step_iterator)
                    send_data_to_fpga(value_to_send)
                except StopIteration:
                    # 重置資料迭代器，完成後重新開始
                    print("資料已全部傳送完畢")
                    break;
        time.sleep(0.1)  # 減少迴圈的執行頻率
except KeyboardInterrupt:
    print("使用者終止程序")
finally:
    ser.close()