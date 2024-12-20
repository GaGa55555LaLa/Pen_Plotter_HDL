import serial
import time

ser = serial.Serial(
    port='COM6',  # 看裝置管理員      
    baudrate=230400,   
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
)

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

data_list = [1, 200, 1, 300, 1, 0, 100, 0, 200, 1, 0, 150, 1, 100, 0];
data_iterator = iter(data_list)  # 將清單轉換為迭代器

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
                    value_to_send = next(data_iterator)
                    send_data_to_fpga(value_to_send)
                except StopIteration:
                    # 重置資料迭代器，完成後重新開始
                    print("Data list is complete. Restarting...")
                    data_iterator = iter(data_list)
        time.sleep(0.1)  # 減少迴圈的執行頻率
except KeyboardInterrupt:
    print("使用者終止程序")
finally:
    ser.close()