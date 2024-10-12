#!/usr/bin/env python3
import serial
import time
from datetime import datetime

class SerialMonitor:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1,
            write_timeout=1
        )
        print(f"Đang monitor Serial port {port} với baudrate {baudrate}")
        print("--------------------------------------------")
        print("Chú thích:")
        print("→ : Dữ liệu từ ROS xuống STM32")
        print("← : Dữ liệu từ STM32 lên ROS")
        print("--------------------------------------------")

    def monitor(self):
        try:
            while True:
                # Đọc dữ liệu từ Serial
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    if data:
                        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        print(f"\n[{timestamp}] ← Nhận từ STM32:")
                        # In dữ liệu dạng hex và decimal
                        print("HEX:", ' '.join([f'{b:02X}' for b in data]))
                        print("DEC:", list(data))
                        
                        # Nếu định dạng giống protocol từ ROS (7 bytes)
                        if len(data) >= 7 and data[0] == 0xFF and data[1] == 0xF5:
                            print("Phân tích gói tin:")
                            print(f"Start byte: {data[0]} (0x{data[0]:02X})")
                            print(f"Command byte: {data[1]} (0x{data[1]:02X})")
                            print(f"Góc B: {data[2]} độ")
                            print(f"Góc C: {data[3]} độ")
                            print(f"Góc B': {data[4]} độ")
                            print(f"Góc C': {data[5]} độ")
                
                time.sleep(0.01)  # Giảm tải CPU

        except KeyboardInterrupt:
            print("\nDừng monitor")
        except serial.SerialException as e:
            print(f"Lỗi Serial: {e}")
        finally:
            if self.ser.is_open:
                self.ser.close()

    def send_test_data(self):
        """Gửi dữ liệu test để giả lập ROS"""
        test_data = bytearray([0xFF, 0xF5, 45, 90, 120, 180])
        try:
            self.ser.write(test_data)
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"\n[{timestamp}] → Gửi từ ROS:")
            print("HEX:", ' '.join([f'{b:02X}' for b in test_data]))
            print("DEC:", list(test_data))
        except serial.SerialException as e:
            print(f"Lỗi khi gửi dữ liệu: {e}")

def main():
    # Thay đổi port phù hợp với hệ thống của bạn
    port = input("Nhập port Serial (mặc định /dev/ttyUSB0): ").strip() or '/dev/ttyUSB0'
    
    monitor = SerialMonitor(port=port)
    print("\n1. Chỉ monitor dữ liệu")
    print("2. Monitor và gửi dữ liệu test")
    choice = input("Chọn chế độ (1/2): ").strip()
    
    if choice == "2":
        # Tạo thread riêng để gửi dữ liệu test mỗi 2 giây
        import threading
        def send_periodic():
            while True:
                monitor.send_test_data()
                time.sleep(2)
        
        sender = threading.Thread(target=send_periodic, daemon=True)
        sender.start()
    
    monitor.monitor()

if __name__ == "__main__":
    main()