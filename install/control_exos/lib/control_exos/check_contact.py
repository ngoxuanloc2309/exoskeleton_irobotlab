#!/usr/bin/env python3
import serial
import time

def verify_serial(port='/dev/ttyUSB0', baudrate=115200):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Đang kiểm tra dữ liệu trên {port}...")
        
        count = 0
        start_time = time.time()
        
        while True:
            if ser.in_waiting:
                byte = ser.read()
                if byte[0] == 0xFF:  # Tìm start byte
                    count += 1
                    print(f"Đã nhận gói tin thứ {count}")
                    remaining = ser.read(6)  # Đọc 6 byte còn lại
                    print(f"Dữ liệu đầy đủ: 0xFF {' '.join([f'0x{b:02X}' for b in remaining])}")
            
            # Hiển thị tốc độ nhận dữ liệu sau mỗi 5 giây
            if time.time() - start_time >= 5:
                print(f"\nTốc độ nhận: {count/5:.2f} gói/giây")
                count = 0
                start_time = time.time()
                
    except serial.SerialException as e:
        print(f"Lỗi: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    verify_serial()