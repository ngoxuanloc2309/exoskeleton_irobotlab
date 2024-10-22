#!usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np


# Tạo dữ liệu
def main() :
    x = np.linspace(0, 10, 100)
    y = np.sin(x)

    # Tạo đồ thị
    plt.figure(figsize=(10, 6))
    plt.plot(x, y, label='sin(x)')

    # Thêm tiêu đề và nhãn
    plt.title('Đồ thị hàm sin')
    plt.xlabel('x')
    plt.ylabel('y')

    # Thêm lưới
    plt.grid(True)

    # Hiển thị chú thích
    plt.legend()

    # Hiển thị đồ thị
    plt.show()

if __name__ == '__main__':
    (main)