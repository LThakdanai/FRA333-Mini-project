import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ฟังก์ชันสำหรับการอัพเดทข้อมูลในกราฟ
def update(frame):
    # สร้างข้อมูลสุ่มขนาด 3x1
    data = np.random.rand(3, 1)

    # แยกข้อมูลแต่ละคอลัมน์
    data_0 = data[0]
    data_1 = data[1]
    data_2 = data[2]

    # ลบข้อมูลที่มีอยู่ในกราฟออก
    plt.clf()

    # สร้างกราฟสำหรับแต่ละคอลัมน์ใหม่
    plt.plot(data_0, label='Column 0')
    plt.plot(data_1, label='Column 1')
    plt.plot(data_2, label='Column 2')

    plt.xlabel('Index')
    plt.ylabel('Values')
    plt.title('Real-time Plot of Each Column in 3x1 Numpy Array')
    plt.legend()

# สร้าง figure
fig = plt.figure(figsize=(8, 6))

# เรียกใช้ FuncAnimation เพื่ออัพเดทกราฟทุกๆ 1000 milliseconds (1 วินาที)
ani = FuncAnimation(fig, update, interval=1000)

plt.show()
