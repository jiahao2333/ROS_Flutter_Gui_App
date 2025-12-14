from gpiozero import LED
from time import sleep

# 初始化 LED 对象，参数为 BCM 编号
led = LED(4)

while True:
    led.on()   # 点亮
    print("LED On")
    sleep(1)   # 等待1秒
    led.off()  # 熄灭
    print("LED Off")
    sleep(1)