import RPi.GPIO as GPIO
import time

# 设置 GPIO 模式为 BCM 编码
GPIO.setmode(GPIO.BCM)

# 定义引脚
LED_PIN = 18

# 设置引脚为输出模式
GPIO.setup(LED_PIN, GPIO.OUT)

print(f"开始控制 GPIO {LED_PIN}，按 Ctrl+C 退出")

try:
    while True:
        GPIO.output(LED_PIN, GPIO.HIGH) # 开灯
        print("LED ON")
        time.sleep(1)
        
        GPIO.output(LED_PIN, GPIO.LOW)  # 关灯
        print("LED OFF")
        time.sleep(1)

except KeyboardInterrupt:
    print("程序停止")

finally:
    GPIO.cleanup() # 清理 GPIO 状态，这对下次运行很重要