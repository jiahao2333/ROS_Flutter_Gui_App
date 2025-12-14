# test_gpio.py
import RPi.GPIO as GPIO
import time

# 强制指定 GPIO 18 (物理引脚12)
LED_PIN = 17

try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_PIN, GPIO.OUT)
    print(f"成功打开 GPIO {LED_PIN}！(对应树莓派5的 Chip 4)")
    
    # 尝试操作一下
    GPIO.output(LED_PIN, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(LED_PIN, GPIO.LOW)
    print("操作成功，GPIO 库工作正常。")
    
    GPIO.cleanup()
except Exception as e:
    print(f"依然报错: {e}")