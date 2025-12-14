from gpiozero import LED
from time import sleep
import signal

# 使用 GPIO 17 (BCM 编码)
led = LED(17)

print("Container is running. Blinking LED on GPIO 17...")

try:
    while True:
        led.on()
        print("LED ON")
        sleep(1)
        led.off()
        print("LED OFF")
        sleep(1)
except KeyboardInterrupt:
    print("Exiting...")