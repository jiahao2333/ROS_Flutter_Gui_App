import lgpio
import time

# ⚠️ 关键点：这里必须填 4，对应你的 gpiochip4 [pinctrl-rp1]
CHIP = 4
LED_PIN = 17 # BCM 17

print(f"正在打开 gpiochip{CHIP} (RP1)...")

try:
    # 打开第 4 号芯片
    h = lgpio.gpiochip_open(CHIP)
    print("✅ 成功连接到 RP1 芯片！")

    # 申请引脚
    lgpio.gpio_claim_output(h, LED_PIN)
    
    while True:
        lgpio.gpio_write(h, LED_PIN, 1)
        print("LED ON")
        time.sleep(1)
        lgpio.gpio_write(h, LED_PIN, 0)
        print("LED OFF")
        time.sleep(1)

except Exception as e:
    print(f"❌ 错误: {e}")
finally:
    try:
        lgpio.gpiochip_close(h)
    except:
        pass