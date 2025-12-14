import gpiod
import time
import sys

# 树莓派 5 的核心配置
CHIP_PATH = "/dev/gpiochip4"  # 直接指定设备路径
LINE_OFFSET = 17              # BCM 17

print(f"正在尝试打开 {CHIP_PATH} ...")

try:
    # 1. 获取芯片对象
    chip = gpiod.Chip(CHIP_PATH)
    print(f"✅ 成功连接芯片: {chip.label}")
    
    # 2. 获取引脚 (Line)
    line = chip.get_line(LINE_OFFSET)
    
    # 3. 申请控制权 (设置为输出模式)
    config = gpiod.LineRequestConfig(consumer="DockerTest")
    line.request(config, type=gpiod.LINE_REQ_DIR_OUT)
    
    print("开始闪烁...")
    while True:
        line.set_value(1)
        print("LED ON")
        time.sleep(1)
        line.set_value(0)
        print("LED OFF")
        time.sleep(1)

except OSError as e:
    print(f"❌ 系统级错误: {e}")
    print("如果是 'No such file' -> 确认 /dev/gpiochip4 是否存在")
    print("如果是 'Permission denied' -> 确认 docker run 加了 --privileged")
    print("如果是 'Device or resource busy' -> 说明有别的程序(如系统服务)正在占用这个引脚")
except Exception as e:
    print(f"❌ 未知错误: {e}")