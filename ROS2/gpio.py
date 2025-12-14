import lgpio

print("尝试直接打开 gpiochip4 (树莓派5专用)...")

try:
    # 树莓派5的关键接口是 chip 4
    h = lgpio.gpiochip_open(4)
    print("✅ 成功！Chip 4 可以打开！")
    
    # 获取芯片信息
    info = lgpio.gpiochip_get_info(h)
    print(f"   芯片名称: {info.name}")
    print(f"   引脚数量: {info.lines}")
    
    lgpio.gpiochip_close(h)
    
except Exception as e:
    print(f"❌ 失败: {e}")
    print("尝试打开 chip 0 (旧版树莓派默认)...")
    try:
        h = lgpio.gpiochip_open(0)
        print("⚠️ Chip 0 打开了 (但这在树莓派5上通常是系统控制芯片，不是GPIO)")
        lgpio.gpiochip_close(h)
    except:
        print("❌ Chip 0 也打不开")