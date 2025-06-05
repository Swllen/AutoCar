import subprocess

# 默认摄像头设备路径
DEVICE = "/dev/video0"

# 可选参数及说明
PARAMETERS = {
    "1": ("exposure_auto", "曝光模式（1=手动, 3=自动）"),
    "2": ("exposure_absolute", "曝光值"),
    "3": ("brightness", "亮度"),
    "4": ("contrast", "对比度"),
    "5": ("saturation", "饱和度"),
    "6": ("gain", "增益"),
    "7": ("sharpness", "锐度"),
    "8": ("gamma", "伽马值"),
}


def show_device_capabilities(device):
    print(f"\n📋 设备 {device} 支持的参数（v4l2-ctl -l）：\n")
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device, "-l"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True  # ✅ 替代 text=True
        )

        print(result.stdout)
    except subprocess.CalledProcessError as e:
        print("❌ 无法获取设备参数信息:", e.stderr.strip())


def list_parameters():
    print("\n📌 可调参数列表（输入编号选择）：")
    for key, (param, desc) in PARAMETERS.items():
        print(f"  {key}. {param} - {desc}")
    print("  q. 退出程序")


def set_parameter(device, param, value):
    try:
        subprocess.run([
            "v4l2-ctl", "-d", device, "-c", f"{param}={value}"
        ], check=True)
        print(f"[✔] 成功设置 {param} = {value}")
    except subprocess.CalledProcessError as e:
        print(f"[✘] 设置失败: {e}")


def main():
    print("🎥 USB摄像头参数调节工具")
    print(f"🎯 当前设备：{DEVICE}")

    # 打印支持参数
    show_device_capabilities(DEVICE)

    while True:
        list_parameters()
        choice = input("\n请输入参数编号 (或 q 退出): ").strip()

        if choice.lower() == 'q':
            print("✅ 程序已退出。")
            break
        elif choice not in PARAMETERS:
            print("⚠ 无效选择，请重新输入。")
            continue

        param, desc = PARAMETERS[choice]
        value = input(f"请输入 {desc} 的数值：").strip()

        if not value.isdigit():
            print("⚠ 无效输入，请输入数字。")
            continue

        set_parameter(DEVICE, param, value)


if __name__ == "__main__":
    main()
