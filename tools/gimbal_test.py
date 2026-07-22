#!/usr/bin/env python3
"""
小云台串口通信测试工具
用法: python3 tools/gimbal_test.py [--send] [--raw]
"""

import serial
import struct
import time
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--send', action='store_true', help='发送测试角度命令')
    parser.add_argument('--raw', type=str, help='发送原始hex (如 --raw "5AA501...")')
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    print(f"[+] 串口 {args.port} 已打开, 波特率 {args.baud}")

    # ── 先清空接收缓冲区 ──
    ser.flushInput()
    ser.flushOutput()

    # ── 模式1: 发送测试命令 ──
    if args.send:
        # 读取当前云台状态
        print("[*] 等待接收云台数据...")
        raw = ser.read(200)
        print(f"[RX] 收到 {len(raw)} 字节: {raw.hex(' ')}")
        if len(raw) >= 43:
            # 解析第一个有效帧
            for i in range(len(raw) - 42):
                if raw[i] == 0x5A and raw[i+1] == 0xA5 and raw[i+41] == 0x7F and raw[i+42] == 0xFE:
                    frame = raw[i:i+43]
                    mode = frame[2]
                    yaw = struct.unpack('<f', frame[18:22])[0]
                    pitch = struct.unpack('<f', frame[26:30])[0]
                    print(f"[解析] mode={mode}, yaw={yaw:.3f}rad({yaw*57.3:.1f}deg), pitch={pitch:.3f}rad({pitch*57.3:.1f}deg)")

                    # 构造发送包: 在当前位置基础上加10度
                    target_yaw = yaw + 10.0 * 3.14159 / 180.0  # 10度
                    target_pitch = pitch + 5.0 * 3.14159 / 180.0

                    # 发送相对偏移: yaw_rel = target_yaw - current_yaw
                    yaw_rel = 10.0 * 3.14159 / 180.0
                    pitch_rel = 5.0 * 3.14159 / 180.0

                    tx_pkt = struct.pack('<BB', 0x5A, 0xA5)  # 帧头
                    tx_pkt += struct.pack('<B', 1)            # mode=1 自瞄
                    tx_pkt += struct.pack('<f', yaw_rel)      # yaw 相对角 (弧度)
                    tx_pkt += struct.pack('<f', 5.0)          # yaw_vel
                    tx_pkt += struct.pack('<f', 0.0)          # yaw_acc
                    tx_pkt += struct.pack('<f', pitch_rel)    # pitch 相对角 (弧度)
                    tx_pkt += struct.pack('<f', 5.0)          # pitch_vel
                    tx_pkt += struct.pack('<f', 0.0)          # pitch_acc
                    tx_pkt += struct.pack('<BB', 0x7F, 0xFE)  # 帧尾

                    print(f"[TX] 发送 {len(tx_pkt)} 字节: {tx_pkt.hex(' ')}")
                    print(f"[TX] yaw_rel={yaw_rel:.3f}rad({yaw_rel*57.3:.1f}deg), pitch_rel={pitch_rel:.3f}rad({pitch_rel*57.3:.1f}deg)")
                    ser.write(tx_pkt)
                    ser.flush()
                    print("[+] 已发送，观察云台是否移动...")

                    # 等待看云台回发数据
                    time.sleep(0.5)
                    raw2 = ser.read(200)
                    print(f"[RX after TX] 收到 {len(raw2)} 字节: {raw2.hex(' ')}")
                    break
            else:
                print("[-] 未找到有效帧")
        else:
            print(f"[-] 收到数据不足43字节")

    # ── 模式2: 发送原始hex ──
    if args.raw:
        data = bytes.fromhex(args.raw.replace(' ', ''))
        print(f"[TX raw] 发送 {len(data)} 字节: {data.hex(' ')}")
        ser.write(data)
        ser.flush()
        print("[+] 已发送")

    # ── 默认模式: 持续监听 ──
    if not args.send and not args.raw:
        print("[*] 监听模式 (Ctrl+C 退出)...")
        count = 0
        try:
            while count < 20:
                raw = ser.read(100)
                if raw:
                    print(f"[RX #{count}] {len(raw)} 字节: {raw.hex(' ')}")
                    # 找所有有效帧
                    for i in range(len(raw) - 42):
                        if raw[i] == 0x5A and raw[i+1] == 0xA5 and raw[i+41] == 0x7F and raw[i+42] == 0xFE:
                            frame = raw[i:i+43]
                            mode = frame[2]
                            yaw = struct.unpack('<f', frame[18:22])[0]
                            pitch = struct.unpack('<f', frame[26:30])[0]
                            print(f"  -> mode={mode}, yaw={yaw*57.3:.1f}deg, pitch={pitch*57.3:.1f}deg")
                else:
                    print(f"[RX #{count}] 超时无数据")
                count += 1
        except KeyboardInterrupt:
            pass

    ser.close()
    print("[+] 串口已关闭")

if __name__ == '__main__':
    main()
