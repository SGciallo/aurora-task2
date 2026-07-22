#!/usr/bin/env python3
"""小云台协议探测工具 — 测试不同参数"""
import serial, struct, time

PORT = '/dev/ttyUSB0'

# ── 测试1: 发送模式切换命令(连续20次) ──
print('=== 测试1: 连续发送 mode=1 命令 ===')
ser = serial.Serial(PORT, 115200, timeout=0.5)
ser.flushInput()
raw = ser.read(200)
if raw:
    for i in range(len(raw)-42):
        if raw[i]==0x5A and raw[i+1]==0xA5 and raw[i+41]==0x7F and raw[i+42]==0xFE:
            f=raw[i:i+43]
            yaw = struct.unpack('<f',f[19:23])[0]
            pit = struct.unpack('<f',f[27:31])[0]
            print(f'  [发送前] mode={f[2]} yaw={yaw*57.3:.1f}deg pit={pit*57.3:.1f}deg')
            break

# 发送20次 mode=1, yaw=pitch=0
pkt = struct.pack('<BB',0x5A,0xA5) + struct.pack('<B',1)
pkt += struct.pack('<f',0.0)*6  # 6个float都是0
pkt += struct.pack('<BB',0x7F,0xFE)
assert len(pkt) == 29, f"包大小不对: {len(pkt)}"
print(f'  TX({len(pkt)}B): {pkt.hex(" ")}')

for i in range(20):
    ser.write(pkt)
    ser.flush()
    time.sleep(0.02)

time.sleep(0.5)
raw2 = ser.read(300)
found = False
for i in range(len(raw2)-42):
    if raw2[i]==0x5A and raw2[i+1]==0xA5 and raw2[i+41]==0x7F and raw2[i+42]==0xFE:
        f=raw2[i:i+43]
        yaw = struct.unpack('<f',f[19:23])[0]
        pit = struct.unpack('<f',f[27:31])[0]
        print(f'  [发送后] mode={f[2]} yaw={yaw*57.3:.1f}deg pit={pit*57.3:.1f}deg')
        found = True
        break
if not found:
    print('  [发送后] 未找到有效帧')
ser.close()

# ── 测试2: 不同波特率 ──
print('\n=== 测试2: 波特率扫描 ===')
for baud in [921600, 460800, 230400, 57600, 38400, 19200, 9600]:
    try:
        ser = serial.Serial(PORT, baud, timeout=0.2)
        ser.flushInput()
        raw = ser.read(200)
        ser.close()
        status = f"收到{len(raw)}字节" if raw else "无数据"
        print(f'  {baud:>7}: {status}')
    except Exception as e:
        print(f'  {baud:>7}: 错误 {e}')

# ── 测试3: 尝试发送mode=2和mode=3 ──
print('\n=== 测试3: 尝试 mode=2 (小符) mode=3 (大符) ===')
for test_mode in [2, 3]:
    ser = serial.Serial(PORT, 115200, timeout=0.3)
    ser.flushInput()
    pkt = struct.pack('<BB',0x5A,0xA5) + struct.pack('<B',test_mode)
    pkt += struct.pack('<f',0.0)*6
    pkt += struct.pack('<BB',0x7F,0xFE)
    print(f'  发送 mode={test_mode} ...')
    for i in range(10):
        ser.write(pkt)
        ser.flush()
        time.sleep(0.03)
    time.sleep(0.3)
    raw = ser.read(200)
    for i in range(len(raw)-42):
        if raw[i]==0x5A and raw[i+1]==0xA5 and raw[i+41]==0x7F and raw[i+42]==0xFE:
            f=raw[i:i+43]
            print(f'    RX mode={f[2]} (期望变成{test_mode})')
            break
    ser.close()

print('\n=== 测试完成 ===')
