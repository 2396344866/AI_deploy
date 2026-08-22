#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
静态核对 Components/Debug 调试遥测的帧定义与启动顺序（不依赖硬件）：
  1. dbg_config.h 的 DBG_FRAME_N 必须为 44（37 原通道 + 37-43 磁力计）；
  2. dbg_telemetry.c 中所有 s_frame[N] 赋值索引 N 必须满足 0 <= N < DBG_FRAME_N（无越界）；
  3. for (i=0; i<DBG_FRAME_N; i++) 循环存在；
  4. main.c 中 Dbg_Telemetry_Init() 调用必须在 BSP_UART1_RxStart() 之前（波特率重设不破坏命令 RX）。
"""
import re, sys, os

ROOT = os.path.dirname(os.path.abspath(__file__))   # 本脚本位于 Components/Debug/
CFG  = os.path.join(ROOT, "Inc", "dbg_config.h")
SRC  = os.path.join(ROOT, "Src", "dbg_telemetry.c")
MAIN = os.path.join(ROOT, "..", "..", "Core", "Src", "main.c")

ok = True

# ---- 1. DBG_FRAME_N ----
cfg_txt = open(CFG, encoding="utf-8", errors="ignore").read()
m = re.search(r"#define\s+DBG_FRAME_N\s+(\d+)", cfg_txt)
frame_n = int(m.group(1)) if m else None
print(f"[1] DBG_FRAME_N = {frame_n} (期望 44)")
if frame_n != 44:
    ok = False

# ---- 2/3. s_frame[N] 索引 + 循环 ----
src_txt = open(SRC, encoding="utf-8", errors="ignore").read()
idxs = [int(x) for x in re.findall(r"s_frame\[(\d+)\]\s*=", src_txt)]
loop_ok = bool(re.search(r"for\s*\([^)]*i\s*<\s*DBG_FRAME_N", src_txt))
print(f"[2] s_frame 赋值索引集合 = {sorted(set(idxs))}")
print(f"    最大索引 = {max(idxs) if idxs else '无'}，最小索引 = {min(idxs) if idxs else '无'}")
if idxs and (min(idxs) < 0 or max(idxs) >= (frame_n or 999)):
    ok = False
    print("    !! 越界：存在索引 <0 或 >= DBG_FRAME_N")
if not loop_ok:
    ok = False
    print("    !! 未找到 for(i<DBG_FRAME_N) 发送循环")
print(f"[3] 发送循环 for(i<DBG_FRAME_N) 存在 = {loop_ok}")

# ---- 4. 启动顺序：Dbg_Telemetry_Init 在 BSP_UART1_RxStart 之前 ----
main_txt = open(MAIN, encoding="utf-8", errors="ignore").read()
p_init = main_txt.find("Dbg_Telemetry_Init(")
p_rx   = main_txt.find("BSP_UART1_RxStart(")
print(f"[4] main.c: Dbg_Telemetry_Init 位置 = {p_init}, BSP_UART1_RxStart 位置 = {p_rx}")
if p_init == -1 or p_rx == -1:
    ok = False
    print("    !! 两个调用有一个未找到")
elif p_init > p_rx:
    ok = False
    print("    !! 顺序错误：波特率重设(Dbg_Telemetry_Init) 在 UART1 Rx 启动之后，会破坏命令接收")

print("\n结论:", "✅ 全部通过" if ok else "❌ 存在问题，见上")
sys.exit(0 if ok else 1)
