# 启动故障与死机取证（通用方法论）

> 本文件由 `Components/Debug/Ref/启动故障与死机取证手册.md` 整合而来（源文件已删除，内容"放到一起"）。
> **层级**：运行期 / 调试取证，与 `Doc/` 下编译期排错记录分工，不混编。
> **风格**：关键词加粗 + 短句 + 逻辑优先。
> **具体实例仍记各任务 `Error/<TASK>_error.md`（事件 E N）**；本文件只讲通用取证方法论，不重复记实例。

---

## 一、HSE 配成 BYPASS 导致启动卡死（典型实例）

### 现象
- 上电运行，串口无任何输出，程序"死了"。
- Pause 后 PC 停在 `Error_Handler` 的 `while(1)`（main.c:254）。
- 卡点在 `SystemClock_Config()` 内部超时，非业务循环。

### 根因
- HSE 被 CubeMX 配成 `BYPASS Clock Source`（旁路/有源晶振模式）。
- 开发板焊的是无源晶体，须靠 STM32 内部驱动电路激励起振。
- BYPASS 模式内部反相放大电路关闭，只监听 `OSC_IN` 外部方波、`OSC_OUT` 悬空。
- 无源晶振无激励 → `HSERDY` 永不成立 → `HAL_RCC_OscConfig` 超时 → `Error_Handler()` 死循环。

### 修复（CubeMX 零代码改动）
1. HSE 下拉：`BYPASS Clock Source` → `Crystal/Ceramic Resonator`。
2. GENERATE CODE 重新生成。
3. Keil Rebuild → Download → Debug，HSE 起振。

> 防复发：每次 CubeMX 改时钟树后核对 HSE 模式与板载晶振类型匹配（无源→Ceramic，有源→BYPASS）。

---

## 二、启动卡死通用全链路（"串口无数据 + 停在死循环"适用）

1. 运行固件，发现串口无数据（卡在初始化早期，未到 printf）。
2. 关闭全速（Pause / Stop），观察 PC。
3. 若停在 `Error_Handler` `while(1)` 或任意 `for(;;)` → 冻死。
4. 观察 **LR 寄存器**（R14），记下值。
5. Debug → Disassembly → Show Disassembly at Address 输入该 LR/PC → 反汇编定位具体指令/函数段。

### 关键认知
- 任何 `while(1)`/`for(;;)` 死循环 = 致命冻死；调试器暂停即落在循环体内。
- 两类死循环来源：
  - **主动 `Error_Handler`**：HAL/业务显式调，定位看调用栈。
  - **`configASSERT` 内联死循环**：FreeRTOS 内部断言（FreeRTOSConfig.h），定位看断言所在函数（多为 `xQueue*`/`xTask*`）。

---

## 三、HardFault vs 普通死循环（LR 含义差异）

| 场景 | 触发机制 | LR 含义 | 处置 |
|---|---|---|---|
| 普通死循环 | `Error_Handler`/`configASSERT` 的 `while(1)` | 普通函数返回地址（`0x0800xxxx`） | 看调用栈找调用者；LR/PC 反汇编定位函数 |
| HardFault（异常） | 总线/内存/Usage 违规 | **EXC_RETURN**（`0xFFFFFFF1`） | 读 MSP/CFSR/BFAR 取证 |

- 通用入口 0 步"读 LR"两者共用；但：
  - 普通死循环：LR 给代码地址，直接反汇编看函数。
  - HardFault：LR 是 EXC_RETURN 魔法值，不能直接反汇编；须从**栈帧偏移 +24（MSP+24）**取原始返回地址再反汇编。
  - HardFault 还需 **CFSR/BFAR** 解码故障类型（普通死循环不需要）。

---

## 四、HardFault 取证最小步骤（工程实战，去演示冗余）

1. **Pause** 停在 `HardFault_Handler` 死循环。
2. 读 **xPSR** bit[7:0]=3 确认真在 HardFault。
3. 读 **CFSR**（`print (*(volatile uint*)0xE000ED28)`）→ 解码位域定类型。
4. 若 **BFARVALID=1**，读 **BFAR**（`0xE000ED38`）→ 故障地址。
5. Show Disassembly at Address 输入故障 PC（精确错用 CFSR 指向的 PC；非精确错用栈帧返回地址）→ 看指令。
6. 结合 **LR / 栈帧返回地址** 反推调用者。

> 不能跳过 CFSR 直接反汇编：遇 IMPRECISERR（非精确错误）指令地址无效。PRECISERR（BFARVALID=1）可走捷径：直接 BFAR + 反汇编，省 CFSR 精细解读。

---

## 五、一句话总结（面试/复盘）

> 启动卡死先看"卡在哪个死循环"——串口无数据 + 停在 Error_Handler，八成是时钟/初始化早期；
> 读 LR + 反汇编定位函数段，再按场景分流：普通死循环查调用栈，HardFault 必须 CFSR 解码。
> HSE 配 BYPASS 但板载无源晶振，是最典型"编译过、运行死"陷阱。

---

## 六、中断/异常上下文禁用阻塞式与不可重入 API（尤其串口打印）

### 机理
- ISR/异常上下文（含 HardFault / NMI / 任意 IRQ）运行在特权级、**不可被调度剥夺**；调阻塞 API（等信号量/队列、`HAL_Delay`、`printf` 内部 malloc）会**死锁**（调度器停了，等待物永不到）。
- 串口打印 `log_backend_putc` 轮询 `USART1->ISR/TDR`，**本身不可重入**（无串行锁）：
  - 异常上下文调它 + drain 任务 / Channel B 直发路径并发 → **字节流在 UART 线交错**（现象：日志前缀被吃、乱序）。
  - 若 USART 已故障（如 HardFault 由总线错引起）→ 二次 fault 或挂死（现象：HardFault 里卡 `bsp_uart1_emit` 的 `while(TXE)`、串口 921600 高速闪、PC 卡死无法关闭）。

### 工程铁律（复用）
- `HardFault_Handler` / 任意 ISR / `HAL_UART_ErrorCallback` 内：**只做"保存寄存器到全局 `volatile` + `while(1)`"**，
  **不调 `log_backend_putc`、不调 `logger_flush_to_flash`、不调任何 HAL/RTOS 阻塞 API**。
- 取证方式：把 `CFSR/HFSR/MMFAR/BFAR/PC/LR` 存入全局 `volatile`，停机后由 Keil Watch 读，或醒后由正常上下文打印。

### 与 HardFault 取证的关系
- 见本文件「四、HardFault 取证最小步骤」：取证在**正常调试会话**读寄存器，不在 handler 内打印。
- handler 内打印看似"省一步"，实则制造新故障（高速闪串口 / 二次 fault），**务必止血**。

### 关联实例
- 具体实例（含 `debug3` 触发真实 HardFault、BFAR=0x2E2E2E2E、卡 `bsp_uart1_emit`）见 `Error/logger_error.md` **事件 E28**。

---

## 附：文件关系

```
Components/Debug/Error/Error_Readme_idx.md                      运行期故障总导航（E 事件映射）
Components/Debug/Error/<TASK>_error.md         按任务的具体故障实例（事件 E N）
Components/Debug/Error/crash_error.md          本文件：启动卡死 / HardFault 通用取证方法论
Components/Debug/Error/Error_Readme_idx.md               编译/工程配置期故障（问题 N）
```
