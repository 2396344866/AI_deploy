# OTA 无线升级方案（STM32H743 + ESP-01S AT + 阿里云 IoT）

> 适用项目：`AI_deploy`（STM32H743VIT6，FreeRTOS，本地 AI 推理 + VOFA 串口遥测）  
> 目标：在不插烧录器的情况下，远程给现场设备批量更新固件（含 INT8 模型 + EdgeImpulse 球磨机检测模型）  
> 方案：**ESP-01S（MQTT AT 固件）+ 阿里云 IoT 平台 OTA + 自写 STM32 BootLoader + 双 APP 分区**  
> 配套图：`h7_flash_layout.svg`（Flash 分区，含代码区/RW/ZI 标注）、`bootloader_flow.svg`（升级流程）

### 修订记录

| 日期 | 变更 |
|---|---|
| 2026-09-03 | ① §3 分区表修正：原 APP2=`0x080C0000` 与 APP1 **重叠 384 KB**（`0x080C0000` 实为 S6 起点），`VECT_TAB_OFFSET` 同步错，改为 Bank 对齐的 `0x08100000`；② 下载职责**定死为「APP 下载、BootLoader 只校验/跳转」**（原 §4 与 §5.2/§5.3 自相矛盾）；③ OTA 缓冲由 W25Q64 改为**内部 Flash 双 bank 边下边烧**；④ 新增 §10 看门狗跨阶段方案、§11 与 Route C 的接口约定；⑤ 新增易错 10–14 |

> **体量基准（实测，非估算）**：`MDK-ARM/STM32H743VIT6/STM32H743VIT6.map` → Total ROM = **94,492 B（92.28 KB）**，RW+ZI = **77,480 B（75.66 KB）**。2 MB 内部 Flash 仅用 **4.6%**。这是"不做外部暂存、改边下边烧"的决定性依据。
>
> **命名统一**：本文硬件一律指 **ESP-01S**（乐鑫 ESP8266EX 的模组形态）。旧版本混称"ESP8266"，实为同一块硬件，现统一，避免与 ESP8266 裸芯片 / ESP32 系列混淆。

---

## 0. 你的疑问解答（先把这个看明白，再往下）

你在消息里提了 4 个具体困惑，这里一次性答掉，全部基于**当前代码实查**：

### 0.1 「代码区在哪里？我没看到啊」
- 当前工程用的是 **Keil 生成的默认 `.sct`**（`MDK-ARM/STM32H743VIT6.sct`，单镜像 `LR_IROM1 0x08000000 0x00200000`；IRAM 段 `0x20000000` 128KB + `0x24000000` 512KB），**没有自定义分散加载、没有 BootLoader 槽**。等价于"Options → Target 默认 IROM1 = 0x0800_0000 / 2MB"。
- 所以**整个 APP（含你的代码、FreeRTOS、所有 `const` 模型权重）都从 0x0800_0000 开始往高地址排**，向量表（startup 的 `RESET`/`__Vectors`）就在这个地址开头。这就是"代码区"。
- 做 OTA 时，必须把 APP 从 `0x0800_0000` **挪到 `0x0804_0000`（APP1 槽）**，并新建 BootLoader 占 `0x0800_0000`。这意味着要**改成分散加载（自定义 .sct）+ 改 `VECT_TAB_OFFSET`**。你"没看到代码区"是因为它现在和 BootLoader 空间叠在一起、还没拆。

### 0.2 「RW data 段、ZI data、变量名初始也在 Flash？都是在 RAM 更新吧？」
对，你的理解基本正确，补精确版：

| 段 | 链接位置 | 上电时谁来搬 | 运行时在哪 |
|---|---|---|---|
| `.text`（代码） | **内部 Flash** | 不搬，XIP 直接跑 | Flash |
| `.rodata`（const，含 `model_weights.h`、EdgeImpulse 模型常量） | **内部 Flash** | 不搬 | Flash（只读） |
| `.data`（已初始化变量，RW） | **初始映像在 Flash 末尾**，运行时**拷到 RAM** | `__main`（C 库启动） | RAM（D-TCM/AXI SRAM） |
| `.bss` / ZI（零初始化变量） | 仅占 RAM 空间，无 Flash 映像 | `__main` 清零 | RAM |
| `.tensor_arena`（EdgeImpulse 推理工作区） | 显式 `__attribute__((section(".tensor_arena")))` | 不搬（运行时分配） | RAM（见 tflite_learn_*.cpp:113） |

- 关键点：**只有 `.data` 的初始值在 Flash 里有一份拷贝**（供 `__main` 上电搬进 RAM），搬完 Flash 那份就没用了；变量运行期全在 RAM，OTA 升级的是 Flash 里的 `.text/.rodata/.data映像`，RAM 内容由新 APP 重启后重新初始化。
- 也即：OTA 升级**只改内部 Flash 的只读/初始映像区**，RAM 布局由新固件自己 `__main` 重建，不存在"要单独搬 RAM"的问题。

### 0.3 「模型参数部分存储 Flash 已实现，只需改存储位置到 APP1？」
要分**两类参数**，处理方式不同，别混：

**(A) 编译进固件的模型权重（随 APP 走，OTA 自动覆盖）**
- `Components/Fault_Diagnosis/Inc/model_weights.h`（`const float` 数组，freertos.c 包含）→ 链在 `.rodata` → **内部 Flash 代码区**。
- `Components/EdgeImpulse/tflite-model/` 的球磨机目标检测模型权重（`ei_*` 常量、tflite compiled 里的 `const` 数组）→ 同样链在 `.rodata` → **内部 Flash 代码区**。
- 这两类**不需要你手动"改存储位置到 APP1"**——只要 APP 整体被链接到 APP1 槽（0x0804_0000），它们自然跟着进 APP1。OTA 时整个 APP.bin（含这两个模型）一起被写进 APP2 备用槽，激活后无缝切换。**球磨机检测的参数是 APP 固件的一部分，和 Fault_Diagnosis 权重待遇完全相同，不用单独对应、不用单独 OTA 通道。**

**(B) 运行时存进 W25Q64 的参数（OTA 不碰，独立保留）**
- 你已实现"模型参数存 Flash"指的是 **W25Q64**（SPI1，PA4 CS）：磁标定硬铁/软铁矩阵（约 12 个 float）、崩溃黑匣子日志（`w25q_crashlog_save`）等。
- 这些在**外部 SPI Flash**，不在内部 Flash 代码区，**OTA 升级内部 Flash 的 APP 完全不影响它们**，上电照常从 W25Q64 加载。无需为 OTA 改动这部分。

> 一句话：编译期权重（两类模型）→ 进 APP 固件 → OTA 整包覆盖；运行期标定数据 → 在 W25Q64 → OTA 碰不到、自然保留。两者解耦，正是你想要的结构。

### 0.4 「APP2 部分如何处理？」
- APP2 是**升级备用槽**，平时不跑。逻辑：
  1. 新固件下载完先写 **APP2**，CRC32 校验通过才把 `active_slot` 切到 APP2。
  2. 重启后 BootLoader 跳 APP2；新 APP 心跳 3 分钟成功 → 固化 `FLAG=OK`（此后 APP2 变"当前"）。
  3. 下一次升级时，**旧的 APP1 变成新的备用槽**，下载写回 APP1。如此两槽交替，永远保留一份可回滚的旧固件。
- APP1/APP2 **大小必须相等**（各 **768KB**），两份构建用同一份 `.sct` 模板、只是 `ROM_START` 不同（`0x08040000` vs `0x08100000`）。
- ⚠️ **必须出两份 bin，一份产物不能两槽通用**。镜像里的函数指针、`const` 表、以及**向量表的内容**都是链接期定死的**绝对地址**；把 APP1 的 bin 烧进 APP2，其复位向量仍指向 `0x0804xxxx` → 会跳回 APP1 区域执行。因此 Keil 需要 `APP_SLOT1` / `APP_SLOT2` 两个 target，云端按 `!active_slot` 分发对应那份（详见易错 10）。

---

## 1. 为什么选「ESP-01S(AT) + 阿里云」

| 方案                       | 你的契合度                                                       | 结论       |
| ------------------------ | ----------------------------------------------------------- | -------- |
| **ESP-01S AT + 阿里云 OTA** | 你已完成 ESP-01S 的阿里云完整对接（含 MQTT，驱动见 `Components/BSP/ESP/Src/esp01s.c`），连云链路零门槛，云平台控制台直接推固件 | ✅ **首选** |
| 4G Cat.1（合宙 Air724）      | 覆盖广但要学 LuatOS / 新模组，无 Wi-Fi 场景才需要                           | 现场无网时再上  |
| 自建 HTTP 服务器              | 最灵活最累，签名加密版本管理全自己写                                          | 不推荐起步    |
| BLE/Nordic DFU           | 短距离，手机传包，不适合批量远程                                            | ❌        |

**你已完整对接过 ESP-01S 的阿里云 MQTT**，连云链路（三元组、MQTT 订阅/发布、JSON 报文）是现成资产，直接复用。ESP-01S 出厂默认 AT 固件不支持 MQTT 指令，需刷入**带 MQTT 的 AT 固件**——既然你已成功对接过，说明这块固件/配置已在手，直接沿用。

---

## 2. 硬件采购与现状（更新：你已基本买齐）

### ✅ 你已完成（不用再买）
| 成品 | 状态 | 备注 |
|---|---|---|
| **ESP-01S 模块** | ✅ 已购 | 需确认已刷带 MQTT 的 AT 固件（你之前对接成功过，应在手） |
| **W25Q64 SPI Flash** | ✅ 已购 + 驱动完 + 测试完 | SPI1（PA4 CS / PA5-7），含崩溃黑匣子，见 `Components/BSP/W25Q64/Src/BSP_W25Q64.c` |
| **串口助手** | ✅ 已有 | 调试 ESP-01S AT 指令用 |

### ✅ 串口分配（已落地，无需再补）

| 外设 | 引脚 | 波特率 | 用途 |
|---|---|---|---|
| USART1 | PA9/PA10 | 921600 | VOFA 遥测 + 日志 + 串口控制台 |
| **USART2** | **PA2/PA3** | **115200** | **ESP-01S（AT/MQTT）—— 驱动已落地，`esp01s.c` 用 `huart2`** |
| USART3 | — | 115200 | 预留 |
| UART4 | PA0/PA1 | 115200 | 淘晶驰串口屏 |
| USART6 | — | 921600 | ESP32-S3 图像协处理器 |

- ⚠️ **DMA 现状（2026-08-26 核实）**：`usart.c` 实际仅生成 `hdma_usart1_rx`，**USART2 未配 DMA**；`esp01s.c` 已改用 `HAL_UARTEx_ReceiveToIdle_IT` 中断接收（与 ESP32-S3 同决策，无需动 CubeMX）。
  → **OTA 沿用这套中断接收即可，不必为 OTA 补 DMA**：92 KB 固件在 115200bps 满速约 6.6 s，串口不是瓶颈；OTA 真正的耗时大户是 **Flash 擦除**（见易错 12）。若日后确需 DMA，须在 `.ioc` 补未被占用的流（`DMA1 Stream5/6` 或 DMA2），避让 `DMA1 Stream2/3`（SPI1/W25Q64）与 `DMA1 Stream4`（USART1_RX）。

### 🟡 按需
| 成品 | 何时买 | 参考价 |
|---|---|---|
| 合宙 Air724UG 4G DTU | 设备无 Wi-Fi 覆盖的现场 | ¥160~280 |

### 🟢 可靠性：片内已有，无需采购

| 机制 | 现状 | 用途 |
|---|---|---|
| **IWDG1**（片内，LSI 32 kHz） | ✅ 已在 `.ioc` 配置（Prescaler=32 / Reload=4095 ≈ **4.1 s**）；2026-09-02 已改为**三阶段监管**并启用（`APP_ENABLE_WATCHDOG=1`） | 新固件起不来 / 擦写卡死 → 复位触发回滚，见 §10 |

> 板子 **STM32H743VIT6 = 2MB Flash**，双 APP（各 768KB）装 92KB 的 APP 绰绰有余，**不用为空间或可靠性买任何东西**。

---

## 3. Flash 分区（详见 h7_flash_layout.svg）

> 基准：H743 = 2MB / 128KB = **S0–S15**；**Bank1 = S0–S7（`0x0800_0000`–`0x080F_FFFF`）**，**Bank2 = S8–S15（`0x0810_0000`–`0x081F_FFFF`）**。

| 扇区 | 地址 | Bank | 用途 | 大小 |
|---|---|---|---|---|
| S0 | 0x0800_0000 | 1 | **BootLoader**（判定/校验/切槽/跳转，**不含网络**，独立工程） | 128 KB |
| S1 | 0x0802_0000 | 1 | 参数区：`ota_param_t`（FLAG / version / crc / img_size / active_slot / pending_slot / boot_attempt） | 128 KB |
| S2–S7 | 0x0804_0000 | 1 | **APP1**（槽 A = .text + .rodata[含两类模型权重] + .data 映像） | 768 KB |
| S8–S13 | 0x0810_0000 | 2 | **APP2**（槽 B） | 768 KB |
| S14–S15 | 0x081C_0000 | 2 | 预留（参数镜像 / 出厂恢复镜像 / 文件系统） | 256 KB |

> 🔴 **旧版分区表有硬错误，已修正**：原表 APP1 写 `S2–S8 @0x08040000` + 896KB（结束于 `0x0811_FFFF`），APP2 却写 `0x080C_0000`——而 `0x080C_0000` 实为 **S6 起点**，两槽**重叠 384 KB**，`VECT_TAB_OFFSET` 的 `0xC0000` 同步错。且"S2–S8"本身跨 Bank 边界（S8 已属 Bank2），无论怎么摆都不满足 RWW。

要点：
- **每槽完整落在单个 Bank 内** —— 这是本表的核心约束，不是凑整。APP1 全在 Bank1、APP2 全在 Bank2，才吃得到 H7 的 **read-while-write**：从 APP1 运行时烧 APP2（或反之）CPU 不 stall。槽若跨 Bank 边界，烧到同 Bank 那段时取指停摆，OTA 期间整个系统卡死数秒。
- `VECT_TAB_OFFSET`：APP1 = `0x40000`，APP2 = **`0x100000`**（**不是旧版的 0xC0000**）。在 `system_stm32h7xx.c` 改，且 BootLoader 跳之前 `SCB->VTOR` 也要设对。
- `active_slot` 决定跳哪个槽；**下载永远写 `!active_slot`**（备用槽），见 §4。
- 参数区在 S1（Bank1）：从 APP2 跑时写参数零 stall；从 APP1 跑时写参数有短暂停顿（同 Bank）。可接受——参数写入只在状态迁移时发生（每轮 OTA 几次），不是热路径。若要彻底消除可在 S14 放镜像区。
- **编译期模型权重**（Fault_Diagnosis `model_weights.h` + EdgeImpulse 球磨机检测）都在 APP 固件的 `.rodata` 里，**随 APP 整包升级自动覆盖**，不单独开通道（见 §0.3）。
- **运行期参数**（磁标定、崩溃黑匣子）在外部 **W25Q64**，OTA 碰不到、自然保留。
- ⚠️ **W25Q64 不再承担 OTA 缓冲**（旧版设计，已废弃）：APP 仅 92KB、内部 Flash 余 1.9MB，没有外部暂存需求；且 W25Q64 已承载磁标定 + 崩溃黑匣子，再叠 OTA 缓冲就是三用途共享一片、无任何分区隔离的踩踏风险。OTA 改走**内部 Flash 边下边烧**，冲突自动消解（见 §5.3）。

---

## 4. 升级流程（详见 bootloader_flow.svg）

> **职责定死**（旧版 §4 与 §5.2/§5.3 自相矛盾，此处统一）：
> **APP 负责下载 + 烧写，BootLoader 只负责「判定 → 校验 → 切槽 → 跳转」，BootLoader 不含任何网络代码。**
>
> 理由：① BootLoader 必须"永远不需要被升级"——升级它是唯一无法回滚的操作（BootLoader 刷坏 = 真砖）；② 网络协议栈（AT/MQTT/HTTP）恰恰是最易变的部分，塞进 BootLoader 等于把必然迭代的东西锁进不可回滚的位置；③ `esp01s.c` 在 APP 工程，BootLoader 要用只能复制一份，直接制造第二真相源。BootLoader 只留 Flash 驱动 + CRC32 + 跳转，约 8–15KB。

**状态机**（`FLAG ∈ {OK, UPDATE, PENDING}`）

| FLAG | 含义 | BootLoader 动作 |
|---|---|---|
| `OK` | 常态 | 校验 `active_slot` → 直接跳 |
| `UPDATE` | 已受理升级任务 | **仍跳当前 `active_slot` 的 APP**，下载由 APP 完成 |
| `PENDING` | 新固件已烧入备用槽，待确认 | **重算备用槽 CRC** → 通过则切 `active_slot` 并跳；失败/超次则回滚 |

**时序**

1. 上电 → BootLoader(S0)：`IWDG_Start()`（裸轮询喂狗，见 §10）→ 读 S1 参数。
2. `FLAG == OK` → 校验 active 槽栈顶合法 → 跳 APP。
3. `FLAG == UPDATE` → **跳当前 active APP**（BootLoader 不进下载模式）。APP 起来后：
   - 订阅阿里云 OTA topic，拿到 `{version, size, url, sign}`；
   - 按 `!active_slot` 挑对应构件（APP1 版 / APP2 版 bin，见易错 10）；
   - HTTP 分包拉取 → **边收边烧进备用槽**（不落 W25Q）。
4. 收完 → **镜像自检**（MSP / 复位向量 / 目标槽范围，见易错 11）→ 整包 CRC32 与云端 `sign` 比对。
5. 通过 → 写 S1：`pending_slot = !active_slot`、新 version、`crc32`、`img_size`、`FLAG = PENDING`、`boot_attempt = 0` → `NVIC_SystemReset()`。
6. 重启 BootLoader 见 `PENDING` → **自己重算一遍备用槽 CRC（不信 APP 写的 flag）**：
   - 通过 → `active_slot = pending_slot`、`boot_attempt = 0`，**仍保留 PENDING**（交给心跳固化）→ 跳新 APP；
   - 失败 → `boot_attempt++`；`>= OTA_BOOT_MAX(3)` 则回滚（见下）。
7. 新 APP 启动 → **3 分钟内 MQTT 心跳连续成功** → APP 自己写 `FLAG = OK` 固化，升级完成。
   - 心跳失败 / 根本起不来 → 每次进 `PENDING` 都 `boot_attempt++`，`>= 3` 则 `active_slot` 切回旧值 + `FLAG = OK` + 上报回滚事件 → 复位。

> **两步确认缺一不可**：第 6 步的 CRC 重算兜住"烧坏了 / 烧一半断电"；第 7 步的心跳兜住"烧对了但业务层跑飞（联网失败、推理崩溃）"。少任何一步都会留下变砖窗口。
>
> **不做断点续传**：92KB 全包重下只需 ~7s，比维护 offset 状态简单得多。⚠️ 但**重下前必须重新擦除**——Flash 只能 1→0，写过的位置未擦不能改写。

---

## 5. STM32 端代码框架（C 伪代码 + 关键函数）

> 框架骨架，非完整可编译工程。重点展示职责划分与易错调用。

### 5.1 参数区结构（S1）

```c
/* ota_param.h —— 参数区结构，存于 0x08020000 (S1) */
#define OTA_PARAM_ADDR   0x08020000U
#define OTA_MAGIC        0xABADC0DEU
#define APP1_ADDR        0x08040000U   /* VECT_TAB_OFFSET = 0x40000 */
#define APP2_ADDR        0x080C0000U   /* VECT_TAB_OFFSET = 0xC0000 */
#define APP_SLOT_SIZE    (896*1024U)

typedef struct {
    uint32_t magic;         /* OTA_MAGIC，掉电后仍能识别 */
    uint32_t active_slot;   /* 0=APP1, 1=APP2 */
    uint32_t ota_flag;      /* 0=OK, 1=UPDATE, 2=PENDING */
    uint32_t new_version;   /* 待激活/已激活版本号 */
    uint32_t crc32;         /* 整包 CRC，校验时用 */
    uint32_t heartbeats_ok; /* 新固件心跳计数 */
} ota_param_t;
```

### 5.2 BootLoader 主逻辑（S0 独立工程）

```c
/* bootloader_main.c —— 独立工程，链接到 0x08000000 */
void bootloader_run(void) {
    ota_param_t p;
    flash_read_param(&p);                     /* 从 S1 读 */

    if (p.magic != OTA_MAGIC) {               /* 首次上电/参数损坏 */
        p.active_slot = 0; p.ota_flag = 0;
        flash_write_param(&p);
    }

    if (p.ota_flag == OTA_FLAG_UPDATE) {      /* 需要升级 */
        if (esp_download_and_verify() == OK) {/* MQTT收URL→HTTP拉包→CRC32 */
            flash_erase_slot(!p.active_slot); /* 擦备用槽 */
            flash_write_slot(!p.active_slot); /* W25Q → 内部Flash */
            p.ota_flag = OTA_FLAG_PENDING;
            p.active_slot = !p.active_slot;   /* 切到新槽 */
            flash_write_param(&p);
            NVIC_SystemReset();               /* 软件复位 */
        }
    }

    uint32_t app_addr = (p.active_slot == 0) ? APP1_ADDR : APP2_ADDR;
    if (app_is_valid(app_addr)) {             /* 检查栈顶指针合法性 */
        jump_to_app(app_addr);                /* 见 5.5 */
    }
    while (1);                                /* 都不合法：停 BootLoader */
}
```

### 5.3 ESP8266 下载 + 校验（APP 工程内，USART2 驱动）

```c
/* esp_ota.c —— 在 APP 里跑，下载阶段由 APP 主导而非 BootLoader */
int esp_download_and_verify(void) {
    /* 1) ESP-01S(USART2) 已通过 AT 连 Wi-Fi + 连阿里云 MQTT（你做过，略） */
    /* 2) 订阅阿里云 OTA 下发 topic（见 §6 易错 7）*/
    /* 3) 收到 { "version":x, "size":N, "url":"https://.../x.bin", "sign":"CRC32" } */
    /* 4) AT+CIPSTART 建 TCP/SSL 连 URL 主机，HTTP GET 拉包（走 USART2 收发 AT）*/
    /* 5) 分包写入 W25Q（每包 512B，记 offset 实现断点续传）*/
    /* 6) 整包算 CRC32，与云端 sign 比对 */
    if (crc32_of_w25q != expected) return ERR_CRC;
    return OK;
}
```

### 5.4 CRC32（整包完整性，必须有）

```c
uint32_t crc32_calc(const uint8_t *buf, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320U & (-(int32_t)(crc & 1)));
    }
    return ~crc;
}
```

### 5.5 跳转到 APP（最易错：向量表重映射）

```c
void jump_to_app(uint32_t app_addr) {
    __disable_irq();
    HAL_RCC_DeInit();
    HAL_DeInit();
    SCB->VTOR = app_addr;                     /* 漏这步必 HardFault */
    uint32_t msp = *(volatile uint32_t*)app_addr;
    __set_MSP(msp);
    uint32_t reset = *(volatile uint32_t*)(app_addr + 4);
    void (*app_reset)(void) = (void(*)(void))reset;
    app_reset();
}
```

> **APP 工程里也必须设 `VECT_TAB_OFFSET`**（`system_stm32h7xx.c` 的宏，或 `HAL_` 之后 `SCB->VTOR = FLASH_BASE + offset`），否则 APP 自己跑中断也会 HardFault。

### 5.6 心跳回滚（新 APP 启动后）

```c
void app_heartbeat_task(void) {
    if (ota_param.ota_flag == OTA_FLAG_PENDING) {
        if (mqtt_publish_heartbeat() == OK) {
            ota_param.heartbeats_ok++;
            if (ota_param.heartbeats_ok >= 18) {  /* 3分钟 */
                ota_param.ota_flag = OTA_FLAG_OK;
                flash_write_param(&ota_param);    /* 固化，不再回滚 */
            }
        } else if (now() - boot_time > 180000) {
            ota_param.ota_flag = OTA_FLAG_OK;
            ota_param.active_slot = !ota_param.active_slot; /* 切回旧槽 */
            flash_write_param(&ota_param);
            NVIC_SystemReset();
        }
    }
}
```

### 5.7 ESP-01S 接 USART2 对接要点（新增，对应你"要重分配串口"）

- **硬件**：ESP-01S 的 TX/RX 接 STM32 的 **USART2_RX(PA3) / USART2_TX(PA2)**（交叉接，ESP TX→PA3，ESP RX→PA2）。ESP-01S 3.3V 供电，注意电流（峰值 ~300mA）单板 LDO 要够。
- **CubeMX**：启用 USART2，模式 Asynchronous，波特率 **115200**（和 ESP-01S AT 默认一致；你 VOFA 那次是忘了在 PC 端改波特率，ESP AT 这边初始就设 115200 即可）。开 RX 的 **DMA + 空闲中断**（仿 `Doc/BSP.md` 里 USART1 的写法）。
- **DMA 流避让**：USART2_RX 选 `DMA1_Stream5`（或 DMA2 任一空闲流），**不要**用 Stream2/3（SPI1/W25Q64）和 Stream4（USART1_RX）。`.ioc` 里确认无冲突。
  - ⚠️ 现状：`usart.c` 仅 `hdma_usart1_rx`，USART2 暂无 DMA。若沿用 §2.1/§2.2 的中断接收范式则无需改此步；若改 DMA 范式须先在 `.ioc` 补配后重生成。
- **协议**：AT 指令走 USART2 轮询/中断收发；MQTT 连接、订阅 `/sys/{pk}/{dn}/thing/ota/firmware/get`、发布进度到 `/thing/ota/update`，全部沿用你已验证的阿里云对接代码，只是把底层串口从"之前的 ESP8266 接线"换到 USART2。
- **调试**：先用串口助手单独给 ESP-01S 发 `AT`、`AT+GMR`、`AT+MQTTUSERCFG?` 确认固件与 MQTT 可用，再接到 STM32 USART2。

---

## 6. 易错点（重点！按踩坑概率排序）

### ⚠️ 易错 1：ESP-01S 默认固件不支持 MQTT
- 出厂 AT 固件只有 `AT+CIPSTART` 这类 TCP 指令，**没有 `AT+MQTT`**。必须刷带 MQTT 的 ESP-AT 固件。
- 验证：`AT+MQTTUSERCFG?` 有响应才算刷对。你已完整对接过，**固件/三元组已在手，直接复用**。

### ⚠️ 易错 2：向量表没重映射（跳 APP 即 HardFault）
- 漏 `SCB->VTOR = app_addr` → APP 一进中断就飞。BootLoader 跳之前设一次，**APP 工程 `system_stm32h7xx.c` 也要设对应 offset**。两处都要。

### ⚠️ 易错 3：Flash 扇区没按边界对齐
- H743 是 128KB/扇区。APP 起始地址必须落扇区起点（0x08040000、0x080C0000），自定义 `.sct` 的 `ROM_START` 必须改，否则跑飞。

### ⚠️ 易错 4：CRC32 算法两端不一致
- 云端与 STM32 端 CRC32（多项式 `0xEDB88320`、初始 `0xFFFFFFFF`、结果取反）必须完全一致，否则永远校验失败。先用已知文件在 PC 和 STM32 对拍。

### ⚠️ 易错 5：下载中途掉电 = 变砖（必须双备份）
- 写 APP 槽时掉电，该槽损坏。靠「先写备用槽 + 校验通过才切激活 + 旧槽保留」保证能回旧版本。W25Q 缓冲 + 断点续传：记录已写 offset，重连后从 offset 继续。

### ⚠️ 易错 6：回滚条件设计反了
- 错误：新 APP 启动失败但 `active_slot` 已切到新槽 → 重启又跳坏的。正确：PENDING 阶段**先不固化**，心跳成功（≥3分钟）才 `FLAG=OK`；失败则把 `active_slot` 切回旧值再复位。

### ⚠️ 易错 7：阿里云 OTA topic 拼错
- 订阅：`/sys/{pk}/{dn}/thing/ota/update`（上报进度）、`/sys/{pk}/{dn}/thing/ota/firmware/get`（拉 URL）。pk/dn 三元组填错收不到指令。你做过阿里云对接，复用老配置。

### ⚠️ 易错 8：APP 与 BootLoader 时钟/外设初始化冲突
- 跳转前 BootLoader `HAL_DeInit()` 复位外设；APP 再正常 `SystemClock_Config()`。否则外设状态残留导致 APP 异常。

### ⚠️ 易错 9（新增）：新开 USART2 的 DMA 流与 W25Q64/SPI1、USART1 冲突
- SPI1 占 DMA1 Stream2/3，USART1_RX 占 DMA1 Stream4。USART2_RX 必须选空闲流（如 DMA1 Stream5 / DMA2），否则编译期或运行期 DMA 互踩。配完在 `.ioc` 核对 DMA 请求映射。
  - ⚠️ 现状：`usart.c` 仅 `hdma_usart1_rx`，USART2 当前**无 DMA 句柄**；§2.1/§2.2 已用中断接收落地。此条仅在校验"§2.3 真要走 DMA"时才相关。

---

## 7. 落地步骤建议（循序渐进）

1. **先在 `.ioc` 开 USART2（PA2/PA3）+ 空闲中断 DMA（避让流）**，单独写小程序用串口助手验证 ESP-01S AT/MQTT 能连阿里云。
2. **单独写 BootLoader 工程**，只做「读 flag → 跳 APP1」，先在 Keil 切 `VECT_TAB_OFFSET` + 自定义 `.sct` 跑通双工程跳转（不加网络）。
3. APP 工程链接地址改到 0x08040000，验证从 BootLoader 跳过去能正常跑（VOFA、推理都在）。
4. APP 里实现「收 URL → HTTP 拉包 → 写 W25Q → CRC32」（ESP-01S 走 USART2）。
5. BootLoader 增加「写 APP2 + 切槽 + 复位」。
6. 加心跳回滚。
7. 阿里云控制台发一次正式 OTA 包，端到端验证。

---

## 8. 与本项目现状态的衔接

- 现有代码**无 BootLoader、无 Flash 分区、无联网模组接入 APP**（ESP-01S 待接 USART2），OTA 是「从零加」，不是改一行。
- `StartInferenceTask` 只跑一次写 `g_Test_results`（`macro_precision=0.91128` 实测正常），**推理逻辑不用动**，它随 APP 整包进 APP1/APP2。
- **两类模型权重**（Fault_Diagnosis INT8 + EdgeImpulse 球磨机检测）都编译进 APP 固件 `.rodata`，OTA 整包覆盖，不用单独处理球磨机参数。
- **W25Q64 里的运行期参数**（磁标定、崩溃黑匣子）OTA 碰不到，自然保留。
- VOFA 串口遥测（firewater 二进制帧，USART1）保留；升级进度可走 MQTT（USART2→ESP-01S）上报，不挤占 VOFA。
- 调试 OTA 时**别用瞬时 watch 下结论**，盯 `ota_param.ota_flag` 状态机更靠谱（同 E18 教训）。

---

## 9. 球磨机图像目标检测：先做还是 OTA 先做？

**结论：建议先把球磨机检测（EdgeImpulse 部分）做到可集成进 APP 的形态，再上 OTA 框架——但两者不必串行到"完全做完"。**

理由：
1. **OTA 升级的是"整个 APP 固件"**。球磨机检测模型（EdgeImpulse 权重 + `ei_run_classifier`）是 APP 固件的一部分（§0.3/§3）。如果先做完 OTA、再往 APP 里塞球磨机检测，等于**第一次正式 OTA 就要带着球磨机检测一起发**——这反而最简单，因为 OTA 框架只需搭一次，后续模型迭代都走 OTA。
2. **但 BootLoader 双分区对 APP 体积有硬约束**：APP1/APP2 各 896KB。你现在 Fault_Diagnosis(~64KB 权重) + FreeRTOS + 驱动已占一部分；球磨机检测（96×96 输入、tensor_arena）会再吃掉几十~一百多 KB Flash + 大量 RAM（tensor_arena 在 RAM，不占 Flash）。**建议先把球磨机检测编译进 APP，量一次 `.map` 的 ROM/RAM 占用**，确认 896KB 槽装得下、RAM（尤其 D-TCM/AXI SRAM 总量）够，再定 BootLoader 的槽大小——避免"OTA 做好了发现 APP 塞不进槽"返工。
3. **实操顺序推荐**：
   - 阶段 A（现在）：把球磨机检测集成进当前 APP（0x08000000 单镜像），跑通推理，`armlink` 出 `.map` 记录 ROM/RAM 峰值。
   - 阶段 B：依据阶段 A 的体量，定 BootLoader + 双 APP 槽大小（必要时 APP 槽从 896KB 调，或确认 896KB 够），搭 BootLoader 跳转（§7 步骤 2-3）。
   - 阶段 C：接 USART2 + ESP-01S，做下载/校验/回滚（§7 步骤 4-7）。
   - 阶段 D：把"带球磨机检测的最终 APP"作为第一个正式 OTA 包发出去，验证端到端。
4. **不建议**：先花大力气把 OTA 完全做绝（含回滚全验证）却用一个"不含球磨机检测"的 APP 去验，因为最终 OTA 必然要带球磨机检测，早带早验、少返工。也不建议把球磨机检测"做到 100% 完美"才碰 OTA——两者解耦，OTA 框架搭好后，模型迭代本就是 OTA 的用武之地。

---

*归档：本方案为 `AI_deploy` 项目 OTA 设计稿（含代码区/RW-ZI 划分、双 APP 与 EdgeImpulse 参数对应、ESP-01S 接 USART2 方案、球磨机检测时序建议），配套 `h7_flash_layout.svg`、`bootloader_flow.svg`。故障与误判归 `Components/Debug/Error/Error_Readme_idx.md`。*
