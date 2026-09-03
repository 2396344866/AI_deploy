# uvprojx / CubeMX 生成 故障归档

> 本文件归 **uvprojx 工程文件 / CubeMX 生成** 类故障（CRLF、双登记、路径风格）。
> 原 `Components/Debug/Error/Error_Readme_idx.md` 的 **问题 2 / 问题 3 / 问题 4** 迁入此处。
> 编译/链接错 → 看 `Error/compile_link_error.md`；运行期 → `Error/<TASK>_error.md` + `Error/post_error.md`。

---

## 问题 2：`cannot create command input file '...\.__i'`
- **现象**：25 个 `cannot create command input file 'c:\h743obj\..\components\edgeimpulse\...'`。
- **根因**：EdgeImpulse 组 `FileName` 被写成**全路径**，Keil 拼输出 `.i` 时把路径拼到 C 盘根目录被拒。
- **处理**：所有 `FileName` 改为纯 basename（如 `arm_convolve_s8.c`），路径只放 `FilePath`。共 207 处。

## 问题 3：同一文件双登记（`_1.o` 重命名）
- **现象**：44 个文件各出现 2 条 `File` 记录，Keil 编译第二个时重命名为 `_1.o`。
- **根因**：CubeMX 每次 GENERATE CODE 追加带完整 `<FileOption>` 的新条目，旧裸条目（无 FileOption）未覆盖→同一文件两条。
- **处理**：按"有无 FileOption"判断去留，删裸条目、留 CubeMX 完整版。36 个重复被清。

## 问题 4（核心）：打开文件全空白 + 重新加载失败 + 177 Error
- **现象**：Keil 打开**每个源文件都是空白**；点"重新加载"弹失败；Rebuild 报 **177 Error**（集中在 `portmacro.h` 与 `tasks.c` 不匹配）。
- **根因（致命）**：uvprojx 换行符被改成 **LF**，而 **Keil 强制要求 CRLF**。`ET.fromstring` 能解析 LF（误报合法），Keil 直接拒绝→工程读成空壳。此前的 LF→CRLF 转换脚本因 shell 转义吞掉 `\r` **实际未生效**，验证代码也误报成功。
- **处理（用户手动）**：① 所有路径分隔符 `\` 改为 `/`；② FreeRTOS `port.c`/`portmacro.h` 移到 `Components/FreeRTOS_Port/ARM_CM4F/`，工程指向新位置；③ 恢复可编译状态 → Build 0 Error。
- **复盘铁律**：
  1. Keil uvprojx 必须用 CRLF；不可用脚本批量改写 uvprojx 后不二进制验证 CRLF（`b'\x0d\x0a'` 计数）。
  2. CubeMX 对"文件是否已登记"用字符串精确匹配；旧条目（`..\`）与新写（`../`）不等→每次生成追加→翻倍；所有路径必须统一 `../` 正斜杠才幂等。
  3. `FileName` 只放 basename，全路径放 `FilePath`，否则 Keil 拼 `.i` 输出路径到错误位置报 `cannot create command input file`。
  4. 头新源旧导致 L6218E：CubeMX 固件包升级后 `.h` 声明新函数但 `.c` 未实现，新建独立 `.c` 提供实现（放 Core/Src，避 CubeMX 覆盖）。
