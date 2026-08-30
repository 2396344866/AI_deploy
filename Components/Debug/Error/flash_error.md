# FLASH 任务运行期故障归档（W25Q64 自检 / 黑匣子）

> 本文件是 **FLASH 任务**（StartFlashTask + W25Q64 外置 Flash / 日志黑匣子落盘）的运行期故障归属地，
> 事件用全局连续编号 **E N**（与 `Components/Debug/error.md` 一致）。
> 联调笔记见 `Components/Debug/debug/flash_debug.md`（待建）；测试方案见 `Components/Debug/Test/flash_test.md`。
> 编译期故障归 `Doc/Keil_MDK_ARM_工程排错记录.md`（问题 N）；启动死机取证归 `Components/Debug/Error/crash_error.md`。

---

## 占位说明（按用户要求不编造故障）

> **当前状态**：FLASH 任务**暂无已记录的运行期故障**，本文件为预留模板。
> 原则：未实测到、未确认根因的事件**一律不编造条目**。
> 新增运行期故障时，直接在本文件追加「事件 E N」（编号接 `Components/Debug/error.md` 全局最大编号，不重置），
> 每个事件含「现象 / 调试器定位 / 根因链 / 修复方案 / 验证步骤 / 状态」六段（格式参考 `Error/motor_error.md`）。

> **重点排查方向（已知风险，非故障记录）**：`logger_flush_to_flash` 在崩溃路径下的可靠性——
> 时钟失效 / Flash 写入超时 → 看门狗复位 → 关键日志丢失。落地时优先在此登记相关 E 事件。

---

## 事件登记位（待填）
| 事件 | 现象摘要 | 状态 |
|------|----------|------|
| （暂无） | — | — |
