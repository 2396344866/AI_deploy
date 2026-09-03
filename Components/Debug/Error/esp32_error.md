# ESP32 任务运行期故障归档（ESP32-S3 CAM 图像结论帧）

> 本文件是 **ESP32 任务**（StartEsp32Task + ESP32-S3 CAM 图像协处理器 / FOMO 结论帧）的运行期故障归属地，
> 事件用全局连续编号 **E N**（与 `Components/Debug/Error/Error_Readme_idx.md` 一致）。
> 联调笔记见 `Components/Debug/debug/esp32_debug.md`（待建）；测试方案见 `Components/Debug/Test/esp32_test.md`。
> 编译期故障归 `Components/Debug/Error/Error_Readme_idx.md`（问题 N）；启动死机取证归 `Components/Debug/Error/crash_error.md`。

---

## 占位说明（按用户要求不编造故障）

> **当前状态**：ESP32 任务**暂无已记录的运行期故障**，本文件为预留模板。
> 原则：未实测到、未确认根因的事件**一律不编造条目**。
> 新增运行期故障时，直接在本文件追加「事件 E N」（编号接 `Components/Debug/Error/Error_Readme_idx.md` 全局最大编号，不重置），
> 每个事件含「现象 / 调试器定位 / 根因链 / 修复方案 / 验证步骤 / 状态」六段（格式参考 `Error/motor_error.md`）。

> **重点排查方向（已知风险，非故障记录）**：H743↔S3 串口帧同步（结论帧丢字节 / 校验错）、
> S3 端 FOMO 推理超时、N16R8 内存不足导致模型加载失败。落地时优先在此登记相关 E 事件。

---

## 事件登记位（待填）
| 事件 | 现象摘要 | 状态 |
|------|----------|------|
| （暂无） | — | — |
