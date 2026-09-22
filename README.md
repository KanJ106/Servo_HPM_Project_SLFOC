# Servo HPM Project SLFOC

面向 HPM6E00/HPM6E80 伺服平台的 FOC 固件工程，包含现有编码器控制链、无感启动、SMO/PLL、CANopen 接口、板端诊断以及电流环源代码替换工作。

## 当前状态

- 电流快环按 16 kHz 配置，名义周期为 62.5 μs。
- 保留了 `ToqLoop_Calc`、逆 Park 和 `hypotf` 的 ILM 放置调整。
- 无感启动仍处于板端验证阶段；完整生产 ISR 的最坏执行时间尚未闭合。
- “强拉 → SMO 接管 → 独立运行 300 ms”尚未完成正式验收，不能据此认定无感启动已经可用于产品。

## 目录

- `Servo_HPM_Project/CtrlLoop/30_Drive/source`：当前已恢复或替换的电流环源代码。
- `Servo_HPM_Project/CtrlLoop/33_Sensorless`：无感估算、启动、保护、诊断及主机侧测试。
- `Servo_HPM_Project/CtrlLoop/43_CanOpen`：CANopen 协议及应用接口。
- `Servo_HPM_Project/CtrlLoop/60_User`：控制调度、中断和主程序入口。
- `Servo_HPM_Project/tools`：构建、板端核验和数据导出脚本。
- `Servo_HPM_Project/hpm_sdk_localized_for_hpm6e00evk`：工程使用的 HPMicro SDK 子集。

## 构建

使用 SEGGER Embedded Studio 打开：

`Servo_HPM_Project/hpm6e00evk_flash_xip_debug/segger_embedded_studio/Servo_HPM_Project.emProject`

默认板卡和链接布局针对现有 HPM6E00EVK 工程配置。烧录前必须重新核对目标硬件、电机参数、电流标定、母线电压、限位及急停条件。

## 源码完整性说明

`Servo_HPM_Project/CtrlLoop/70_Obj` 中仍包含原工程遗留的预编译目标文件。当前快环相关的部分实现已在 `CtrlLoop/30_Drive/source` 提供源码，但仓库暂时不是所有遗留控制模块的完整源码重建。预编译目标文件保留是为了维持现有工程的链接和复现能力。

本仓库不提交板端采样、ELF、历史 SDK 快照和本机 IDE 会话。它们体积较大，并且不是构建源码所必需。

## 许可

项目自有新增源码按根目录 `LICENSE` 提供的 MIT License 发布。HPMicro SDK、CANopenNode、SEGGER RTT、EasyLogger 等第三方内容继续适用各自文件头或随附许可；根许可证不替代第三方许可。未提供对应源码的遗留预编译目标文件不在 MIT 授权范围内。
