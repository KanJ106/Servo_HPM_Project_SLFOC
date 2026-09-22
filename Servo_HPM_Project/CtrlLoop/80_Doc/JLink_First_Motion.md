# J-Link 首轮实板调试
日期：2026-09-18。对象：48 V、星型、10 极对、101:1、无抱闸关节模组。
本次交付是可构建候选固件和调试入口；没有下载、连接目标 CPU、使能或实测旋转。
用户已确认 J-Link 连接成功；48 V 断开后 MCU 无法保持供电。因此在线读数应在 48 V 供电、Servo OFF 下进行；这里没有通过工具独立复核目标连接。
断电会丢失 RAM 记录。正常测试结束后保持供电、确认输出已关闭再导出；异常时优先断电，不为保存波形延误关断。
COM4 是 USB–RS485，本流程不使用 COM4，也不要求 CAN 心跳。

## 固件与入口
选择 hpm6e00evk_flash_xip_debug/segger_embedded_studio/Servo_HPM_Project.emProject 的 BANK0。
目标配置为 HPM6E80xVMx、JTAG。必须与实板芯片、引脚和 Flash 布局核对。
该配置带有 target_loader_erase_all=Yes，不可直接按 Download 覆盖未知板上程序。
先保留原固件及参数备份，确认引导程序/参数区再决定下载方式。本轮没有更改擦除策略。
BIN、ELF、MAP 位于同目录 Output/BANK0/Exe；以 diagnostics/source_foc_check/bank0_manifest.json 的 SHA256 对应本次构建。
旧固件版本号沿用 0x20260918；必须同时核对构建清单和新符号 g_sensorless_jlink，不能仅凭版本号判断新旧。

g_sensorless_jlink 是独立 RAM 邮箱（ABI 1，52 字节，已用本次 ELF 核验），不改变原生产邮箱 ABI 7 / CAN OD 布局。
初始化不使能。request_seq 最后写入，acknowledge_seq 和 result 表示请求是否接受。
owner/enable_requested/elapsed_ms/exit_reason 是状态镜像，直接修改这些镜像不会改变内部授权。
command：1=单次定位或 I/F 测试，2=撤销并中止，3=停机释放所有权。
命令 1/3 要求 arm_key=0x4A4C494E；消费后清零。命令 2 不需要 key。
result：0 接受、1 ABI/key 错、2 忙、3 参数无效、4 未停机。
exit_reason：1 总期限到、2 主动中止、3 启动拒绝、4 故障、5 正常完成；保留首次退出原因。
启动时固化 direction、target_speed_rpm、duration_ms（1..10000 ms）。
期限从接受命令起算，包含等待正常使能的时间，修改邮箱 duration_ms 不会续期。
调试期间 CAN 写入被拒绝，正常完成/故障后所有权继续保留；显式释放后才允许 CAN 控制。
独立入口只允许 ALIGN=1、IF=2，不允许 FULL=0/HFI；I/F 还要求配方明确允许连续旋转。
原 CAN 完整启动流程仍保留。

## 先读数，再准备第一次运动
1. 首次连接/下载/断点检查在功率输出已独立禁止的条件下进行；确认 J-Link 已接板，不能只看 USB 枚举。
   若 48 V 同时给控制电供电，先确定板上可用的独立功率禁止方式，不能假定断开 48 V 后仍可调试。
2. 加载匹配 ELF，读 StateMachine、FaultP、DrvCoeff、AdRead 和有效配方。
   核对母线读数对应实际 48 V，电流零点合理、相序和采样方向正确，再开放电流控制。
   源码中电流基值按 0.01*DrvCoeff.DrvIbase A 使用；母线反馈按 AdRead.Udc(Q24)*DrvCoeff.UdcBase V 使用。
   这些是当前软件约定，仍须与板上采样链校验。48 V 不是自动等于电压标幺基值。
3. 在 Servo OFF/IDLE 下通过生产邮箱 command=12 应用完整配方，确认 ack/result，再应用 command=2 的启动调参。
   tools/motor_48v_157w.pending.json 仍是待核对数据，不可直接写入启动。
   Rs 可参考星型两端平均电阻的一半约 0.2317 ohm，未扣除线缆和温度影响。
   用户补充的相电感 0.46 按之前单位暂记 mH，频率未知；不能宣称已确定 Ld/Lq。
   极对数采用已确认的 10；首次限流不采用 13.5 Arms 瞬时最大值。
4. 参数操作可复用 tools/startup_debug_v7.gdb 的 sl7_mode/sl7_readback。
   应用配方后会重建启动默认值，因此先配方、后启动参数，并读回实际结果。
   软件仍保留原抱闸状态调度和准备延时；硬件无抱闸不代表绕过这些状态。
5. 清除所有运行路径断点。先定位短测，再 I/F 短测。机械行程、固定方式、允许运动范围必须现场明确。
   电机速度是电机轴 rpm，输出轴约为电机轴/101；减速比不能当作安全限位。
   I/F 的真实转速只能由独立测量确认，SMO 估算不是编码器反馈。

## 操作与回收记录
source tools/startup_jlink_v1.gdb 只定义命令，不会下载或启动。
slj_status 输出上述读数；需在可安全读内存的调试条件下使用。
slj_run DIRECTION MOTOR_RPM DEADLINE_MS 是明确的运动命令，会写请求并 continue。
固件先取得调试所有权，再通过原 SM_CmdInCheck 的 INRUN 发出使能请求。
正常母线/故障检查、自举和状态机不变；实际就绪后才提交原生产邮箱 START。
不要直接修改 RegulFlg、RegilFlgISR、PWM 寄存器或伪造 CAN 状态。

CPU Halt 会暂停软件期限，可能留下 PWM 外设继续输出。尚未验证此板的调试暂停硬件行为。
因此运行期间禁止单步/断点；异常优先使用独立功率切断，不把 slj_abort 当硬件急停。
若调试器不支持经验证的无暂停读写，先等待本次有限测试结束并独立禁止功率，再 Halt 导出。
slj_abort 只是排队撤销命令，要 CPU 执行才能生效；期限到也依赖 CPU 和 1ms 中断正常运行。
退出撤销使能，活动启动置 EXTERNAL 故障，PWM 控制链下一次执行时取消输出接管，原状态机继续关断。
故障原因在 exit_reason / startup first_fault / FaultP 中分别读取，不自动清故障或重试。

冻结后可用 GDB：
dump binary memory trace.bin &g_sensorless_trace ((char *)&g_sensorless_trace + sizeof(g_sensorless_trace))
再用 tools/export_startup_trace.py 按其 --help 指定实际 PWM 频率、来源 hardware、导出 CSV/JSON。
先保存 g_sensorless_jlink、g_sensorless_production、g_sensorless_diag 及供电/参数/版本；再显式清除故障或开始新记录。
当前编码器有效位仍为 0，本候选固件没有恢复编码器采集，也没有完成有感校准。

## 验证边界
本轮结果：175 个目标源文件编译通过，11 个共享对象尺寸检查通过，229 个对象可重定位链接通过，BANK0 完整构建通过；CANopen/J-Link 主机集成测试零失败。
主机测试覆盖真实生产邮箱、启动模块和调试门控，平台就绪由测试桩模拟。
目标交叉编译、对象 ABI 尺寸检查及 BANK0 链接只证明软件构建一致性。
未证明采样比例、PWM 极性、JTAG 接线、实板 ISR 耗时、停机机械距离或真实启动成功。

## 2026-09-18 仅定位许可修正
allow_continuous_rotation=0 可应用有效配方，默认 ALIGN；IF/FULL 启动仍被拒绝。
flag=1 的配方仍默认 IF。模式变更不绕过此启动门控。
修正版及同次构建哈希见 diagnostics/releases/20260918_align_permission。
此版本未预置本台电机参数及 200 ms 定位时间，更新后仍须停机应用配方、调参并读回。
