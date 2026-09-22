# CANopen 无感 FOC 移植版说明

日期：2026-09-18。项目：Servo_HPM_Project_SLFOC/Servo_HPM_Project。固件标识：0x20260918；参数版本 7；原内存邮箱 ABI 7；新增 CANopen 应用协议版本 1。

## 交付范围

保留目标工程 CANopen、CiA402、板级初始化、SDFM 采样、功率级保护、驱动系数及未替换的目标 .o。迁入 SourceFoc、I/F → SMO/PLL、分阶段调试、首次故障锁存、环形波形及原有回归测试。

这是专用无感测试固件。PWM 路径不执行旧位置/速度环和编码器事务；原编码器启动交换与周期故障轮询停用。编码器独立参考标记无效。原标准速度反馈可能为估算值，不是独立测速。HFI、自动辨识、编码器回退本轮不开放；遗留相关测试通过不表示这些功能已启用。

原始备份：diagnostics/before_sensorless_canopen_20260918_140304/，含 CtrlLoop、工程文件及原 BANK0 Exe。未覆盖旧 Demo。

## 源码和工程接入

- 新增 CtrlLoop/33_Sensorless，另有 SensorlessCanopen.c（应用协议/命令队列/看门狗）、SensorlessCanopenOd.c（对象字典和平台条件）。
- 从旧工程迁入 ToqLoop、Clark/Park/Ipark、Id/Iq PI 与参考、SVPWM 九个 C 文件；工程排除同名九个 .o。
- CtrLoop 接入初始化、复位、PreFoc/电流环/PostFoc；SV_Timer 接入 1ms 命令服务。
- 保留目标 DrvCoeff 的 160 字节布局；没有把旧工程 164 字节布局或旧 .o 覆盖进来。共享对象大小检查通过不等于所有字段偏移及物理量纲均已证明。
- PWM 中断每次最多处理 4 个 FIFO0 报文；剩余留待后续周期。g_sl_can_rx0_deferred 为累计延后报文计数，g_sl_can_rx0_highwater 为观测积压峰值，不等于丢帧计数。
- g_startup_pwm_isr_last_cycles/max_cycles/samples 记录中断 C 主体耗时，未包含全部汇编入口/出口。
- CAN 写传统驱动参数时，Servo ON 状态拒绝写入；6040 等标准 CiA402 控制入口继续保留。
- 新增 ER.03A 参数故障、ER.03B 无感跟踪/控制故障；细分原因保留在诊断位图。

## CANopen 对象

现用标准 CANopen 8 字节帧。CanBaud=0 选择 500 kbit/s，否则 1 Mbit/s；节点号取实际 RamCommu->CanID。本轮不修改节点号/波特率的原有存储方式。

| 索引 | 内容 | 属性 |
|---|---|---|
| 0x2F00 | 版本、命令、请求/确认序号、心跳与结果 | SDO |
| 0x2F01 | 电机/电流环配方暂存区 | SDO，停机写 |
| 0x2F02 | 启动/SMO 调参暂存区 | SDO，停机写 |
| 0x2F03 | 实际生效配方 | SDO，只读 |
| 0x2F04 | 实际生效启动配置 | SDO，只读 |
| 0x2F05 | 实际生效 SMO 参数 | SDO，只读 |
| 0x2F06 | 状态、首次故障、估算速度、Iq、记录信息 | SDO/可映射 TPDO，只读 |
| 0x2F07:0 | 完整冻结波形，15912 字节 | SDO 分段上传，只读 |

全部记录 sub0 为 U8 子项数；非零子项在线上均为 4 字节、小端。浮点字段为 IEEE754 float32 位模式，int16 字段在线上使用符号扩展后的 int32。完整字段索引/类型见 tools/sensorless_canopen_schema.json；这是显式字段协议，不是内存邮箱或 C 结构体的直接映射。新增对象后字典为 640 项，容量 700 项，另留终止项。

0x2F00 子项：

| sub | 名称 | 操作 |
|---|---|---|
| 1 | protocol | 读 1；会话先写 1，旧协议拒绝 |
| 2 | request_seq | 最后写非零新序号，提交一次命令 |
| 3 | command | 0/NONE、2/APPLY_TUNING、3/START、5/STOP、6/CLEAR、12/APPLY_RECIPE |
| 4 | arm_key | START 写 0x534C，每次命令服务后消耗 |
| 5 | direction | int32，+1 或 -1 |
| 6 | target_speed_rpm | 非负机械转速，方向单独指定 |
| 7 | acknowledge_seq | 请求处理完才确认 |
| 8 | result | 该次命令结果，不等于测试已成功 |
| 9 | heartbeat_counter | 每次写不同值续期，建议 200ms 一次 |
| 10 | watchdog_ms | 只读，1000ms |
| 11 | abort_reason | 0 无、1 主机心跳超时/未建立、2 平台条件撤销 |
| 12 | pending | 非零时拒绝改写待执行命令及参数，心跳除外 |
| 13–16 | 固件版本、参数版本、生效参数序号、毫秒计数 | 只读 |

result：0 idle；1 accepted；2 busy；3 bad arm key；4 servo not ready；5 invalid profile；6 invalid tuning；7 invalid command。其它值沿用 SensorlessProduction.h，当前 CAN 入口未开放产线序列/自动辨识命令。

最近一次序号重复提交拒绝，不自动重试启动。单主机拥有此接口；不要让 J-Link 与 CAN 同时提交邮箱命令。主机重连先读 request/ack/pending 再决定操作。

## 控制与停机边界

START 要求：已应用有效配方、主机心跳有效、NMT Operational、CiA402 Operation Enabled、控制字仍允许运行且未 Halt、平台 RegulFlg/RegilFlgISR 都有效、无平台故障。CAN START 不自动 Servo ON，不直接操作抱闸。

正常测试 STOP 经无感停止状态机处理。撤销使能、Quick Stop、Halt、NMT 退出运行、主机心跳超时，会在 PWM 路径把活动启动置为外部故障，撤销无感输出并走原一级故障功率保护。这一版的外部取消路径不是 CiA402 标准减速轨迹跟随；实际抱闸/停机机械响应待上机确认。断线重连不自动重启。

CLEAR 仅清无感诊断，要求 Servo OFF；平台故障复位仍使用原 CiA402/驱动器故障复位流程。完成限时 I/F 不等于 Servo OFF。工厂测试 SysFtest 必须关闭，不能绕开正常状态机运行该测试。

## 参数提交与读回

1. 停机，写完整 0x2F01 配方，提交 cmd=12；确认 ack 和 result=1。
2. 配方应用会重建启动默认配置并默认选择限时 I/F。随后写 0x2F02 调试参数，mask 选择需要应用的组，提交 cmd=2。
3. 同一 mask 组内填写完整字段；不得用遗漏字段代表“保持原值”。暂存区未写字段仍保留暂存值。
4. 读 0x2F03/04/05 验证真实生效值。整组读前后读取 0x2F00:15，相同且为偶数才接受；运行时只读上次停机快照。
5. 基值是实际硬件校准后的相电流峰值/电压标幺基值。旧 200V Demo 的数值不能直接视为新板参数。模板 null 项必须填写；本轮未填入未知电机参数。

## 主机工具

tools/sensorless_canopen.py 使用标准 SDO；硬件访问需 python-can 及实际适配器驱动。接口名、通道、波特率、节点号全部由使用者指定，没有默认连接或自动使能。已验证编解码模拟测试，尚未验证具体适配器。

以下参数为占位符，替换为现场真实值；在项目根目录运行：

```text
python tools/sensorless_canopen.py --interface <适配器接口> --channel <通道> --bitrate <500000或1000000> --node <节点> status
python tools/sensorless_canopen.py --interface <接口> --channel <通道> --bitrate <波特率> --node <节点> apply motor.json --tuning debug.json
python tools/sensorless_canopen.py --interface <接口> --channel <通道> --bitrate <波特率> --node <节点> start --rpm <已核准转速> --seconds <限定时长>
python tools/sensorless_canopen.py --interface <接口> --channel <通道> --bitrate <波特率> --node <节点> dump startup_trace.bin
```

motor.json 可采用 source_foc_recipe.template.json 的 source_recipe 部分；debug.json 可采用其中 startup_debug 部分。start 前通过原主机正常建立 NMT/CiA402 使能及抱闸条件。start 子命令维持应用心跳，退出时尝试 STOP；通信本身故障时不能保证 STOP 已送达，由固件心跳超时处理。

dump 同时输出 .session.json，包含节点、固件版本、生效参数及记录信息。先导出再 CLEAR 或下一次 START。分段读取遇到记录变化会拒绝；状态 0x2F06 各字段是单次实时读，不宣称整组原子快照。精确同拍分析使用冻结 trace。

```text
python tools/export_startup_trace.py startup_trace.bin --sample-hz <实际控制频率> --provenance hardware
```

控制频率取生效 startup.sample_time_s 的倒数并与硬件 PWM 配置核实；不能因为示例测试使用 16000Hz 就认定每块板相同。

## 验证与上机顺序

已执行目标头文件编译、共享对象大小检查、可重定位链接、BANK0 全量构建、已有算法测试、真实模块串联测试、CAN 命令/超时/冻结记录测试及 Python SDO 模拟测试。结果日志和最终产物哈希在 diagnostics/source_foc_check。存在原工程 IQmath 严格别名、旧式函数声明等警告；“构建通过”不表示全工程零警告。

最终 MAP：波形缓存 15912 字节；两个 CAN 适配模块的 RW+ZI 合计 2803 字节。堆预留 16KB（0x002254C0–0x002294BF），栈预留 16KB（0x0023C000–0x0023FFFF），链接无重叠。尚无 ISR 最坏耗时、实际堆/栈峰值、总线负载或功率级实测值。BANK1 工程选择已同步排除重复 .o，但本次交付验证仅 BANK0。

离线回归：5 套原算法测试、5 套控制/配方/启动/CAN 集成测试通过；Python SDO 测试 5 项通过。CAN 集成测试还覆盖 NaN 参数拒绝且生效参数不变，以及生效参数序号在内容不变时保持稳定。日志为 migration_legacy.log、migration_host.log、canopen_transport.log、canopen_host_python.log。

上机次序：
1. 禁止功率输出，核对同批 ELF/BIN/MAP 和板卡；读取协议、固件、参数版本。
2. 验证 SDO 下参/拒绝条件/读回、节点状态、故障复位，以及总线负载下 ISR 周期。
3. 核实电流零偏、量纲、相序、PWM 极性、母线与限流、抱闸及停机。
4. 定位 → 限时 I/F → 角度交接/固定 Iq → 速度环接管。首轮默认 I/F。
5. 断线、快速停机、重复启动、数据导出后重测。

本轮没有连接 CAN、下载固件或运行电机；不得把模拟 trace 或模拟测试标记为硬件通过。
