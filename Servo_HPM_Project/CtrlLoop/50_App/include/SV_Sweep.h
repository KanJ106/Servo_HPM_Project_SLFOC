#ifndef __SV_SWEEP_H
#define __SV_SWEEP_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "IQmathLib.h" 

// 定义数据缓冲区
#define SWEEP_POINTS_MAX 1000
static float freq_buffer[SWEEP_POINTS_MAX];
static float gain_buffer[SWEEP_POINTS_MAX];
static float phase_buffer[SWEEP_POINTS_MAX];

// 扫频状态机
typedef enum {
    SWEEP_STATE_IDLE = 0,
    SWEEP_STATE_INIT,
    SWEEP_STATE_RUNNING,
    SWEEP_STATE_COMPLETE,
    SWEEP_STATE_ERROR
} SweepState_t;

// 扫频配置参数
typedef struct {
    float f_start_hz;      // 起始频率 (Hz)
    float f_end_hz;        // 终止频率 (Hz)
    float sweep_time_s;    // 总扫频时间 (s)
    float amplitude;       // 激励幅值 (额定转矩百分比)
    uint16_t points;       // 计划采集的点数
} SweepConfig_t;

// 扫频运行时数据
typedef struct {
    float current_freq_hz;     // 当前频率
    float phase_accumulator;   // 相位累加器
    float time_accumulator;    // 时间累加器
    uint16_t current_point;    // 当前数据点索引
    
    // 正交解调相关
    float i_integral;
    float q_integral;
    float integration_time;    // 在当前频率点的积分时间
    float point_time_elapsed;  // 当前频率点已积分时间
    bool point_complete;       // 当前频率点采集完成标志
    
    // 频率跟踪
    float prev_freq_hz;
} SweepRuntime_t;

// 扫频结果数据
typedef struct {
    float *frequencies;    // 频率数组
    float *gains;          // 增益数组 (dB)
    float *phases;         // 相位数组 (度)
    uint16_t count;        // 有效数据点数
} SweepResult_t;

// 全局扫频控制器
typedef struct {
    SweepState_t state;
    SweepConfig_t config;
    SweepRuntime_t runtime;
    SweepResult_t results;

    int cmd;//扫频开始命令
    int sel;//扫频类型选择
    _iq signal_out0;//正弦波扫频信号输出0
    _iq signal_out1;//正弦波扫频信号输出1
    _iq signal_out2;//正弦波扫频信号输出2
    int32 signal_out3;//正弦波扫频信号输出3
    int cnt1;//计数1

    Uint16	DrvModePre;//系统原来的控制模式
    Uint16	DrvModeSet;//系统重新设置的控制模式
    
    // 保护标志
    bool data_ready;
    
    // 回调函数指针
    void (*on_complete)(void);
} SweepController_t;

// 函数声明
void SWEEP_Init(SweepController_t *ctrl, float *freq_buf, float *gain_buf, float *phase_buf, uint16_t buf_size);
bool SWEEP_Start(SweepController_t *ctrl, SweepConfig_t *config);
void SWEEP_Stop(SweepController_t *ctrl);
void SWEEP_BackgroundTask(SweepController_t *ctrl);  // 1kHz后台任务
void SWEEP_CurrentLoopISR(SweepController_t *ctrl);  // 16kHz电流环中断

// 工具函数
static inline void SWEEP_ResetIntegrators(SweepRuntime_t *runtime);
static inline float SWEEP_CalculateCurrentFrequency(const SweepConfig_t *config, float time);

void process_communication_command(void);

extern SweepController_t sweep_controller;  // 在main.c中定义
extern SweepConfig_t sweep_config1;

#endif