#include "SV_Sweep.h"
//#include "motor_control.h"
#include <stdlib.h>
#include "Drive.h"
#include "SV_PanelCtl.h"//

// 定义时间常量
#define CURRENT_LOOP_DT      (0.0000625f)  // 1/16000 = 62.5us
#define BACKGROUND_LOOP_DT   (0.001f)      // 1ms


// 全局扫频控制器
SweepController_t sweep_controller;
SweepConfig_t sweep_config1;

// 初始化扫频控制器
void SWEEP_Init(SweepController_t *ctrl, float *freq_buf, float *gain_buf, float *phase_buf, uint16_t buf_size)
{
    ctrl->state = SWEEP_STATE_IDLE;
    ctrl->data_ready = false;
    
    // 初始化结果缓冲区
    ctrl->results.frequencies = freq_buf;
    ctrl->results.gains = gain_buf;
    ctrl->results.phases = phase_buf;
    ctrl->results.count = 0;
    
    // 清空运行时数据
    memset(&ctrl->runtime, 0, sizeof(SweepRuntime_t));
    memset(&ctrl->config, 0, sizeof(SweepConfig_t));
    
    ctrl->on_complete = NULL;
}

// 启动扫频
bool SWEEP_Start(SweepController_t *ctrl, SweepConfig_t *config)
{
    if (ctrl->state != SWEEP_STATE_IDLE && ctrl->state != SWEEP_STATE_COMPLETE) {
        return false;  // 忙状态，不能启动
    }
    
    if (config->f_start_hz >= config->f_end_hz || config->sweep_time_s <= 0) {
        return false;  // 参数错误
    }

    if(ctrl->cnt1 < 1000){
        return false;  // 忙状态，不能启动
    }
    
    // 保存配置
    ctrl->config = *config;
    
    // 重置运行时数据
    memset(&ctrl->runtime, 0, sizeof(SweepRuntime_t));
    ctrl->runtime.current_freq_hz = config->f_start_hz;
    ctrl->runtime.prev_freq_hz = config->f_start_hz;
    ctrl->runtime.current_point = 0;
    ctrl->runtime.point_complete = false;
    ctrl->results.count = 0;
    ctrl->data_ready = false;
    
    // 计算每个频率点的积分时间
    // 确保在每个频率点至少积分3-5个周期
    float min_freq = (config->f_start_hz < 10.0f) ? 10.0f : config->f_start_hz;
    ctrl->runtime.integration_time = 5.0f / min_freq;  // 积分5个周期
    
    // 进入初始化状态
    ctrl->state = SWEEP_STATE_INIT;
    
    return true;
}

// 停止扫频
void SWEEP_Stop(SweepController_t *ctrl)
{
    if (ctrl->state == SWEEP_STATE_RUNNING || ctrl->state == SWEEP_STATE_INIT) {
        ctrl->state = SWEEP_STATE_IDLE;
    }
}

// 1kHz后台任务
void SWEEP_BackgroundTask(SweepController_t *ctrl)
{
    switch (ctrl->state) {
        case SWEEP_STATE_INIT:
            // 执行初始化检查，确保系统稳定
            // 可以添加延时确保电机稳定
            static uint32_t init_counter = 0;
            if (++init_counter >= 100) { // 等待100ms
                init_counter = 0;
                ctrl->state = SWEEP_STATE_RUNNING;
            }

            /*
            if(sweep_controller.sel==2){
                sweep_controller.DrvModePre = RamBasePara->DrvMode;
                RamBasePara->DrvMode = 0; 
                Servo2Panel.Bit.FFt = 1;
            }
            */

            //ctrl->state = SWEEP_STATE_RUNNING;
            break;
            
        case SWEEP_STATE_RUNNING:
            // 检查是否完成扫频
            if (ctrl->runtime.time_accumulator >= ctrl->config.sweep_time_s) {

                sweep_controller.cnt1 = 0;
                sweep_controller.signal_out0 = 0;
                sweep_controller.signal_out1 = 0;
                sweep_controller.signal_out2 = 0;
                sweep_controller.signal_out3 = 0;

                ScopeCtrl->cmd = 0;
                ScopeCtrl->MortorSon = 0;

                ctrl->state = SWEEP_STATE_COMPLETE;
                ctrl->data_ready = true;

                // 调用完成回调
                if (ctrl->on_complete != NULL) {
                    ctrl->on_complete();
                }
            }
            break;
            
        case SWEEP_STATE_COMPLETE:
            // 保持完成状态，等待上位机读取数据

            break;
            
        case SWEEP_STATE_ERROR:
            // 错误处理，可以添加错误恢复机制
            break;
            
        default:
            break;
    }
    
    // 更新显示或状态上报（可选）
    static uint32_t status_counter = 0;
    if (++status_counter >= 100) { // 每100ms上报一次状态
        status_counter = 0;
        // COMM_SendSweepStatus(ctrl->state, ctrl->runtime.current_freq_hz);
    }
}

// 16kHz电流环中断服务程序
void SWEEP_CurrentLoopISR(SweepController_t *ctrl)
{
    if (ctrl->state != SWEEP_STATE_RUNNING) {
        ctrl->signal_out0 = 0;
        ctrl->signal_out1 = 0;
        ctrl->signal_out2 = 0;
        return;
    }
    
    SweepConfig_t *cfg = &ctrl->config;
    SweepRuntime_t *rt = &ctrl->runtime;
    SweepResult_t *res = &ctrl->results;
    
    // 1. 更新时间和频率
    rt->time_accumulator += CURRENT_LOOP_DT;
    rt->current_freq_hz = SWEEP_CalculateCurrentFrequency(cfg, rt->time_accumulator);
    
    // 2. 生成激励信号并注入
    float injection_signal = 0.0f;
    if (rt->current_freq_hz <= cfg->f_end_hz) {
        // 更新相位 (确保连续性)
        rt->phase_accumulator += 2.0f * M_PI * rt->current_freq_hz * CURRENT_LOOP_DT;
        if (rt->phase_accumulator > 2.0f * M_PI) {
            rt->phase_accumulator -= 2.0f * M_PI;
        }
        
        // 生成正弦信号
        injection_signal = cfg->amplitude * sinf(rt->phase_accumulator);
        
        // 注入到转矩指令---在其他模块中加入
       // MOTOR_InjectTorqueSignal(injection_signal);
        if(ctrl->sel==1) {
            ctrl->signal_out0 = 0;
            ctrl->signal_out1 =  _IQmpy(DrvCoeff.MotIe,_IQ(injection_signal));
            ctrl->signal_out2 = 0;
        }
        else if(ctrl->sel==2) {
            ctrl->signal_out0 = 0;
            ctrl->signal_out1 = 0;
            ctrl->signal_out2 = _IQmpyI32(DrvCoeff.SpdBasePu,(int16)injection_signal);//_IQmpy(DrvCoeff.MotIe,_IQ(injection_signal));
        }
        else {
            ctrl->signal_out0 = _IQmpy(DrvCoeff.MotIe,_IQ(injection_signal));
            ctrl->signal_out1 = 0;
            ctrl->signal_out2 = 0;
        }

        ctrl->signal_out3 = injection_signal*100;
    }

    /*
    
    // 3. 采集响应并进行正交解调
    if (rt->current_freq_hz > 0 && rt->current_freq_hz <= cfg->f_end_hz) {
        // 获取速度反馈
        float velocity_fb = MOTOR_GetVelocityFeedback();
        
        // 生成正交参考信号
        float ref_sin = sinf(rt->phase_accumulator);
        float ref_cos = cosf(rt->phase_accumulator);
        
        // 相乘并积分
        rt->i_integral += velocity_fb * ref_sin * CURRENT_LOOP_DT;
        rt->q_integral += velocity_fb * ref_cos * CURRENT_LOOP_DT;
        
        // 更新当前频率点积分时间
        rt->point_time_elapsed += CURRENT_LOOP_DT;
        
        // 检查频率变化或积分完成
        bool freq_changed = fabsf(rt->current_freq_hz - rt->prev_freq_hz) > 0.5f; // 频率变化阈值
        
        if ((rt->point_time_elapsed >= rt->integration_time || freq_changed) && !rt->point_complete) {
            // 确保有足够的积分时间
            if (rt->point_time_elapsed >= (1.0f / rt->current_freq_hz)) { // 至少积分1个周期
                // 计算该频率点的增益和相位
                float magnitude = sqrtf(rt->i_integral * rt->i_integral + rt->q_integral * rt->q_integral);
                float gain_linear = magnitude / (cfg->amplitude + 1e-9f);  // 避免除零
                float gain_db = 20.0f * log10f(gain_linear + 1e-9f);
                float phase_rad = atan2f(rt->q_integral, rt->i_integral);
                float phase_deg = phase_rad * 180.0f / M_PI;
                
                // 存储结果
                if (rt->current_point < cfg->points) {
                    res->frequencies[rt->current_point] = rt->prev_freq_hz; // 使用上一个稳定频率
                    res->gains[rt->current_point] = gain_db;
                    res->phases[rt->current_point] = phase_deg;
                    res->count = ++rt->current_point;
                }
            }
            
            // 重置积分器和标志
            SWEEP_ResetIntegrators(rt);
            rt->point_time_elapsed = 0.0f;
            rt->point_complete = true;
        }
        
        // 如果频率变化超过阈值，重置点完成标志以便开始新频率点的积分
        if (freq_changed) {
            rt->point_complete = false;
            rt->prev_freq_hz = rt->current_freq_hz;
        }
    }
    */
}

// 工具函数：计算当前频率 (线性扫频)
static inline float SWEEP_CalculateCurrentFrequency(const SweepConfig_t *config, float time)
{
    if (time >= config->sweep_time_s) {
        return config->f_end_hz;
    }
    
    float progress = time / config->sweep_time_s;
    return config->f_start_hz + (config->f_end_hz - config->f_start_hz) * progress;
}

// 工具函数：重置积分器
static inline void SWEEP_ResetIntegrators(SweepRuntime_t *runtime)
{
    runtime->i_integral = 0.0f;
    runtime->q_integral = 0.0f;
}



// 处理通信命令
void process_communication_command(void)
{
    sweep_controller.cmd = ScopeCtrl->cmd;
    sweep_controller.sel = ScopeCtrl->sel;

    if(ScopeCtrl->cmd==0){
        return;
    }

    sweep_config1.f_start_hz = ScopeCtrl->f_start_hz;
    sweep_config1.f_end_hz = ScopeCtrl->f_end_hz;
    sweep_config1.amplitude = (float)ScopeCtrl->amplitude/100;
    sweep_config1.sweep_time_s = (float)ScopeCtrl->sweep_time_s/10;

    /*
    if(sweep_controller.sel==2){
        sweep_config1.amplitude = RamPC->amplitude2;//(float)RamPC->amplitude2/300;
    }else{
        sweep_config1.amplitude = (float)RamPC->amplitude/100;
    }
    */


    switch(sweep_controller.cmd){
        case 1:
            SWEEP_Start(&sweep_controller, &sweep_config1);

            sweep_controller.cnt1++;

            if(sweep_controller.sel==1){
                IqRef.IqRefGet = 2;
                IdRef.IdRefGet = 2;
            }else if(sweep_controller.sel==2){
                IqRef.IqRefGet = 0;
                IdRef.IdRefGet = 0;
            }else{
                IqRef.IqRefGet = 2;
                IdRef.IdRefGet = 2;
            }

            break;
            
        case 2:
            SWEEP_Stop(&sweep_controller);

            sweep_controller.cnt1 = 0;
            sweep_controller.signal_out0 = 0;
            sweep_controller.signal_out1 = 0;
            sweep_controller.signal_out2 = 0;
            sweep_controller.signal_out3 = 0;
            break;
            
        case 3:
            break;
            
        default:
            break;
    }
}