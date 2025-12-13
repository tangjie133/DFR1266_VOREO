#ifndef USER_CODE_H
#define USER_CODE_H

#define SAMPLE_RATE     16000
#define FRAME_SIZE 1024
#define CHANNELS 4

#define HOP_SIZE        512       // hop size (FRAME_SIZE/2 => 50% overlap)
#define MICRO_SIDE_M    0.05f     // square side length (50mm)
#define MICRO_RADIUS    (MICRO_SIDE_M / 2.0f * sqrtf(2.0f) / 2.0f) // not used directly
#define SPEED_OF_SOUND  343.0f

#define VAD_ENERGY_TH   1e-6f     // frame energy threshold to enable processing

/* Kalman parameters for angle smoothing */
#define KALMAN_Q_ANGLE  0.001f
#define KALMAN_Q_RATE   0.01f
#define KALMAN_R_MEAS   2.0f  

extern float angle;

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 初始化麦克风（音频编解码器）
 */
void initMIC(void);

/**
 * @brief 初始化语音识别系统
 */
void initSpeechRecog(void);

/**
 * @brief 初始化音频DOA（到达方向）系统
 */
void initAudioDoa(void);

/**
 * @brief 自动检测TDM槽位到物理麦克风的映射关系
 */
void autodetectSlotMapping(void);


#ifdef __cplusplus
}
#endif

#endif // USER_CODE_H
