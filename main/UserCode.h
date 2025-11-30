#ifndef USER_CODE_H
#define USER_CODE_H


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Feed audio data task
 * 
 * This task reads audio data from the codec device and feeds it to the AFE (Audio Front-End)
 * for processing. It runs continuously to maintain audio streaming.
 * 
 * @param arg Pointer to esp_afe_sr_data_t structure
 */
void feed_Task(void *arg);

/**
 * @brief Detect wake word and speech recognition task
 * 
 * This task fetches processed audio data from AFE and detects wake words.
 * When a wake word is detected, it triggers speech recognition processing.
 * 
 * @param arg Pointer to esp_afe_sr_data_t structure
 */
void detect_Task(void *arg);

//初始化麦克风
void initMIC(void);

//初始化语音识别
void initSpeechRecog(void);

#ifdef __cplusplus
}
#endif

#endif // USER_CODE_H
