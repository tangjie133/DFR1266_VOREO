#ifndef AUDIO_INITIALIZATION_H
#define AUDIO_INITIALIZATION_H

#include "esp_codec_dev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Audio codec for recording
 * @param record_dev Pointer to the codec device handle that will be initialized
 * @return 0 on success, non-zero on error
 */
int initAudio(esp_codec_dev_handle_t *record_dev);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_INITIALIZATION_H

