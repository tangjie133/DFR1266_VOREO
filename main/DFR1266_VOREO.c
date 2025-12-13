#include <stdio.h>
#include "UserConfig.h"
#include "UserCode.h"
#include "PeripheralCode.h"
#include "rgb.h"

void app_main(void)
{
    initMIC();
    peripheralInit();
    setRGBColor(RGB_BLACK);
    //autodetectSlotMapping();
    initAudioDoa();
    //initSpeechRecog();

}

