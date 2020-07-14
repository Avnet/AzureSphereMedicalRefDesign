#ifndef HR4_H_
#define HR4_H_

#include "eventloop_timer_utilities.h"

void HR4TimerEventHandler(EventLoopTimer* timer);

// State variable to keep track of what we're doing
extern enum HR4_State currentState;

extern float median_hr, median_spo2;
extern uint8_t sampleCount;
extern bool autoRestartHRTest;

enum HR4_State
{
    IDLE,
    HR4_CALIBRATE,
    REPOSITION_FINGER,
    DATA_COLLECTION,
    SEND_RESULTS,
    DETECT_FINGER,
    DISPLAY_RESULTS
};

#endif // HR4_H_