#ifndef RTOS_STATS_H_
#define RTOS_STATS_H_

#ifdef __cplusplus
extern "C" {
#endif

void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

#ifdef __cplusplus
}
#endif

#endif /* RTOS_STATS_H_ */
