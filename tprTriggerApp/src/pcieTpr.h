#ifndef  PCIE_TPR_ASYN_DRIVER_HH
#define  PCIE_TPR_ASYN_DRIVER_HH

#include <stdint.h>
#include "tprsh.hh"

#include "timingFifoApi.h"

#define  RESERVED_CH    11
#define  EV360HZ_CH     RESERVED_CH
#define  EV360HZ_EV      1
#define  MAX_SOFT_EV     8

extern "C" {

void   pcieTprSetSoftEv(int idx, int ev_num, bool ev_enable);
void   pcieTprSetChEv(int ev_num, bool ev_enable);
void   pcieTprInitSoftEv(void);
int    pcieTpr_evPrefix(char *dev_prefix);
int    pcieTprEvCallbackInit(void);
void * pcieTprInit(char *dev_prefix);
void * pcieTprReport(int level);
void * pcieTprGPWrapper(void);

TimingPulseId timingGetLastFiducial();
int timingGetCurrTimeStamp(epicsTimeStamp *ptine);
int timingGetEventTimeStamp(epicsTimeStamp *ptime, int eventCode);
int timingFifoRead(unsigned int    eventCode,
                   int             incr,
                   uint64_t        *index,
                   EventTimingData *pTimingDataDest);
int RegisterTimingEventCallback(TimingEventCallback callback, void *pUserPvt);

}

#endif   /* PCIE_TPR_ASYN_DRIVER_HH */
