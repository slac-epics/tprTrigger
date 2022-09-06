#ifndef  PCIE_TPR_ASYN_DRIVER_HH
#define  PCIE_TPR_ASYN_DRIVER_HH

#include <stdint.h>
#include "tprsh.hh"

#include "timingFifoApi.h"

extern "C" {

void   pcieTprSetSoftEv(int idx, int ev_num, bool ev_enable);
void   pcieTprSetChEv(int ev_num, bool ev_enable);
void   pcieTprInitSoftEv(void);
int    pcieTpr_evPrefix(char *dev_prefix);
void * pcieTprInit(char *dev_prefix);
void * pcieTprGPWrapper(void);

TimingPulseId timingGetLastFiducial();
int timingGetCurrTimeStamp(epicsTimeStamp *ptine);
int timingGetEventTimeStamp(epicsTimeStamp *ptime, int eventCode);


}

#endif   /* PCIE_TPR_ASYN_DRIVER_HH */
