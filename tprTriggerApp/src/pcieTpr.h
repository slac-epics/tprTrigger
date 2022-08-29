#ifndef  PCIE_TPR_ASYN_DRIVER_HH
#define  PCIE_TPR_ASYN_DRIVER_HH

#include <stdint.h>
#include "tprsh.hh"


extern "C" {

void * pcieTprInit(char *dev_prefix);
void * pcieTprGPWrapper(void);

}

#endif   /* PCIE_TPR_ASYN_DRIVER_HH */
