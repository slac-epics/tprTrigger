#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <map>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include <stdint.h>

#include <ellLib.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsGeneralTime.h>
#include <generalTimeSup.h>

#include <dbScan.h>

#include "pcieTpr.h"

static const char *name_s[] = { "tprA", 
                                "tprB",
                                "tprC",
                                "tprD",
                                "tprE",
                                "tprF",
                                "tprG",
                                "tprH" };
static const char *dev_s[] = { "/dev/tpra",   /* 1st card */
                               "/dev/tprb",   /* 2nd card */
                               "/dev/tprc",   /* 3rd card */
                               "/dev/tprd",   /* 4th card */
                               "/dev/tpre",   /* 5th card */
                               "/dev/tprf",   /* 6th card */
                               "/dev/tprg",   /* 7th card */ 
                               "/dev/tprh",    /* 8th card */ 
                               NULL };
static int ev_prefix[] = { 1000, 2000, 3000, 4000, 50000, 6000, 7000, 8000 };
                   
static char dev_c[] = { '0', '1', '2', '3', '4', '5', '6', '7',
                        '8', '8', 'a', 'b', 'c', 'd',
                        '\0' };

typedef struct {
    EVENTPVT             pevent;
    epicsTimeStamp      time;
} ts_tbl_t;

static std::map <int, ts_tbl_t> ts_tbl;


typedef struct {
    char * thread_name;
    char * file_name;
    int    ev_num;
    int    ch_idx;
} trgChParam_t;


static void tprChannelFunc(void *param)
{

    trgChParam_t *p = (trgChParam_t *) param;
    int fd          = open(p->file_name, O_RDWR);
    if(fd < 0) {
        printf("%s thread: could not open %s\n", p->thread_name, p->file_name);
        return;
    }
    
    void *ptr = mmap(0, sizeof(Tpr::TprQueues), PROT_READ, MAP_SHARED, fd, 0);
    if(ptr == MAP_FAILED) {
        printf("%s thread: %s failed to map\n", p->thread_name, p->file_name);
        return;
    }

    volatile Tpr::TprQueues& q = *(Tpr::TprQueues*) ptr;
    char                  *buff = new char[32];
    char                  *ev_name = new char[16];
    int64_t               prev_allrp = -1;
    
    sprintf(ev_name, "%d", p->ev_num);
    ts_tbl[p->ev_num].pevent = eventNameToHandle((const char *) ev_name);

    while(1) {
        read(fd, buff, 32);
        int64_t      allrp = q.allwp[p->ch_idx] -1;
        volatile uint32_t *dp = (&q.allq[q.allrp[p->ch_idx].idx[allrp &(MAX_TPR_ALLQ-1)] &(MAX_TPR_ALLQ-1) ].word[0]);

        if(prev_allrp == allrp) {   // there is no update
            printf("%s thread %s is notified but, no new pattern\n", p->thread_name, p->file_name);
            epicsThreadSleep(1.);
            continue;
        }

        ts_tbl[p->ev_num].time.secPastEpoch = dp[5];
        ts_tbl[p->ev_num].time.nsec = dp[4];

        postEvent(ts_tbl[p->ev_num].pevent);

//        printf("%s thread: (%s, %d, %d): event update %s, timestamp update %d\n", p->thread_name, p->file_name, p->ev_num, p->ch_idx, ev_name,p->ev_num);
        /*
        printf("%s thread: (%s, %d, %d): (wp %16llx, qidx %16llx) (H: %lx, sz %d), %d %d %d %d\n", p->thread_name, p->file_name, p->ev_num, p->ch_idx,
                                                             allrp, q.allrp[p->ch_idx].idx[allrp &(MAX_TPR_ALLQ-1)] &(MAX_TPR_ALLQ-1) , dp[0], dp[1], dp[2], dp[3], dp[4], dp[5]);   */     


        prev_allrp = allrp;
    }

    munmap(ptr, sizeof(Tpr::TprQueues));
    close(fd);
}


static void * createPcieThread(char * thread_name, char * file_name, int ev_num, int ch_idx)
{
    trgChParam_t * pCh  = new trgChParam_t;
    pCh->thread_name    = epicsStrDup(thread_name);
    pCh->file_name      = epicsStrDup(file_name);
    pCh->ev_num         = ev_num;
    pCh->ch_idx         = ch_idx;

    epicsThreadCreate(thread_name, epicsThreadPriorityHigh,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) tprChannelFunc, (void *) pCh);
    
    return (void *) NULL;
}



static int pcieTprTimeGet_gtWrapper(epicsTimeStamp *time, int eventCode)
{
//    printf("pcieTprTimeGet_gtWrapper: ev_code = %d\n", eventCode);
    *time  = ts_tbl[eventCode].time;

    return 0;
}

static int pcieTprTimeGetSystem_gtWrapper(epicsTimeStamp *time, int eventCode)
{
    if(epicsTimeGetCurrent(time)) return -1;

    return 0;
}


void *  pcieTprInit(char *dev_prefix)
{
    int dev_idx;
    int ch_idx;
    int ev_num;
    char file_name[80];
    char thrd_name[80];

    for(dev_idx =0; dev_s[dev_idx]; dev_idx++) {
        if(!strcmp(dev_prefix, dev_s[dev_idx])) break;
    }

    if(!dev_s[dev_idx]) return (void *) NULL;

    for(ch_idx = 0; dev_c[ch_idx]; ch_idx++) {
        ev_num = ev_prefix[dev_idx] + ch_idx;
        sprintf(file_name, "%s%c", dev_s[dev_idx], dev_c[ch_idx]);
        sprintf(thrd_name, "%s%c", name_s[dev_idx], dev_c[ch_idx]);


        createPcieThread(thrd_name, file_name, ev_num, ch_idx);
    }



    return (void*) NULL;
}


void * pcieTprGPWrapper(void)
{
    printf("Found PCIeTPR: execute GPWrapperInit()\n");

    generalTimeRegisterEventProvider("pcieTprTimeGet",       1000, (TIMEEVENTFUN) pcieTprTimeGet_gtWrapper);
    generalTimeRegisterEventProvider("pcieTprTimeGetSystem", 2000, (TIMEEVENTFUN) pcieTprTimeGetSystem_gtWrapper);

    return (void *) NULL;
}


extern "C" {

}  /* extern C */

