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
#include "tprTriggerAsynDriver.h"


#if 0
static uint64_t __rdtsc(void){
    uint32_t lo, hi;
    __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
    return ((uint64_t)hi << 32) | lo;
}
#endif

int DEBUG_PCIE_TPR  = 1;

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
                        '8', '9', 'a', 'b', 'c', 'd',
                        '\0' };

typedef struct {
    unsigned      mask:  16;  // [15..0]     channel mask
    unsigned      tag :   4;  // [19..16]    0: event, 1: bsa ctrl, 2: bsa event
    unsigned      dm0 :   2;  // [21..20]    bit gap
    unsigned      mode:   1;  // [22]        1: LCLS1, 0:LCLS2
    unsigned      dm1 :   9;  // [31..23]    bit gap
    unsigned      size:  32;
    uint64_t      pid64: 64;
    uint64_t      ts64:  64; 
    uint32_t      rates: 32;
    uint32_t      tslot: 32;
} time_st_t;



typedef union {
    uint32_t     u32[2];
    uint64_t        u64;
    epicsTimeStamp time;
} time_cmb_t;


typedef struct {
    EVENTPVT             pevent;
    epicsMutex           *plock;
    epicsTimeStamp       time;
    uint64_t             pid64;
    uint64_t             rawpid;

    //////
    volatile Tpr::TprQueues *pQ;
    bool     ev_enable;
    int      ev_num;
    int      ch_idx;
    uint64_t index;
} ts_tbl_t;


typedef struct {
    int      ev_num;
    bool     ev_enable;
} soft_ev_list_t;


typedef struct {
    ELLNODE              node;
    TimingEventCallback  callback;
    void                *pUserPvt;
} TimingEventCallback_t;

typedef struct {
    epicsMutex          *plock;
    ELLLIST             list;
} evCallback_t;

static evCallback_t  *evCallback = NULL;

static std::map <int, ts_tbl_t> ts_tbl;
typedef std::map <int, ts_tbl_t>::iterator ts_tbl_iter;
static soft_ev_list_t soft_ev_list[MAX_SOFT_EV];
static bool have_master = false;

typedef struct {
    char * thread_name;
    char * file_name;
    int    ev_num;
    int    ch_idx;
} trgChParam_t;


static bool isNewTs(uint64_t ts)
{
    static uint64_t ts_last = 0LL;

    if(ts > ts_last) {
        ts_last = ts;
        return true;
    }

    return false;
    
}

static bool check_ev_mask(uint32_t ev, volatile uint32_t *mask)
{
    return mask[ev/32] & (0x00000001 << (ev%32));
}


static void prepLowTsTbl(int num)
{
    char ev_name[16];

    if(ts_tbl.find(num) == ts_tbl.end()) {
        sprintf(ev_name, "%d", num);
        ts_tbl[num].pevent = eventNameToHandle((const char *) ev_name);
        ts_tbl[num].plock  = new epicsMutex();
        ts_tbl[num].pQ     = NULL;   // invalid for low event number
        ts_tbl[num].ev_num = num;
        ts_tbl[num].ch_idx = -1;     // invalid for low event number
        ts_tbl[num].index  = -1;     // invalid for low event number
        ts_tbl[num].ev_enable = false;
    }
}

static void tprChannelFunc(void *param)
{

    trgChParam_t *p = (trgChParam_t *) param;
    int fd          = open(p->file_name, O_RDWR);
    bool     master = false;

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


    if(!have_master && p->ch_idx == RESERVED_CH) {
        master      = true;
        have_master = true;
   }

    prepLowTsTbl(0);   // prepare timestamp for best time
    if(ts_tbl.find(p->ev_num) == ts_tbl.end()) {  // ts_tbl has not been configured for the specific ev_num
        sprintf(ev_name, "%d", p->ev_num);
        ts_tbl[p->ev_num].pevent = eventNameToHandle((const char *) ev_name);
        ts_tbl[p->ev_num].plock  = new epicsMutex();
        ts_tbl[p->ev_num].pQ     = (volatile Tpr::TprQueues *) ptr;
        ts_tbl[p->ev_num].ev_num = p->ev_num;
        ts_tbl[p->ev_num].ch_idx = p->ch_idx;
        ts_tbl[p->ev_num].index  = q.allwp[p->ch_idx] -1;      // an initial read point
        ts_tbl[p->ev_num].ev_enable = false;

    }

    ts_tbl_t *pT = &(ts_tbl[p->ev_num]);


    while(1) {
        read(fd, buff, 32);
        int64_t      allrp = q.allwp[p->ch_idx] -1;
        volatile uint32_t *dp = (&q.allq[q.allrp[p->ch_idx].idx[allrp &(MAX_TPR_ALLQ-1)] &(MAX_TPR_ALLQ-1) ].word[0]);


        if(prev_allrp == allrp) {   // there is no update
            if ( DEBUG_PCIE_TPR >= 2 )
                printf("%s thread %s is notified but, no new pattern\n", p->thread_name, p->file_name);
            epicsThreadSleep(1.);
            continue;
        }

        prev_allrp = allrp;

	volatile time_st_t *ts    = (volatile time_st_t *) dp;
	int lcls1_mode = ts->mode||((ts->pid64>>63)&1);

        pT->plock->lock();
        pT->time.secPastEpoch = dp[5];
        pT->time.nsec = dp[4];
        pT->rawpid = ts->pid64;
        pT->pid64 = lcls1_mode ? pT->time.nsec & 0x1ffff: ts->pid64;
        pT->plock->unlock();

        if(!ts->mode) {    // LCLS2 mode timestamp update, using the best timestamp
            uint64_t ts = *(uint64_t *)(&dp[4]);
            if(isNewTs(ts)) {
                ts_tbl_t *pT0 = &(ts_tbl[0]);
                pT0->plock->lock();
                pT0->time   = pT->time;
                pT0->rawpid = pT->rawpid;
                pT0->pid64  = pT->pid64;
                pT0->plock->unlock();
            }
        }

	// ch11 and LCLS1 mode, handle the LCLS1 timestamp and low number events
        if(master && lcls1_mode) {   
            int timeslot = (dp[6] >> 16 & 0x7);
            if(timeslot == 1 || timeslot == 4) {    // update active timeslot timestamp for TSE=-1
                ts_tbl_t *pT0 = &(ts_tbl[0]);
                pT0->plock->lock();
                pT0->time   = pT->time;
                pT0->rawpid = pT->rawpid;
                pT0->pid64  = pT->pid64;
                pT0->plock->unlock();
            }

            // proceed the event callback
            if(evCallback && ellCount(&evCallback->list) > 0) {
                evCallback->plock->lock();
                TimingEventCallback_t * p = (TimingEventCallback_t *) ellFirst(&evCallback->list);
                while(p) {
                    (*p->callback)(p->pUserPvt);
                    p = (TimingEventCallback_t *) ellNext(&p->node);
                }
                evCallback->plock->lock();
            }

            for(int i = 0; i <MAX_SOFT_EV; i++) {  // soft event loop
                if(soft_ev_list[i].ev_enable && soft_ev_list[i].ev_num >= 1 && soft_ev_list[i].ev_num <= 255) {  // soft event 
                    volatile uint32_t *ev_mask = &dp[14];
                    if(check_ev_mask(soft_ev_list[i].ev_num, ev_mask)) {
                        ts_tbl_t *pTN = &(ts_tbl[soft_ev_list[i].ev_num]);
                        pTN->plock->lock();
                        pTN->time   = pT->time;
                        pTN->rawpid = pT->rawpid;
                        pTN->pid64  = pT->pid64;
                        pTN->plock->unlock();
                        if(pTN->ev_enable) postEvent(pTN->pevent);
                    }
                }   // soft event

            }   // loop for soft event list
        }

        if(pT->ev_enable) postEvent(pT->pevent);

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

    if(eventCode == epicsTimeEventBestTime) eventCode = 0;

    if(ts_tbl.find(eventCode) == ts_tbl.end() || !time)  return -1;

    ts_tbl[eventCode].plock->lock();
    *time  = ts_tbl[eventCode].time;
    ts_tbl[eventCode].plock->unlock();

    return 0;
}

static int pcieTprTimeGetSystem_gtWrapper(epicsTimeStamp *time, int eventCode)
{
    if(epicsTimeGetCurrent(time)) return -1;

    return 0;
}


int pcieTpr_evPrefix(char *dev_prefix)
{
    int i;

    for(i = 0; dev_s[i]; i++) {
        if(!strcmp(dev_prefix, dev_s[i])) break;
    }

    if(dev_s[i]) return ev_prefix[i];
    else         return -1;    
}

int pcieTprEvCallbackInit(void)
{
    if(evCallback) return 0;

    evCallback        = new evCallback_t;
    evCallback->plock = new epicsMutex();
    ellInit(&evCallback->list);

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

    if(!dev_s[dev_idx])
    {
        printf( "pcieTprInit(%s): dev_s[%d] is NULL\n", dev_prefix, dev_idx );
        return (void *) NULL;
    }

    if ( DEBUG_PCIE_TPR >= 1 )
        printf( "pcieTprInit(%s): dev_idx=%d\n", dev_prefix, dev_idx );
    for(ch_idx = 0; dev_c[ch_idx]; ch_idx++) {
        ev_num = ev_prefix[dev_idx] + ch_idx;
        sprintf(file_name, "%s%c", dev_s[dev_idx], dev_c[ch_idx]);
        sprintf(thrd_name, "%s%c", name_s[dev_idx], dev_c[ch_idx]);

        if ( DEBUG_PCIE_TPR >= 2 )
            printf( "pcieTprInit: calling createPcieThread(%s, %s, ev %d, ch %d\n", thrd_name, file_name, ev_num, ch_idx );
        createPcieThread(thrd_name, file_name, ev_num, ch_idx);
    }

    return (void*) NULL;
}


void * pcieTprReport(int level)
{
    char time_text[40];
    char time_format[] = "%Y-%m-%d %H:%M:%S.%09f";
    int  time_text_len = 32;
    if(level < 1) return (void *) NULL;

    pcieTprEvCallbackInit();
    printf("Pcie Tpr Event Callback: %d registered\n", ellCount(&evCallback->list));
    TimingEventCallback_t *p = (TimingEventCallback_t *) ellFirst(&evCallback->list);
    while(p) {
        printf("\t callback: %p, pUserPvt: %p\n", p->callback, p->pUserPvt);
        p = (TimingEventCallback_t *) ellNext(&p->node); 
    }
    printf("Pcie Tpr Channel Event\n");
    for(std::map<int, ts_tbl_t>::iterator it = ts_tbl.begin(); it != ts_tbl.end(); it++) {
       ts_tbl_t *p = &it->second;
       epicsTimeToStrftime(time_text, time_text_len, time_format, &p->time);
       printf("(%d) ch_idx %d, ev_num %d, soft event: %s, last timestamp %s\n",
              it->first, p->ch_idx, p->ev_num, p->ev_enable? (char*) "enable": (char*) "disable", time_text);
    }
    printf("Pcie Tpr Soft Event\n");
    for(int i = 0; i < MAX_SOFT_EV; i++) {
        printf("soft_ev_num %d, soft ev %s\n", soft_ev_list[i].ev_num, soft_ev_list[i].ev_enable? (char*) "enable": (char *) "disable");
    }
    return (void *) NULL;
}


void   pcieTprSetSoftEv(int idx, int ev_num, bool ev_enable)
{
    if(idx < 0 || idx >= MAX_SOFT_EV || ev_num < 1 || ev_num > 255) return;

    prepLowTsTbl(ev_num);

    soft_ev_list[idx].ev_num    = ev_num;
    soft_ev_list[idx].ev_enable = ev_enable;

    ts_tbl[ev_num].ev_enable = ev_enable;

}

void pcieTprSetChEv(int ev_num, bool ev_enable)
{
    if(ts_tbl.find(ev_num) == ts_tbl.end()) return;
    ts_tbl[ev_num].ev_enable = ev_enable;
}


void  pcieTprInitSoftEv(void)
{
    for(int idx = 0; idx < MAX_SOFT_EV; idx++) {
        soft_ev_list[idx].ev_num    = -1;
        soft_ev_list[idx].ev_enable = false;
    }

}

void * pcieTprGPWrapper(void)
{
    if ( DEBUG_PCIE_TPR >= 1 )
        printf("Found PCIeTPR: execute GPWrapperInit()\n");

    generalTimeRegisterEventProvider("pcieTprTimeGet",       1000, (TIMEEVENTFUN) pcieTprTimeGet_gtWrapper);
    generalTimeRegisterEventProvider("pcieTprTimeGetSystem", 2000, (TIMEEVENTFUN) pcieTprTimeGetSystem_gtWrapper);

    return (void *) NULL;
}


extern "C" {

TimingPulseId timingGetLastFiducial(void)
{
    uint64_t pid64;
    ts_tbl_iter it = ts_tbl.find(0);
    ts_tbl_t *p;
    if ( it == ts_tbl.end() )
    {
        printf( "timingGetLastFiducial: Invalid timestamp table!\n" );
        return 0LL;
    }
    p = &it->second;
    if ( p->plock == NULL )
    {
        printf( "timingGetLastFiducial: Invalid timestamp table lock! p->plock = %p\n", p->plock );
        return 0LL;
    }
    p->plock->lock();
    pid64 = p->pid64;
    p->plock->unlock();
     
    return (TimingPulseId) pid64;
}

int timingGetLastXpmMode(void)
{
    uint64_t pid64;
    ts_tbl_iter it = ts_tbl.find(0);
    ts_tbl_t *p;
    if ( it == ts_tbl.end() )
    {
        printf( "timingGetLastFiducial: Invalid timestamp table!\n" );
        return 0LL;
    }
    p = &it->second;
    if ( p->plock == NULL )
    {
        printf( "timingGetLastFiducial: Invalid timestamp table lock! p->plock = %p\n", p->plock );
        return 0LL;
    }
    p->plock->lock();
    pid64 = p->rawpid;
    p->plock->unlock();
    return ((pid64>>63)&1)? 0 : 1;
}

int timingGetEventTimeStamp(epicsTimeStamp *ptime, int eventCode)
{
    return pcieTprTimeGet_gtWrapper(ptime, eventCode);
}

int timingGetCurrTimeStamp(epicsTimeStamp *ptime)
{
    return pcieTprTimeGet_gtWrapper(ptime, 0);
}

int timingEntryRead(unsigned int eventCode, void *dtpr, EventTimingData *pTimingDataDest)
{
    if(ts_tbl.find(eventCode) == ts_tbl.end() || !dtpr || !pTimingDataDest) return -2;   // invalid index or inputs

    ts_tbl_t *pT = &(ts_tbl[eventCode]);   

    if(pT->pQ) pT->index = pT->pQ->allwp[pT->ch_idx] -1;
    else       return -1;                 // NULL to return

    volatile uint32_t *dp = (&pT->pQ->allq[pT->pQ->allrp[pT->ch_idx].idx[pT->index &(MAX_TPR_ALLQ-1)] &(MAX_TPR_ALLQ-1) ].word[0]);
    volatile Tpr::TprEntry *p = (Tpr::TprEntry *) dp;
    volatile time_st_t *ts    = (volatile time_st_t *) dp;
    pTimingDataDest->fifo_time.secPastEpoch = dp[5];
    pTimingDataDest->fifo_time.nsec         = dp[4];
    pTimingDataDest->fifo_fid               = ts->mode ? ts->pid64 & 0x1ffff: ts->pid64;
    pTimingDataDest->fifo_tsc               = p->fifo_tsc;

    memcpy(dtpr, (void *) p, sizeof(Tpr::TprEntry));

   return 0;

}

int timingFifoRead(unsigned int    eventCode,
                   int             incr,
                   uint64_t        *index,
                   EventTimingData *pTimingDataDest)
{
    if(ts_tbl.find(eventCode) == ts_tbl.end() ||
       !index || !pTimingDataDest) return -2;   // invalid index or inputs

    ts_tbl_t *pT = &(ts_tbl[eventCode]);


    if(incr == TS_INDEX_INIT) {
        if(pT->pQ) *index = pT->index = pT->pQ->allwp[pT->ch_idx] -1;
        else       return -1;                 // NULL to return

        volatile uint32_t *dp = (&pT->pQ->allq[pT->pQ->allrp[pT->ch_idx].idx[pT->index &(MAX_TPR_ALLQ-1)] &(MAX_TPR_ALLQ-1) ].word[0]);
        volatile Tpr::TprEntry *p = (Tpr::TprEntry *) dp;
        volatile time_st_t *ts    = (volatile time_st_t *) dp;
        pTimingDataDest->fifo_time.secPastEpoch = dp[5];
        pTimingDataDest->fifo_time.nsec         = dp[4];
        pTimingDataDest->fifo_fid               = ts->mode? ts->pid64 &0x1ffff: ts->pid64;
        pTimingDataDest->fifo_tsc               = p->fifo_tsc;
    } else {
        volatile uint32_t *dp_prev = (&pT->pQ->allq[pT->pQ->allrp[pT->ch_idx].idx[pT->index &(MAX_TPR_ALLQ-1)] &(MAX_TPR_ALLQ-1) ].word[0]);
        pT->index += incr;  *index = pT->index;
        volatile uint32_t *dp = (&pT->pQ->allq[pT->pQ->allrp[pT->ch_idx].idx[pT->index &(MAX_TPR_ALLQ-1)] &(MAX_TPR_ALLQ-1) ].word[0]);
        uint64_t t_prev = *(uint64_t *)(&dp_prev[4]);
        uint64_t t_new  = *(uint64_t *)(&dp[4]);

        if(incr > 0 && t_prev > t_new) return -4;      // FIFO underrun
        if(incr < 0 && t_prev < t_new) return -3;      // FIFO overrun

        volatile Tpr::TprEntry *p = (Tpr::TprEntry *) dp;
        volatile time_st_t *ts    = (volatile time_st_t *) dp;
        pTimingDataDest->fifo_time.secPastEpoch = dp[5];
        pTimingDataDest->fifo_time.nsec         = dp[4];
        pTimingDataDest->fifo_fid               = ts->mode ? ts->pid64 & 0x1ffff : ts->pid64;
        pTimingDataDest->fifo_tsc               = p->fifo_tsc;
    }



    return 0;
}

int RegisterTimingEventCallback(TimingEventCallback callback, void *pUserPvt)
{
    pcieTprEvCallbackInit();

    TimingEventCallback_t *p = new TimingEventCallback_t;
    p->callback = callback;
    p->pUserPvt = pUserPvt;

    ellAdd(&evCallback->list, &p->node);


    return 0;
}

}  /* extern C */

