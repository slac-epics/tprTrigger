#ifndef TPR_TRIGGER_ASYN_DRIVER_H
#define TPR_TRIGGER_ASYN_DRIVER_H

#include <ellLib.h>
#include <asynPortDriver.h>
#include <epicsEvent.h>
#include <epicsTypes.h>
#include <epicsTime.h>

#include <ellLib.h>


class tprTriggerAsynDriver:asynPortDriver {
    public:
        tprTriggerAsynDriver(const char *portName, const char *corePath);
        Tpr::TprTriggerYaml* getApiDrv(void) { return pApiDrv; }
        void CreateParameters(void);
        void Monitor(void);
        void SetDebug(int debug);
        
        asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
        asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
        
    private:
        Tpr::TprTriggerYaml *pApiDrv;
        
        epicsFloat64 lcls2_clock;
        
        epicsFloat64 application_clock_1;
        epicsFloat64 application_clock_2;
        
        void SetClock1(epicsFloat64 clock_mhz);
        void SetClock2(epicsFloat64 clock_mhz);
        
        void SetMode(epicsInt32 mode);
        void SetMsgDelay(epicsFloat64 msg_delay);
        void SetMasterDelay(epicsFloat64 master_delay);

        void SetLCLS1ChannelEnable(int channel, epicsInt32 enable);
        void SetLCLS2ChannelEnable(int channel, epicsInt32 enable);
        void SetRateMode(int channel, epicsInt32 rate_mode);
        void SetFixedRate(int channel, epicsInt32 fixed_rate);
        void SetACRate(int channel, epicsInt32 ac_rate);
        void SetTSMask(int channel, epicsInt32 ts_mask);
        void SetSeqBit(int channel, epicsInt32 seq_bit);
        void SetDestMode(int channel, epicsInt32 dest_mode);
        void SetDestMask(int channel, epicsInt32 dest_mask);
        void SetEventCode(int channel, epicsInt32 event_code);
        
        
        void SetLCLS1TriggerEnable(int trigger, epicsInt32 enable);
        void SetLCLS2TriggerEnable(int trigger, epicsInt32 enable);
        void SetSource(int trigger, epicsInt32 source);
        void SetPolarity(int trigger, epicsInt32 polarity);
        void SetLCLS1Delay(int trigger, epicsFloat64 delay);
        void SetLCLS2Delay(int trigger, epicsFloat64 delay);
        void SetLCLS1Width(int trigger, epicsFloat64 width);
        void SetLCLS2Width(int trigger, epicsFloat64 width);
             
        int _update_flag;
        uint32_t _prev_chn_counter[NUM_CHANNELS];
        
    protected:
    //
    // parameter section for asynPortDrver
    //
    
        int firstTprTrgParam;
#define FIRST_TPR_TRG_PARAM    firstTprTrgParam

        int p_fpga_version;        /* asynInt32, ro */
        int p_uptime_counter;      /* asynInt32, ro */
        int p_sof_counter;         /* asynInt32, ro */
        int p_eof_counter;         /* asynInt32, ro */
        int p_fid_counter;         /* asynInt32, ro */
        int p_crc_err_counter;     /* asynInt32, ro */
        int p_rx_clock_counter;    /* asynInt32, ro */
        int p_rx_link_status;      /* asynInt32, ro */
        int p_version_error;       /* asynInt32, ro */
        int p_frame_version;       /* asynInt32, ro */
        
        int p_mode;               /* asynInt32, rw, 0: LCLS1, 1: LCLS2 */
        int p_msg_delay;          /* asynFloat64, rw */  // LCLS2 100 pulses delay and fine adjust 
        int p_master_delay;       /* asynFloat64, rw */  // LCLS1 master delay adjust
        
        int p_app_clock_1;          /* asynFloat64, rw */  // application clock in MHz for LCLS1
        int p_app_clock_2;          /* asynFloat64, rw */  // application clock in MHz for LCLS2
  
        struct {
            int p_enable[2];         /* asynInt32, rw, 0: diable, 1: enable */
            
            int p_rate_mode;      /* asynInt32, rw, 0: fixed rate, 1: AC rate, 2: Seq, 3: Reserved */
            int p_fixed_rate;     /* asynInt32, rw, 0: to 6: */
            int p_ac_rate;        /* asynInt32, rw, 0: to 5: */
            int p_ts_mask;        /* asynInt32, rw, 6bits mask */
            int p_seq_bit;        /* asynInt32, rw */
            int p_dest_mode;      /* asynInt32, rw, 0: Inclusive, 1: Exclusive, 2: Don't care, 3: Reserved */
            int p_dest_mask;      /* asynInt32, rw */
            
            int p_event_code;     /* asynInt32, rw */
            
            int p_counter;        /* asynInt32, ro */
            int p_rate;           /* asynFloat64, rw */
            
        } p_channel_st[NUM_CHANNELS];
        
        struct {
            int p_enable[2];         /* asynInt32, rw, 0: disable, 1: enable */
            int p_source;         /* asynInt32, rw */
            int p_polarity;       /* asynInt32, rw */
            
            int p_width[2];       /* asynFloat64, rw */
            int p_delay[2];       /* asynFloat64, rw */
            
            int p_widthTicks;     /* asynInt32, ro */
            int p_delayTicks;     /* asynInt32, ro */
        } p_trigger_st[NUM_TRIGGERS];
        


        int lastTprTrgParam;
#define LAST_TPR_TRG_PARAM     lastTprTrgParam

};

#define NUM_TPR_TRG_DET_PARAMS ((int)(&LAST_TPR_TRG_PARAM-&FIRST_TPR_TRG_PARAM -1))

#define fpgaVersionString          "fpgaVersion"
#define uptimeCounterString        "uptime"
#define sofCounterString           "sofCounter"
#define eofCounterString           "eofCounter"
#define fidCounterString           "fidCounter"
#define crckErrCounterString       "crcErrCounter"
#define rxClockCounterString       "rxClockCounter"
#define rxLinkStatusString         "rxLinkStatus"
#define versionErrorString         "versionError"
#define frameVersionString         "frameVersion"

#define modeString                 "mode"

#define msgDelayString             "msgDelay"
#define masterDelayString          "masterDelay"
#define appClock1String            "applicationClock1"
#define appClock2String            "applicationClock2"

#define chnEnableString            "chnEnable_C%dLCLS%d"
#define chnRateModeString          "chnRateMode_C%d"
#define chnFixedRateString         "chnFixedRate_C%d"
#define chnACRateString            "chnACRate_C%d"
#define chnTSMaskString            "chnTSMask_C%d"
#define chnSeqBitString            "chnSeqBit_C%d"
#define chnDestModeString          "chnDestMode_C%d"
#define chnDestMaskString          "chnDestMask_C%d"
#define chnEventCodeString         "chnEventCode_C%d"
#define chnCounterString           "chnCounter_C%d"
#define chnRateString              "chnRate_C%d"

#define trgEnableString            "trgEnable_T%dLCLS%d"
#define trgSourceString            "trgSource_T%d"
#define trgPolarityString          "trgPolarity_T%d"
#define trgDelayString             "trgDelay_T%dLCLS%d"
#define trgWidthString             "trgWidth_T%dLCLS%d"

#define trgDelayTicksString        "trgDelayTicks_T%d"
#define trgWidthTicksString        "trgWidthTicks_T%d"



typedef struct {
    ELLNODE     node;
    char        port_name[64];
    char        core_path[256];
    Tpr::TprTriggerYaml  *pApiDrv;
    tprTriggerAsynDriver *pAsynDrv;
} tprTriggerDrvList_t;


#endif