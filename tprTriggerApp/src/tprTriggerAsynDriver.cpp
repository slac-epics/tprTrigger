//////////////////////////////////////////////////////////////////////////////
// This file is part of 'tprTrigger'.
// It is subject to the license terms in the LICENSE.txt file found in the 
// top-level directory of this distribution and at: 
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
// No part of 'tprTrigger', including this file, 
// may be copied, modified, propagated, or distributed except according to 
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <time.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <ellLib.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>

#include <ellLib.h>

#include <iocsh.h>

#include <drvSup.h>
#include <epicsExport.h>
#include <envDefs.h>

#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>

#include <yamlLoader.h>

#include <tprTriggerYaml.hh>
#include <tprTriggerAsynDriver.h>

#define EPICS_ENV_VALID_TPR_CHANNELS      "tprValidChannels"

static const char * drverName = "tprTriggerAsynDriver";

static int init_flag = 0;
static ELLLIST *pList = NULL;

extern "C" {
static int tprValidChannels  = -1;
epicsExportAddress(int, tprValidChannels);
}

   

static void init_pList(void)
{
    if(!pList) {
        pList = new ELLLIST;
        ellInit(pList);
    }
}

static const char * _num2Str(int i)
{
    static const char *_str[] = { "00", "01", "02", "03", "04", "05", "06", "07",
                                  "08", "09", "10", "11", "12", "13", "14", "15", ""};

    if(i<0 || i>15) return _str[16];
    else            return _str[i];
}


Tpr::TprTriggerYaml *p;

static void tick(epicsFloat64 t, epicsFloat64 clock, uint32_t *tick)
{
    *tick = t * 1.E-3 * clock + 0.5;
}

static void tick(epicsFloat64 t, epicsFloat64 clock, uint32_t *tick, uint32_t *tap)
{
    epicsFloat64 ts = t * 1.E-3 * clock;

    *tick = ts;
    *tap  = (ts - (int) ts) * 64. * 190. / clock + 0.5;
    if(*tap > 62) *tap = 62;

}

void API_TEST(void)
{

    printf("API_TEST(): Report\n");
    p->report();
}

void API_TEST_INIT(void)
{

    printf("API_TEST_INIT():\n");
    Path _core = cpswGetRoot()->findByName("mmio/AmcCarrierEmpty/AmcCarrierCore");
    p = new Tpr::TprTriggerYaml(_core);
    
}

tprTriggerAsynDriver::tprTriggerAsynDriver(const char *portName, const char *corePath, const char *named_root)
    : asynPortDriver(portName,
                     1,
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
                     NUM_TPR_TRG_DET_PARAMS,
#endif /* asyn version check, under 4.32 */
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask | asynInt32ArrayMask | asynInt16ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynEnumMask | asynInt32ArrayMask | asynInt16ArrayMask,
                     1,
                     1,
                     0,
                     0)
{
    Path _core;
    busType = _atca;
    if(named_root && !strlen(named_root)) named_root = NULL;
    if(corePath && strlen(corePath)) {
        if(!strncmp(corePath, "PCIe:/", 6) || !strncmp(corePath, "pcie:/", 6)) busType = _pcie;
        else                                                                   busType = _atca;
    } else {
        busType = _atca;
    }

    switch(busType) {
        case _atca:
            _core   = ((!named_root)?cpswGetRoot():cpswGetNamedRoot(named_root))->findByName(corePath);
            pApiDrv = new Tpr::TprTriggerYaml(_core, 0);
            break;
        case _pcie:
            _core   =((!named_root)?cpswGetRoot():cpswGetNamedRoot(named_root))->findByName(corePath+6);;
            pApiDrv = new Tpr::TprTriggerYaml(_core, 1);
            break;
    }



    pApiDrv->_debug_ = 0;      /* turn on the debugging message in API layer */
    // pApiDrv->_debug_ = 0;   /* turn off the debugging message in API layer */
    
    lcls2_clock = (1300./7.);  // 186MHz
    application_clock_1 = 119.;         // 119MHz as a default for LCLS1
    application_clock_2 = (1300./7.);   // 186MHz as a default for LCLS2
    
    _update_flag =0;


    if(tprValidChannels > 0 && tprValidChannels < 16) {
        valid_chns = tprValidChannels;
        valid_trgs = tprValidChannels;
    }
    
    CreateParameters();
}


void tprTriggerAsynDriver::CreateParameters(void)
{
    createParam(fpgaVersionString,     asynParamInt32, &p_fpga_version);
    createParam(uptimeCounterString,   asynParamInt32, &p_uptime_counter);
    createParam(sofCounterString,      asynParamInt32, &p_sof_counter);
    createParam(eofCounterString,      asynParamInt32, &p_eof_counter);
    createParam(fidCounterString,      asynParamInt32, &p_fid_counter);
    createParam(crckErrCounterString,  asynParamInt32, &p_crc_err_counter);
    createParam(rxRstCounterString,    asynParamInt32, &p_rx_rst_counter);
    createParam(rxDecErrCounterString, asynParamInt32, &p_rx_dec_err_counter);
    createParam(rxDspErrCounterString, asynParamInt32, &p_rx_dsp_err_counter);
    createParam(rxClockCounterString,  asynParamInt32, &p_rx_clock_counter);
    createParam(rxLinkStatusString,    asynParamInt32, &p_rx_link_status);
    createParam(versionErrorString,    asynParamInt32, &p_version_error);
    createParam(frameVersionString,    asynParamInt32, &p_frame_version);
    
    createParam(modeString,            asynParamInt32, &p_mode);
    
    createParam(msgDelayString,        asynParamFloat64, &p_msg_delay);
    createParam(msgDelayRBString,      asynParamInt32,   &p_msg_delay_ticks);
    createParam(masterDelayString,     asynParamFloat64, &p_master_delay);
    
    createParam(appClock1String,       asynParamFloat64, &p_app_clock_1);
    createParam(appClock2String,       asynParamFloat64, &p_app_clock_2);

    createParam(uedSpecialString,      asynParamInt32,  &p_ued_special);
    
    
    for(int i = 0; i < valid_chns; i++) {
        char param_name[128];
        
        for(int j =0; j < 2; j++) {
            sprintf(param_name, chnEnableString, _num2Str(i),j+1);  createParam(param_name, asynParamInt32, &((p_channel_st+i)->p_enable[j]));
        }
        sprintf(param_name, chnRateModeString, _num2Str(i));  createParam(param_name, asynParamInt32, &((p_channel_st+i)->p_rate_mode));
        sprintf(param_name, chnFixedRateString, _num2Str(i)); createParam(param_name, asynParamInt32, &((p_channel_st+i)->p_fixed_rate));
        sprintf(param_name, chnACRateString, _num2Str(i));    createParam(param_name, asynParamInt32, &((p_channel_st+i)->p_ac_rate));
        sprintf(param_name, chnTSMaskString, _num2Str(i));    createParam(param_name, asynParamInt32, &((p_channel_st+i)->p_ts_mask));
        sprintf(param_name, chnSeqNumString, _num2Str(i));    createParam(param_name, asynParamInt32, &((p_channel_st+i)->p_seq_num));
        sprintf(param_name, chnSeqBitString, _num2Str(i));    createParam(param_name, asynParamInt32, &((p_channel_st+i)->p_seq_bit));
        sprintf(param_name, chnDestModeString, _num2Str(i));  createParam(param_name, asynParamInt32, &((p_channel_st+i)->p_dest_mode));
        sprintf(param_name, chnDestMaskString, _num2Str(i));  createParam(param_name, asynParamInt32, &((p_channel_st+i)->p_dest_mask));
        sprintf(param_name, chnEventCodeString, _num2Str(i)); createParam(param_name, asynParamInt32, &((p_channel_st+i)->p_event_code));
        sprintf(param_name, chnCounterString, _num2Str(i));   createParam(param_name, asynParamInt32, &((p_channel_st+i)->p_counter));
        sprintf(param_name, chnRateString, _num2Str(i));      createParam(param_name, asynParamFloat64, &((p_channel_st+i)->p_rate));
    }
    
    
    for(int i = 0; i < valid_trgs; i++) {
        char param_name[128];
        for(int j = 0; j < 2; j++) {
            sprintf(param_name, trgEnableString, _num2Str(i), j+1);    createParam(param_name, asynParamInt32, &((p_trigger_st+i)->p_enable[j]));
        }
        sprintf(param_name, trgSourceString, _num2Str(i));    createParam(param_name, asynParamInt32, &((p_trigger_st+i)->p_source));
        sprintf(param_name, trgPolarityString, _num2Str(i));  createParam(param_name, asynParamInt32, &((p_trigger_st+i)->p_polarity));
        
        for(int j = 0; j < 2; j++) {
            sprintf(param_name, trgDelayString, _num2Str(i), j+1);  createParam(param_name, asynParamFloat64, &((p_trigger_st+i)->p_delay[j]));
            sprintf(param_name, trgWidthString, _num2Str(i), j+1);  createParam(param_name, asynParamFloat64, &((p_trigger_st+i)->p_width[j]));

            sprintf(param_name, propDelayString, _num2Str(i), j+1); createParam(param_name, asynParamFloat64, &((p_trigger_st+i)->p_prop_delay[j]));
            sprintf(param_name, propWidthString, _num2Str(i), j+1); createParam(param_name, asynParamFloat64, &((p_trigger_st+i)->p_prop_width[j]));
            sprintf(param_name, propEnableString, _num2Str(i), j+1); createParam(param_name, asynParamInt32,  &((p_trigger_st+i)->p_prop_enable[j]));
        }
        sprintf(param_name, trgDelayTapsString,  _num2Str(i));      createParam(param_name, asynParamInt32, &((p_trigger_st+i)->p_delayTaps));
        sprintf(param_name, trgDelayTicksString, _num2Str(i));      createParam(param_name, asynParamInt32, &((p_trigger_st+i)->p_delayTicks));
        sprintf(param_name, trgWidthTicksString, _num2Str(i));      createParam(param_name, asynParamInt32, &((p_trigger_st+i)->p_widthTicks));

        sprintf(param_name, trgTDESString, _num2Str(i));            createParam(param_name, asynParamFloat64, &((p_trigger_st+i)->p_tdes));
        sprintf(param_name, trgTWIDString, _num2Str(i));            createParam(param_name, asynParamFloat64, &((p_trigger_st+i)->p_twid));
 
        sprintf(param_name, trgTCTLString, _num2Str(i));            createParam(param_name, asynParamInt32,   &((p_trigger_st+i)->p_tctl));
        sprintf(param_name, trgTPOLString, _num2Str(i));            createParam(param_name, asynParamInt32,   &((p_trigger_st+i)->p_tpol));
        sprintf(param_name, trgCMPLString, _num2Str(i));            createParam(param_name, asynParamInt32,   &((p_trigger_st+i)->p_cmpl));

        sprintf(param_name, propTDESString, _num2Str(i));           createParam(param_name, asynParamFloat64, &((p_trigger_st+i)->p_prop_tdes));
        sprintf(param_name, propTWIDString, _num2Str(i));           createParam(param_name, asynParamFloat64, &((p_trigger_st+i)->p_prop_twid));

        sprintf(param_name, propTCTLString, _num2Str(i));           createParam(param_name, asynParamInt32,   &((p_trigger_st+i)->p_prop_tctl));
        sprintf(param_name, propTPOLString, _num2Str(i));           createParam(param_name, asynParamInt32,   &((p_trigger_st+i)->p_prop_tpol));
        sprintf(param_name, propPolarityString, _num2Str(i));       createParam(param_name, asynParamInt32,   &((p_trigger_st+i)->p_prop_polarity));
    }
}

void tprTriggerAsynDriver::Monitor(void)
{
    uint32_t val;
    
    val = pApiDrv->fpgaVersion();   setIntegerParam(p_fpga_version, val);
    val = pApiDrv->upTimeCount();   setIntegerParam(p_uptime_counter, val);
    val = pApiDrv->sofCount();      setIntegerParam(p_sof_counter, val);
    val = pApiDrv->eofCount();      setIntegerParam(p_eof_counter, val);
    val = pApiDrv->fidCount();      setIntegerParam(p_fid_counter, val);
    val = pApiDrv->crcErrCount();   setIntegerParam(p_crc_err_counter, val);
    val = pApiDrv->rxRstCount();    setIntegerParam(p_rx_rst_counter, val);
    val = pApiDrv->rxDecErrCount(); setIntegerParam(p_rx_dec_err_counter, val);
    val = pApiDrv->rxDspErrCount(); setIntegerParam(p_rx_dsp_err_counter, val);
    val = pApiDrv->rxClkCount();    setIntegerParam(p_rx_clock_counter, val);
    val = pApiDrv->rxLinkStatus();  setIntegerParam(p_rx_link_status, val);
    val = pApiDrv->versionErr();    setIntegerParam(p_version_error, val);
    val = pApiDrv->frameVersion();  setIntegerParam(p_frame_version, val);
    
    for(int i=0; i< valid_chns; i++) {
        val = pApiDrv->channelCount(i); setIntegerParam((p_channel_st+i)->p_counter, val);
        if(_update_flag) {
            if(val >= _prev_chn_counter[i]) { 
                epicsFloat64 _rate = (val - _prev_chn_counter[i])/2.;
                setDoubleParam((p_channel_st+i)->p_rate, _rate);
            }
            _prev_chn_counter[i] = val;
        }
    }
    
    _update_flag = _update_flag?0:1;
    
    callParamCallbacks();
}

void tprTriggerAsynDriver::SetDebug(int debug)
{
    if(pApiDrv) pApiDrv->_debug_ = debug;
}


asynStatus tprTriggerAsynDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeInt32";
    
    /* set the parameter in the parameter library */
    status = (asynStatus) setIntegerParam(function, value);
    
    if(function == p_mode) SetMode(value);
    
        
    for(int i = 0; i < valid_chns; i++) {
        for(int j =0; j <2; j++) {
            if(function == (p_channel_st +i)->p_enable[j]) {
                (!j)?SetLCLS1ChannelEnable(i, value):SetLCLS2ChannelEnable(i, value);
                break;
            }
        }
        if(function == (p_channel_st +i)->p_rate_mode) {
            SetRateMode(i, value);
            break;
        } else
        if(function == (p_channel_st +i)->p_fixed_rate) {
            SetFixedRate(i, value);
            break;
        } else
        if(function == (p_channel_st +i)->p_ac_rate) {
            SetACRate(i, value);
            break;
        } else
        if(function == (p_channel_st +i)->p_ts_mask) {
            SetTSMask(i, value);
            break;
        } else
        if(function == (p_channel_st +i)->p_seq_num) {
            epicsInt32 seq_bit; getIntegerParam((p_channel_st + i)->p_seq_bit, &seq_bit);
            SetSeqBit(i, (uint32_t) (((value & 0x1f)<<4)|(seq_bit & 0xf)));
        } else
        if(function == (p_channel_st +i)->p_seq_bit) {
            epicsInt32 seq_num; getIntegerParam((p_channel_st + i)->p_seq_num, &seq_num);
            SetSeqBit(i, (uint32_t) (((seq_num & 0x1f)<<4)|(value & 0xf)));
            break;
        } else
        if(function == (p_channel_st +i)->p_dest_mode) {
            SetDestMode(i, value);
            break;
        } else
        if(function == (p_channel_st +i)->p_dest_mask) {
            SetDestMask(i, value);
            break;
        } else
        if(function == (p_channel_st +i)->p_event_code) {
            SetEventCode(i, value);
            break;
        }   
    }
    
    for(int i =0; i < valid_trgs; i++) {
        for(int j = 0; j<2; j++) {
            if(function == (p_trigger_st +i)->p_enable[j]) {
                (!j)?SetLCLS1TriggerEnable(i, value): SetLCLS2TriggerEnable(i, value);
                break;
            }
        }
        if(function == (p_trigger_st +i)->p_source) {
            SetSource(i, value);
            break;
        } else
        if(function == (p_trigger_st +i)->p_polarity) {
            SetPolarity(i, value);
            break;
        } else
        if(function == (p_trigger_st +i)->p_tpol) {
            PropagatePolarity(i, value);
            break;
        } else
        if(function == (p_trigger_st +i)->p_tctl) {
            PropagateEnable(i, value);
            break;
        } else
        if(function == (p_trigger_st+i)->p_cmpl) {
            if(busType == _pcie) SetCmpl(i, value);
            break;
        }
    }
    

    if(function == p_ued_special) SetUedSpecialMode(value);
    
    callParamCallbacks();
    
    return status;
}

asynStatus tprTriggerAsynDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char * functionName = "writeFloat64";
    
    /* set the parameter in the parameter library */
    status = (asynStatus) setDoubleParam(function, value);
    
    
    if(function == p_msg_delay) {
        SetMsgDelay(value);
    } else
    if(function == p_master_delay) {
        SetMasterDelay(value);
    } else
    if(function == p_app_clock_1) {
        SetClock1(value);
    } else
    if(function == p_app_clock_2) {
        SetClock2(value);
    }
    
    for(int i = 0; i< valid_trgs; i++) {
        for(int j = 0; j <2 ; j++) {
            if(function == (p_trigger_st +i)->p_width[j]) {
                (!j)?SetLCLS1Width(i, value):SetLCLS2Width(i, value);
                break;
            } else
            if(function == (p_trigger_st +i)->p_delay[j]) {
                (!j)?SetLCLS1Delay(i, value):SetLCLS2Delay(i, value);
                break;
            }
        }

        if(function == (p_trigger_st + i)->p_twid) {
            PropagateWidth(i, value);
            break;
        }

        if(function == (p_trigger_st +i)->p_tdes) {
            PropagateDelay(i, value);
            break;
        }

    }

    callParamCallbacks();
    
    return status;
}

void tprTriggerAsynDriver::SetClock1(epicsFloat64 clock_mhz)
{
    if(clock_mhz <= 0.) application_clock_1 = (119.);
    else                application_clock_1 = clock_mhz;
    
    epicsInt32 mode; getIntegerParam(p_mode, &mode); mode = !mode?0:1;
    if(mode !=0) return;  // in LCLS2 mode, nothing todo
    
    uint32_t disable(0); 
    for(int i = 0; i < valid_trgs; i++) pApiDrv->TriggerEnable(i, disable);
    
    for(int i =0; i<valid_trgs; i++) {
        epicsFloat64 width; getDoubleParam((p_trigger_st +i)->p_width[0], &width);
        // uint32_t ticks = (width*1.E-3 * application_clock_1) + 0.5;
        uint32_t ticks, taps;
        tick(width, application_clock_1, &ticks);
        if(!ticks) ticks = 1;
        pApiDrv->SetWidth(i, ticks); setIntegerParam((p_trigger_st+i)->p_widthTicks, ticks);
           
        epicsFloat64 master_delay; getDoubleParam(p_master_delay, &master_delay);
        epicsFloat64 delay;        getDoubleParam((p_trigger_st+i)->p_delay[0], &delay);
        // ticks = ((master_delay+delay)*1.E-3 * application_clock_1) + 0.5;
        if(busType == _pcie) {
            tick(master_delay + delay, application_clock_1, &ticks); taps = 0;
            pApiDrv->SetDelayTap(i, taps); setIntegerParam((p_trigger_st+i)->p_delayTaps, taps);
        } else {
            tick(master_delay + delay, application_clock_1, &ticks);
        }
        pApiDrv->SetDelay(i, ticks); setIntegerParam((p_trigger_st+i)->p_delayTicks, ticks);
    }
    
    for(int i = 0; i <valid_trgs; i++) {
        epicsInt32 _enable; getIntegerParam((p_trigger_st +i)->p_enable[mode], &_enable);
        pApiDrv->TriggerEnable(i, (uint32_t) _enable);
    }
    
}

void tprTriggerAsynDriver::SetClock2(epicsFloat64 clock_mhz)
{
    if(clock_mhz <= 0.) application_clock_2 = (1300./7.);
    else                application_clock_2 = clock_mhz;
    
    epicsInt32 mode; getIntegerParam(p_mode, &mode); mode = !mode?0:1;
    if(mode != 1) return;  // in LCLS1 mode, nothing todo
    
    uint32_t disable(0);
    for(int i = 0; i < valid_trgs; i++) pApiDrv->TriggerEnable(i, disable);

    for(int i =0; i<valid_trgs; i++) {
        epicsFloat64 width; getDoubleParam((p_trigger_st+i)->p_width[1], &width);
        // uint32_t ticks = (width*1.E-3 * application_clock_2) + 0.5;
        uint32_t ticks, taps;
        tick(width, application_clock_2, &ticks);
        if(!ticks) ticks = 1;
        pApiDrv->SetWidth(i, ticks); setIntegerParam((p_trigger_st+i)->p_widthTicks, ticks);
            
        epicsFloat64 delay; getDoubleParam((p_trigger_st+i)->p_delay[1], &delay);
        // ticks = (delay*1.E-3 * application_clock_2) + 0.5;
        if(busType == _pcie) {
            tick(delay, application_clock_2, &ticks, &taps);
            pApiDrv->SetDelayTap(i, taps); setIntegerParam((p_trigger_st+i)->p_delayTaps, taps);
        } else {
            tick(delay, application_clock_2, &ticks);
        }
        pApiDrv->SetDelay(i, ticks); setIntegerParam((p_trigger_st+i)->p_delayTicks, ticks);
    }
    
    
    for(int i = 0; i <valid_trgs; i++) {
        epicsInt32 _enable; getIntegerParam((p_trigger_st +i)->p_enable[mode], &_enable);
        pApiDrv->TriggerEnable(i, (uint32_t) _enable);
    }
}

void tprTriggerAsynDriver::SetMode(epicsInt32 mode)
{
    uint32_t disable(0), enable(1);
    mode = !mode?0:1;
    
    // disable all of channels and all of triggers
    for(int i =0; i <valid_chns; i++) pApiDrv->ChannelEnable(i, disable);
    for(int i =0; i <valid_trgs; i++) pApiDrv->TriggerEnable(i, disable);
    
    if(mode == 0) { /* LCLS1 mode */
    
        // channel, filtering section
        for(int i = 0; i < valid_chns; i++) {
            epicsInt32 event_code; getIntegerParam((p_channel_st +i)->p_event_code, &event_code);
            pApiDrv->SetEventCode(i, (uint32_t) event_code);
        }
        
        // trigger section
        for(int i =0; i<valid_trgs; i++) {
            epicsFloat64 width; getDoubleParam((p_trigger_st +i)->p_width[0], &width);
            // uint32_t ticks = (width*1.E-3 * application_clock_1) + 0.5;
            uint32_t ticks, taps;
            tick(width, application_clock_1, &ticks);
            if(!ticks) ticks = 1;
            pApiDrv->SetWidth(i, ticks); setIntegerParam((p_trigger_st+i)->p_widthTicks, ticks);
            PropagateTWID(i, width);
            
            epicsFloat64 master_delay; getDoubleParam(p_master_delay, &master_delay);
            epicsFloat64 delay;        getDoubleParam((p_trigger_st+i)->p_delay[0], &delay);
            // ticks = ((master_delay+delay)*1.E-3 * application_clock_1) + 0.5;
            if(busType == _pcie) {
                tick(master_delay + delay, application_clock_1, &ticks); taps = 0;
                pApiDrv->SetDelayTap(i, taps); setIntegerParam((p_trigger_st+i)->p_delayTaps, taps);
            } else {
                tick(master_delay + delay, application_clock_1, &ticks);
            }
            pApiDrv->SetDelay(i, ticks); setIntegerParam((p_trigger_st+i)->p_delayTicks, ticks);
            PropagateTDES(i, delay);

            int _enable; getIntegerParam((p_trigger_st +i)->p_enable[0], &_enable);
            PropagateTCTL(i, _enable);

        }
        
        pApiDrv->SetClkSel(0);  /* set clcok for LCLS1 */
        pApiDrv->SetModeSel(0); /* set protocol mode for LCLS1 */ 
    
    
    } else { /* LCLS 2 mode */
    
        // channel, filtering section
        for(int i = 0; i < valid_chns; i++) {
            epicsInt32 rate_mode, fixed_rate, ac_rate, ts_mask, seq_num, seq_bit, dest_mode, dest_mask;
            getIntegerParam((p_channel_st+i)->p_rate_mode, &rate_mode);
            getIntegerParam((p_channel_st+i)->p_fixed_rate, &fixed_rate);
            getIntegerParam((p_channel_st+i)->p_ac_rate, &ac_rate);
            getIntegerParam((p_channel_st+i)->p_ts_mask, &ts_mask);
            getIntegerParam((p_channel_st+i)->p_seq_num, &seq_num);
            getIntegerParam((p_channel_st+i)->p_seq_bit, &seq_bit);
            getIntegerParam((p_channel_st+i)->p_dest_mode, &dest_mode);
            getIntegerParam((p_channel_st+i)->p_dest_mask, &dest_mask);
            
            switch(rate_mode) {
                case 0: /* Fixed Rate */
                    pApiDrv->SetFixedRate(i, (uint32_t)fixed_rate);
                    break;
                case 1: /* AC Rate */
                    pApiDrv->SetACRate(i, (uint32_t)ts_mask, (uint32_t)ac_rate);
                    break;
                case 2: /* Seq */
                    pApiDrv->SetSeqBit(i, (uint32_t) (((seq_num & 0x1f)<<4) | (seq_bit &0xf)));
                    break;
            }
            switch(dest_mode) {
                case 0: /* Inclusive */
                    pApiDrv->SetInclusionMask(i, (uint32_t)dest_mask);
                    break;
                case 1: /* Exclusive */
                    pApiDrv->SetExclusionMask(i, (uint32_t)dest_mask);
                    break;
                case 2: /* don't care */
                    pApiDrv->SetDontCareMask(i);
                    break;
            }
            
            
        }
        
        // trigger section
        for(int i =0; i<valid_trgs; i++) {
            epicsFloat64 width; getDoubleParam((p_trigger_st+i)->p_width[1], &width);
            // uint32_t ticks = (width*1.E-3 * application_clock_2) + 0.5;
            uint32_t ticks, taps;
            tick(width, application_clock_2, &ticks);
            if(!ticks) ticks = 1;
            pApiDrv->SetWidth(i, ticks); setIntegerParam((p_trigger_st+i)->p_widthTicks, ticks);
            PropagateTWID(i, width);
            
            epicsFloat64 delay; getDoubleParam((p_trigger_st+i)->p_delay[1], &delay);
            // ticks = (delay*1.E-3 * application_clock_2) + 0.5;
            if(busType == _pcie) {
                tick(delay, application_clock_2, &ticks, &taps);
                pApiDrv->SetDelayTap(i, taps); setIntegerParam((p_trigger_st+i)->p_delayTaps, taps);
            } else {
                tick(delay, application_clock_2, &ticks);
            }
            pApiDrv->SetDelay(i, ticks); setIntegerParam((p_trigger_st+i)->p_delayTicks, ticks);
            PropagateTDES(i, delay);

            int _enable; getIntegerParam((p_trigger_st +i)->p_enable[1], &_enable);
            PropagateTCTL(i, _enable); 
        }

        epicsInt32 _ued_support;
        getIntegerParam(p_ued_special, &_ued_support);
        if(!_ued_support) {       
            /* back ward compatible mode */
            pApiDrv->SetClkSel(1); /* set clock for LCLS2 */
            pApiDrv->SetModeSel(1); /* set protocol mode for LCLS2 */
        }  else {
            /* UED support */
            pApiDrv->SetClkSel(0);  /* use LCLS1 clock rate */
            pApiDrv->SetModeSel(1); /* use LCLS1 protocol */
        }
        
    }
    
    
    
    // re-enable channels and trigger with latched vlaues
    for(int i =0; i <valid_chns; i++) {
        epicsInt32 _enable; getIntegerParam((p_channel_st + i)->p_enable[mode], &_enable);
        pApiDrv->ChannelEnable(i, (uint32_t) _enable);
    }
    
    for(int i = 0; i <valid_trgs; i++) {
        epicsInt32 _enable; getIntegerParam((p_trigger_st +i)->p_enable[mode], &_enable);
        pApiDrv->TriggerEnable(i, (uint32_t) _enable);
    }
 
}

void tprTriggerAsynDriver::SetMsgDelay(epicsFloat64 msg_delay)
{
    // uint32_t ticks = (msg_delay * 1.E-3 * lcls2_clock) + 0.5;
    uint32_t ticks;

    tick(msg_delay, lcls2_clock, &ticks);
    pApiDrv->SetMsgDelay(ticks);

    setIntegerParam(p_msg_delay_ticks, (epicsInt32) ticks);
    
}

void tprTriggerAsynDriver::SetMasterDelay(epicsFloat64 master_delay)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode ==1) return;   // nothing todo in LCLS2 mode, just latch set values into parameter space
    
    for(int i=0; i<valid_trgs; i++) {
        epicsFloat64 delay; getDoubleParam((p_trigger_st+i)->p_delay[0], &delay);
        // uint32_t ticks = ((master_delay + delay) * 1.E-3 * application_clock_1) + 0.5;
        uint32_t ticks, taps;
        if(busType == _pcie) {
            tick(master_delay + delay, application_clock_1, &ticks); taps = 0;
            pApiDrv->SetDelayTap(i, taps); setIntegerParam((p_trigger_st+i)->p_delayTaps, taps);
        } else {
            tick(master_delay + delay, application_clock_1, &ticks);
        }
        pApiDrv->SetDelay(i, ticks); setIntegerParam((p_trigger_st +i)->p_delayTicks, ticks);
    }
}

void tprTriggerAsynDriver::SetLCLS1ChannelEnable(int channel, epicsInt32 enable)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode ==0)  /* apply channel status in LCLS1 mode */
        pApiDrv->ChannelEnable(channel, enable);
    else return;  /* nothing todo in LCLS1 mode */
}

void tprTriggerAsynDriver::SetLCLS2ChannelEnable(int channel, epicsInt32 enable)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode == 0)  return;     /* nothing todo in LCLS1 mode */
    else
        pApiDrv->ChannelEnable(channel, enable);    /* apply channel status in LCLS2 mode */
}


void tprTriggerAsynDriver::SetRateMode(int channel, epicsInt32 rate_mode)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode == 0) return; // nothing todo in LCLS1 mode, just latch rate mode into parameter space
    
    switch(rate_mode) {
        case 0: /* fixed mode */
            epicsInt32 fixed_rate; getIntegerParam((p_channel_st + channel)->p_fixed_rate, &fixed_rate);
            pApiDrv->SetFixedRate(channel, (uint32_t) fixed_rate);
            break;
        case 1: /* AC mode */
            epicsInt32 ac_rate; getIntegerParam((p_channel_st + channel)->p_ac_rate, &ac_rate);
            epicsInt32 ts_mask; getIntegerParam((p_channel_st + channel)->p_ts_mask, &ts_mask);
            pApiDrv->SetACRate(channel, (uint32_t) ts_mask, (uint32_t) ac_rate);
            break;
        case 2: /* Seq mode */
            epicsInt32 seq_num; getIntegerParam((p_channel_st + channel)->p_seq_num, &seq_num);
            epicsInt32 seq_bit; getIntegerParam((p_channel_st + channel)->p_seq_bit, &seq_bit);
            pApiDrv->SetSeqBit(channel, (uint32_t) (((seq_num & 0x1f)<<4)|(seq_bit & 0xf)));
            break;
    }
}


void tprTriggerAsynDriver::SetFixedRate(int channel, epicsInt32 fixed_rate)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode == 0) return; // nothing todo in CLLS1 mode, just latch set values into parameter space
    
    epicsInt32 rate_mode; getIntegerParam((p_channel_st + channel)->p_rate_mode, &rate_mode);
    if(rate_mode != 0) return; // nothing todo without Fixed Rate mode, just latch set values into parameter space
    
    pApiDrv->SetFixedRate(channel, (uint32_t) fixed_rate);
}


void tprTriggerAsynDriver::SetACRate(int channel, epicsInt32 ac_rate)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode ==0) return; // nothing todo in LCLS1 mode, just latch set values into parameter space
    
    epicsInt32 rate_mode; getIntegerParam((p_channel_st + channel)->p_rate_mode, &rate_mode);
    if(rate_mode != 1) return; // nothing todo without AC Rate mode, just latch set values into parameter space 
    
    epicsInt32 ts_mask; getIntegerParam((p_channel_st + channel)->p_ts_mask, &ts_mask);
    pApiDrv->SetACRate(channel, (uint32_t) ts_mask, (uint32_t) ac_rate);
    
}


void tprTriggerAsynDriver::SetTSMask(int channel, epicsInt32 ts_mask)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode == 0) return;  // nothing todo in LCLS1 mode, just latch set values into parameter space
    
    epicsInt32 rate_mode; getIntegerParam((p_channel_st + channel)->p_rate_mode, &rate_mode);
    if(rate_mode != 1) return;  // nothing todo without AC Rate mode, just latch set values into parameter space
    
    epicsInt32 ac_rate; getIntegerParam((p_channel_st + channel)->p_ac_rate, &ac_rate);
    pApiDrv->SetACRate(channel, (uint32_t) ts_mask, (uint32_t) ac_rate);
}


void tprTriggerAsynDriver::SetSeqBit(int channel, epicsInt32 seq_bit)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode == 0) return;   // nothing todo in LCLS1 mode, just latch set values into parameter space
    
    epicsInt32 rate_mode; getIntegerParam((p_channel_st+ channel)->p_rate_mode, &rate_mode);
    if(rate_mode != 2) return;    // nothing todo without Seq mode, just latch set value into parameter space 
    
    pApiDrv->SetSeqBit(channel, (uint32_t) seq_bit);
    
}


void tprTriggerAsynDriver::SetDestMode(int channel, epicsInt32 dest_mode)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode == 0 ) return;   // nothing todo in LCLS1 mode, just latch set values into parameter space 
    
    epicsInt32 dest_mask; getIntegerParam((p_channel_st + channel)->p_dest_mask, &dest_mask);
    
    switch(dest_mode) {
        case 0:  // inclusive
            pApiDrv->SetInclusionMask(channel, (uint32_t) dest_mask);
            break;
        case 1:  // exclusive
            pApiDrv->SetExclusionMask(channel, (uint32_t) dest_mask);
            break;
        case 2:  // don't care
            pApiDrv->SetDontCareMask(channel);
            break;
    
    }
    
}


void tprTriggerAsynDriver::SetDestMask(int channel, epicsInt32 dest_mask)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode == 0) return;    // nothing doto in LCLS1 mode, just latch set values into parameter space
    
    epicsInt32 dest_mode; getIntegerParam((p_channel_st + channel)->p_dest_mode, &dest_mode);
    
    switch(dest_mode) {
        case 0:    // inclusive
            pApiDrv->SetInclusionMask(channel, (uint32_t) dest_mask);
            break;
        case 1:    // exclusive
            pApiDrv->SetExclusionMask(channel, (uint32_t) dest_mask);
            break;
        case 2:    // don't care
            pApiDrv->SetDontCareMask(channel);
            break;
    
    }
}


void tprTriggerAsynDriver::SetEventCode(int channel, epicsInt32 event_code)
{
    int mode;
    getIntegerParam(p_mode, &mode);
    
    if(mode==0) { /* LCLS1 */
        pApiDrv->SetEventCode(channel, event_code);
    }
    else { /*LCLS2 */
        // nothing todo in LCLS2 mode, just latch the event code into parameter space
        return;
    }
}
        
        
void tprTriggerAsynDriver::SetLCLS1TriggerEnable(int trigger, epicsInt32 enable)
{
    int mode; getIntegerParam(p_mode, &mode);
    mode = !mode?0:1;
    
    if(mode == 0) {    /* set trigger status in LCLS1 mode */ 
        pApiDrv->TriggerEnable(trigger, (uint32_t) enable);
        PropagateTCTL(trigger, enable);
    } else             /* nothing todo in LCLS2 mode */
        return;
}

void tprTriggerAsynDriver::SetLCLS2TriggerEnable(int trigger, epicsInt32 enable)
{
    int mode; getIntegerParam(p_mode, &mode);
    mode = !mode?0:1;
    
    if(mode == 0)    /* nothing doto in LCLS1 mode */
        return;
    else {            /* set trigger status in LCLs2 mode */
        pApiDrv->TriggerEnable(trigger, (uint32_t) enable);
        PropagateTCTL(trigger, enable);
    }
        
}
void tprTriggerAsynDriver::SetSource(int trigger, epicsInt32 source)
{
    pApiDrv->SetSourceMask(trigger, (uint32_t) source);
}


void tprTriggerAsynDriver::SetPolarity(int trigger, epicsInt32 polarity)
{
    pApiDrv->SetPolarity(trigger, (uint32_t) polarity);
    PropagateTPOL(trigger, polarity);
}

void tprTriggerAsynDriver::SetCmpl(int trigger, epicsInt32 cmpl)
{
    pApiDrv->SetComplTrg(trigger, (uint32_t) cmpl);
}

void tprTriggerAsynDriver::SetLCLS1Delay(int trigger, epicsFloat64 delay)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode == 1) return;   // nothing todo in LCLS2 mode, just latch set values into parameter space
    
    epicsFloat64 master_delay; getDoubleParam(p_master_delay, &master_delay);
    // uint32_t ticks  = ((master_delay + delay) * 1.E-3 * application_clock_1) + 0.5;
    uint32_t ticks, taps;
    if(busType == _pcie) {
        tick(master_delay + delay, application_clock_1, &ticks); taps = 0;
        pApiDrv->SetDelayTap(trigger, taps); setIntegerParam((p_trigger_st + trigger)->p_delayTaps, taps);
    } else {
        tick(master_delay + delay, application_clock_1, &ticks);
    }
    pApiDrv->SetDelay(trigger, ticks); setIntegerParam((p_trigger_st + trigger)->p_delayTicks, ticks);

    PropagateTDES(trigger, delay);

}
void tprTriggerAsynDriver::SetLCLS2Delay(int trigger, epicsFloat64 delay)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode == 0) return;   // nothing todo in LCLS1 mode, just latch set values into parameter space
    
    // uint32_t ticks = (delay*1.E-3 * application_clock_2) + 0.5;
    uint32_t ticks, taps;
    if(busType == _pcie) {
        tick(delay, application_clock_2, &ticks, &taps);
        pApiDrv->SetDelayTap(trigger, taps); setIntegerParam((p_trigger_st + trigger)->p_delayTaps, taps);
    } else {
        tick(delay, application_clock_2, &ticks);
    }
    pApiDrv->SetDelay(trigger, ticks); setIntegerParam((p_trigger_st + trigger)->p_delayTicks, ticks);

    PropagateTDES(trigger, delay);

}
void tprTriggerAsynDriver::SetLCLS1Width(int trigger, epicsFloat64 width)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode == 1) return;  // nothing todo in LCLS2 mode, just latch set values into parameter space
    
    // uint32_t ticks = (width*1.E-3 * application_clock_1) + 0.5;
    uint32_t ticks;
    tick(width, application_clock_1, &ticks);
    if(!ticks) ticks =1;
    pApiDrv->SetWidth(trigger, ticks); setIntegerParam((p_trigger_st + trigger)->p_widthTicks, ticks);

    PropagateTWID(trigger, width);

}
void tprTriggerAsynDriver::SetLCLS2Width(int trigger, epicsFloat64 width)
{
    int mode; getIntegerParam(p_mode, &mode);
    if(mode ==0) return;    // nothing todo in LCLS1 mode, just latch set values into parameter space
    
    // uint32_t ticks = (width*1.E-3 * application_clock_2) + 0.5;
    uint32_t ticks;
    tick(width, application_clock_2, &ticks);
    if(!ticks) ticks =1;
    pApiDrv->SetWidth(trigger, ticks); setIntegerParam((p_trigger_st + trigger)->p_widthTicks, ticks);

    PropagateTWID(trigger, width);

}


void tprTriggerAsynDriver::PropagateTDES(int trigger, epicsFloat64 delay)
{
    epicsFloat64 tdes;
    getDoubleParam((p_trigger_st + trigger)->p_tdes, &tdes);

    // to avoid udf on TDES PV at the beginning
    if(tdes == 0.0) {
        setDoubleParam((p_trigger_st + trigger)->p_prop_tdes, -1.);
        setDoubleParam((p_trigger_st + trigger)->p_prop_tdes, delay);
        return;
    }

    if(tdes != delay) setDoubleParam((p_trigger_st + trigger)->p_prop_tdes, delay);
}

void tprTriggerAsynDriver::PropagateTWID(int trigger, epicsFloat64 width)
{
    epicsFloat64 twid;
    getDoubleParam((p_trigger_st + trigger)->p_twid, &twid);
    // to avoid udf on TWID PV at the beginning
    if(twid == 0.0) {
        setDoubleParam((p_trigger_st + trigger)->p_prop_twid, -1.);
        setDoubleParam((p_trigger_st + trigger)->p_prop_twid, width);
        return;
    }

    if(twid != width) setDoubleParam((p_trigger_st + trigger)->p_prop_twid, width);
}


void tprTriggerAsynDriver::PropagateTPOL(int trigger, epicsInt32 polarity)
{
    epicsInt32 tpol;
    getIntegerParam((p_trigger_st + trigger)->p_tpol, &tpol);
    // to avoid udf on TPOL PV at the beginning
    if(tpol == 0) {
        setIntegerParam((p_trigger_st + trigger)->p_prop_tpol, -1);
        setIntegerParam((p_trigger_st + trigger)->p_prop_tpol, polarity);
        return;
    }
    if(tpol != polarity) setIntegerParam((p_trigger_st + trigger)->p_prop_tpol, polarity);
}

void tprTriggerAsynDriver::PropagateTCTL(int trigger, epicsInt32 enable)
{
    epicsInt32 tctl;
    getIntegerParam((p_trigger_st + trigger)->p_tctl, &tctl);
    // to avoid udf on TCTL PV at the beginning
    if(tctl == 0) {
        setIntegerParam((p_trigger_st + trigger)->p_prop_tctl, -1);
        setIntegerParam((p_trigger_st + trigger)->p_prop_tctl, enable);
        return;
    }

    if(tctl != enable) setIntegerParam((p_trigger_st + trigger)->p_prop_tctl, enable);
}

void tprTriggerAsynDriver::PropagateDelay(int trigger, epicsFloat64 tdes)
{
    int mode; getIntegerParam(p_mode, &mode);
    epicsFloat64 delay; getDoubleParam((p_trigger_st + trigger)->p_delay[mode], &delay);

    if(delay != tdes) setDoubleParam((p_trigger_st + trigger)->p_prop_delay[mode], tdes);
}

void tprTriggerAsynDriver::PropagateWidth(int trigger, epicsFloat64 twid)
{
    int mode; getIntegerParam(p_mode, &mode);
    epicsFloat64 width; getDoubleParam((p_trigger_st + trigger)->p_width[mode], &width);

    if(width != twid) setDoubleParam((p_trigger_st + trigger)->p_prop_width[mode], twid);
}


void tprTriggerAsynDriver::PropagatePolarity(int trigger, epicsInt32 tpol)
{
    epicsInt32 polarity; getIntegerParam((p_trigger_st + trigger)->p_polarity, &polarity);
  
    if(polarity != tpol) setIntegerParam((p_trigger_st + trigger)->p_prop_polarity, tpol);
}

void tprTriggerAsynDriver::PropagateEnable(int trigger, epicsInt32 tctl)
{
    int mode; getIntegerParam(p_mode, &mode);
    epicsInt32 enable; getIntegerParam((p_trigger_st + trigger)->p_enable[mode], &enable);

    if(enable != tctl) setIntegerParam((p_trigger_st + trigger)->p_prop_enable[mode], tctl);
}


void tprTriggerAsynDriver::SetUedSpecialMode(int mode)
{
    if(mode) {    // ued_special mode
        pApiDrv->SetModeSelEn(1);    // enable independant clock selection and timing decoding
        pApiDrv->SetModeSel(1);      // make timing decoding to LCLS2 style
        pApiDrv->SetClkSel(0);       // use LCLS1 timing rate
    }
    else {        // backward compatible
        pApiDrv->SetModeSelEn(0);    // unset ModeSelEn register for backward compatible mode
    }
}



extern "C" {

static void tprTriggerAsynDriverConfigure(const char *port_name, const char *core_path, const char *named_root)
{
    tprTriggerDrvList_t    *pDrvNode = new tprTriggerDrvList_t; 
    pDrvNode->pAsynDrv               = new tprTriggerAsynDriver(port_name, core_path, named_root);
    pDrvNode->pApiDrv                = pDrvNode->pAsynDrv->getApiDrv();
    
    
    strcpy(pDrvNode->port_name, port_name);
    strcpy(pDrvNode->core_path, core_path);
    pDrvNode->named_root = (named_root && strlen(named_root))? epicsStrDup(named_root):cpswGetRootName();
    
    ellAdd(pList, &pDrvNode->node);
    
     
}


static void tprTriggerAsynDriverDebug(const char *port_name, const int debug_flag)
{
    tprTriggerDrvList_t   *p;
    
    if(!ellCount(pList)) {
        printf("tprTriggerAsynDriverDebug: not found driver\n");
        return;
    }
    
    p = (tprTriggerDrvList_t *) ellFirst(pList);
    while (p) {
        if(!strcmp(p->port_name, port_name)) break;
        p = (tprTriggerDrvList_t *) ellNext(&p->node);
    }
    
    if(!p) {
        printf("tprTriggerAsynDriverDebug: not found port_name (%s)\n", 
               port_name);
        return;
    }
    
    p->pAsynDrv->SetDebug(debug_flag);
}



static void tprTriggerAsynDriverMonitor(void)
{
    tprTriggerDrvList_t *p;
    
    if(!ellCount(pList)) return;
    
    while(1) {
        p = (tprTriggerDrvList_t*) ellFirst(pList);
        while(p) {
            p->pAsynDrv->Monitor();
            p = (tprTriggerDrvList_t*) ellNext(&p->node);
        }
        epicsThreadSleep(1.);
    }
}


static int tprTriggerAsynDriverReport(int interest);
static int tprTriggerAsynDriverInitialize(void);
static struct drvet tprTriggerAsynDriver = {
    2,
    (DRVSUPFUN) tprTriggerAsynDriverReport,
    (DRVSUPFUN) tprTriggerAsynDriverInitialize
};

epicsExportAddress(drvet, tprTriggerAsynDriver);


static int tprTriggerAsynDriverReport(int interest)
{
    tprTriggerDrvList_t *p;

    printf("Total %d tprTriggerAsyn driver(s) registered.\n", ellCount(pList));
       
    //API_TEST();
    
    if(!ellCount(pList)) return 0;
    
    p = (tprTriggerDrvList_t *) ellFirst(pList);
    while (p) {
        printf("named_root   : %s\n", p->named_root);
        printf("    port name: %s\n", p->port_name);
        printf("    core path: %s\n", p->core_path);
        printf("    api location: %p\n", p->pApiDrv);
        printf("    drv location: %p\n", p->pAsynDrv);
        if(interest>2) {
        printf("    report from api\n");
        p->pApiDrv->report();
        }
    
        p = (tprTriggerDrvList_t *)ellNext(&p->node);
    }
    
    

    return 0;
}

static int tprTriggerAsynDriverInitialize(void)
{
    init_pList();

    if(!ellCount(pList)) return 0;
    
    epicsThreadCreate("tprTriggerMon", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) tprTriggerAsynDriverMonitor, 0);
    
    return 0;
}


static const iocshArg initArg0 = { "port name", iocshArgString };
static const iocshArg initArg1 = { "core path", iocshArgString };
static const iocshArg initArg2 = { "named_root (optional)", iocshArgString };
static const iocshArg * const initArgs[] = { &initArg0,
                                             &initArg1,
                                             &initArg2 };
static const iocshFuncDef initFuncDef = { "tprTriggerAsynDriverConfigure", 3, initArgs };
static void initCallFunc(const iocshArgBuf *args)
{
    init_pList();
    tprTriggerAsynDriverConfigure(args[0].sval, args[1].sval,
                                  (args[2].sval && strlen(args[2].sval))? args[2].sval: NULL);
    
}

static const iocshArg debugArg0 = { "port name", iocshArgString};
static const iocshArg debugArg1 = { "debug flag", iocshArgInt};
static const iocshArg * const debugArgs[] = { &debugArg0, 
                                              &debugArg1 };
static const iocshFuncDef debugFuncDef = { "tprTriggerAsynDriverDebug", 2, debugArgs };
static void debugCallFunc(const iocshArgBuf *args)
{
    init_pList();
    tprTriggerAsynDriverDebug(args[0].sval, args[1].ival);
}


void tprTriggerAsynDriverRegister(void)
{
    iocshRegister(&initFuncDef,  initCallFunc);
    iocshRegister(&debugFuncDef, debugCallFunc);
}

epicsExportRegistrar(tprTriggerAsynDriverRegister);




}  /* extern "C" */
