#include <cpsw_api_builder.h>
#include <cpsw_api_user.h>

#include <tprTriggerYaml.hh>

#include <stdio.h>
#include <string.h>

using namespace Tpr;

TprTriggerYaml::TprTriggerYaml(Path core)
{
    _path               = core;
    _path_axiVersion    = _path->findByName("AxiVersion");
    _path_axiSy56040    = _path->findByName("AxiSy56040");
    _path_timingFrameRx = _path->findByName("AmcCarrierTiming/TimingFrameRx");

    for(int i = 0; i < NUM_CHANNELS; i++) {
        char path_name[128];
        sprintf(path_name, "AmcCarrierTiming/EvrV2CoreTriggers/EvrV2ChannelReg[%d]", i);
        _path_evrV2ChnReg[i] = _path->findByName(path_name);
    }

    for(int i = 0; i < NUM_TRIGGERS; i++) {
        char path_name[128];
        sprintf(path_name, "AmcCarrierTiming/EvrV2CoreTriggers/EvrV2TriggerReg[%d]", i);
        _path_evrV2TrgReg[i] = _path->findByName(path_name);
    }


    _fpgaVersion    = IScalVal_RO::create(_path_axiVersion->findByName("FpgaVersion"));
    _upTimeCnt      = IScalVal_RO::create(_path_axiVersion->findByName("UpTimeCnt"));

    for(int i = 0; i < 4; i++) {
        char path_name[32];
        sprintf(path_name, "OutputConfig[%i]", i);
        _outputConfig[i] = IScalVal::create(_path_axiSy56040->findByName(path_name));
    }

    _sofCount      = IScalVal_RO::create(_path_timingFrameRx->findByName("sofCount"));
    _eofCount      = IScalVal_RO::create(_path_timingFrameRx->findByName("eofCount"));
    _fidCount      = IScalVal_RO::create(_path_timingFrameRx->findByName("FidCount"));
    _crcErrCount   = IScalVal_RO::create(_path_timingFrameRx->findByName("CrcErrCount"));
    _rxClkCount    = IScalVal_RO::create(_path_timingFrameRx->findByName("RxClkCount"));
    _rxRstCount    = IScalVal_RO::create(_path_timingFrameRx->findByName("RxRstCount"));
    _rxDecErrCount = IScalVal_RO::create(_path_timingFrameRx->findByName("RxDecErrCount"));
    _rxDspErrCount = IScalVal_RO::create(_path_timingFrameRx->findByName("RxDspErrCount"));
    _rxCountReset  = IScalVal   ::create(_path_timingFrameRx->findByName("RxCountReset"));
    _rxLinkUp      = IScalVal_RO::create(_path_timingFrameRx->findByName("RxLinkUp"));
    _rxPolarity    = IScalVal   ::create(_path_timingFrameRx->findByName("RxPolarity"));
    _rxReset       = IScalVal   ::create(_path_timingFrameRx->findByName("RxReset"));
    _clkSel        = IScalVal   ::create(_path_timingFrameRx->findByName("ClkSel"));
    _rxDown        = IScalVal   ::create(_path_timingFrameRx->findByName("RxDown"));
    _bypassRst     = IScalVal   ::create(_path_timingFrameRx->findByName("BypassRst"));
    _versionErr    = IScalVal_RO::create(_path_timingFrameRx->findByName("VersionErr"));
    _msgNoDelay    = IScalVal   ::create(_path_timingFrameRx->findByName("MsgNoDelay"));
    _msgDelay      = IScalVal   ::create(_path_timingFrameRx->findByName("MsgDelay"));
    _txClkCount    = IScalVal_RO::create(_path_timingFrameRx->findByName("TxClkCount"));
    _bypassDoneCount  = IScalVal_RO::create(_path_timingFrameRx->findByName("BypassDoneCount"));
    _bypassResetCount = IScalVal_RO::create(_path_timingFrameRx->findByName("BypassResetCount"));
    _frameVersion     = IScalVal_RO::create(_path_timingFrameRx->findByName("FrameVersion"));
    
    for(int i = 0; i < NUM_CHANNELS; i++) {
        _chnEnable[i]         = IScalVal   ::create(_path_evrV2ChnReg[i]->findByName("Enable"));
        _chnBsaEnable[i]      = IScalVal   ::create(_path_evrV2ChnReg[i]->findByName("BsaEnabled"));
        _chnDmaEnable[i]      = IScalVal   ::create(_path_evrV2ChnReg[i]->findByName("DmaEnabled"));
        _chnRateSelect[i]     = IScalVal   ::create(_path_evrV2ChnReg[i]->findByName("RateSel"));
        _chnDestSelect[i]     = IScalVal   ::create(_path_evrV2ChnReg[i]->findByName("DestSel"));
        _chnCount[i]          = IScalVal_RO::create(_path_evrV2ChnReg[i]->findByName("Count"));
        _chnBsaWindowDelay[i] = IScalVal   ::create(_path_evrV2ChnReg[i]->findByName("BsaWindowDelay"));
        _chnBsaWindowSetup[i] = IScalVal   ::create(_path_evrV2ChnReg[i]->findByName("BsaWindowSetup"));
        _chnBsaWindowWidth[i] = IScalVal   ::create(_path_evrV2ChnReg[i]->findByName("BsaWindowWidth"));   
    }
    
    for(int i = 0; i < NUM_TRIGGERS; i++) {
        _trgEnable[i]           = IScalVal   ::create(_path_evrV2TrgReg[i]->findByName("Enable"));
        _trgSourceMask[i]       = IScalVal   ::create(_path_evrV2TrgReg[i]->findByName("Source"));
        _trgPolarity[i]         = IScalVal   ::create(_path_evrV2TrgReg[i]->findByName("Polarity"));
        _trgDelay[i]            = IScalVal   ::create(_path_evrV2TrgReg[i]->findByName("Delay"));
        _trgWidth[i]            = IScalVal   ::create(_path_evrV2TrgReg[i]->findByName("Width"));
        _trgDelayTap[i]         = IScalVal   ::create(_path_evrV2TrgReg[i]->findByName("DelayTap"));
        _trgDelayTapReadback[i] = IScalVal_RO::create(_path_evrV2TrgReg[i]->findByName("DelayTapReadback"));
    }


}


void TprTriggerYaml::InitLCLS1Mode(void)
{
     uint32_t zero(0), one(1), dontcare(0x20000);
    _clkSel->setVal(&zero);         /* Select LCLS1 mode clock */
    
    for(uint32_t i = 0; i < NUM_CHANNELS; i++) {
        _chnDestSelect[i]->setVal(&dontcare);  /* destination, don't care */
        _trgSourceMask[i]->setVal(&i);         /* one to one mapping between channel and trigger */
    }
}

void TprTriggerYaml::InitLCLS2Mode(void)
{
    uint32_t zero(0), one(1);
    _clkSel->setVal(&one);      /* Select LCLS2 mode clock */
    
    
}

void TprTriggerYaml::SetClkSel(uint32_t clock_mode)
{
    uint32_t zero(0),one(1);
    _clkSel->setVal(clock_mode? &one:&zero);
    
    if(_debug_) printf("TprTriggerYaml (%p): set cls_sel (%8.8x)\n", this, clock_mode?one:zero);
}


void TprTriggerYaml::SetMsgDelay(uint32_t delay_ticks)
{
    _msgDelay->setVal(&delay_ticks);
    if(_debug_) printf("TprTriggerYaml (%p): set msg delay (%8.8x) %lu\n", this, delay_ticks, delay_ticks);
}

void TprTriggerYaml::ChannelEnable(int channel, uint32_t enable)
{
    uint32_t zero(0), one(1);
    
    _chnEnable[channel]->setVal(enable?&one:&zero);
    
    if(_debug_) printf("TprTriggerYaml (%p): set channel enable (chn %x, enable %x)\n", this, channel, enable?one:zero); 
    
}

void TprTriggerYaml::SetEventCode(int channel, uint32_t event)
{
    uint32_t  _dontcare(0x20000);
    _chnDestSelect[channel]->setVal(&_dontcare);
    uint32_t _event = (0x00000002<<11) | (0x000003ff & event);
    _chnRateSelect[channel]->setVal(&_event);
    
    if(_debug_) {
        printf("TprTriggerYaml (%p): destination select (chn %x, dest sel %8.8x)\n", this, channel, _dontcare);
        printf("TprTriggerYaml (%p): rate select(event code) (chn %x, event code %8.8x)\n", this, channel, _event);
    }
}

void TprTriggerYaml::SetFixedRate(int channel, uint32_t rate)
{
    uint32_t _rate = (0x000000<<11) | (0x0000000f & rate);
    _chnRateSelect[channel]->setVal(&_rate);
    
    if(_debug_) printf("TprTriggerYaml (%p): rate select (fixed rate) (chn %x, rate select %8.8x)\n", this, channel, _rate);
}

void TprTriggerYaml::SetACRate(int channel, uint32_t ts_mask, uint32_t rate)
{
    uint32_t _rate = (0x00000001<<11) | (0x0000003f & ts_mask)<< 3 | (0x00000007 & rate);
    _chnRateSelect[channel]->setVal(&_rate);
    
    if(_debug_) printf("TprTriggerYaml (%p): rate select (AC rate) (chn %x, rate select %8.8x)\n", this, channel, _rate);
}

void TprTriggerYaml::SetSeqBit(int channel, uint32_t seq_bit)
{
    uint32_t _seq_bit = (0x00000002<<11) | (0x000003ff & seq_bit);
    _chnRateSelect[channel]->setVal(&_seq_bit);
    
    if(_debug_) printf("TprTriggerYaml (%p): rate select (Seq Bit) (chn %x, rate select %8.8x)\n", this, channel, _seq_bit);
}

void TprTriggerYaml::SetInclusionMask(int channel, uint32_t dest_mask)
{
    uint32_t _mask = (0x00000000<<16) | (0x0000ffff & dest_mask);
    _chnDestSelect[channel]->setVal(&_mask);
    
    if(_debug_) printf("TprTriggerYaml (%p): dest select (inclusion) (chn %x, dest select %8.8x)\n", this, channel, _mask);
}

void TprTriggerYaml::SetExclusionMask(int channel, uint32_t dest_mask)
{
    uint32_t _mask = (0x00000001<<16) | (0x0000ffff & dest_mask);
    _chnDestSelect[channel]->setVal(&_mask);
    
    if(_debug_) printf("TprTriggerYaml (%p): dest select (excusion) (chn %x, dest select %8.8x)\n", this, channel, _mask);
}

void TprTriggerYaml::SetDontCareMask(int channel)
{
    uint32_t _mask = (0x00000002<<16);
    _chnDestSelect[channel]->setVal(&_mask);
    
    if(_debug_) printf("TprTriggerYaml (%p): dest select (dont care) (chn %x, dest select %8.8x)\n", this, channel, _mask);
}

void TprTriggerYaml::TriggerEnable(int trigger, uint32_t enable)
{
    uint32_t zero(0), one(1);
    
    _trgEnable[trigger]->setVal(enable?&one:&zero);
    
    if(_debug_) printf("TprTriggerYaml (%p): trigger enable (trg %x, enable %x)\n", this, trigger, enable?one:zero);
}

void TprTriggerYaml::SetSourceMask(int trigger, uint32_t source)
{
    //uint32_t _source = 0x00000001 << source;
    uint32_t _source = 0x0000000f & source;
    
    _trgSourceMask[trigger]->setVal(&_source);
    
    if(_debug_) printf("TprTriggerYaml (%p): source (trg %x, source %x)\n", this, trigger, _source);
}

void TprTriggerYaml::SetPolarity(int trigger, uint32_t polarity)
{
    uint32_t falling(0), rising(1);
    
    _trgPolarity[trigger]->setVal(!polarity?&falling:&rising);
    
    if(_debug_) printf("TprTriggerYaml (%p): polarity (trg %x, prolarity %x)\n", this, trigger, !polarity?falling:rising);
}

void TprTriggerYaml::SetDelay(int trigger, uint32_t delay_ticks)
{
    _trgDelay[trigger]->setVal(&delay_ticks);
    
    if(_debug_) printf("TprTriggerYaml (%p): delay (trg %x, delay %8.8x) %lu\n", this, trigger, delay_ticks, delay_ticks);
}

void TprTriggerYaml::SetWidth(int trigger, uint32_t width_ticks)
{
    _trgWidth[trigger]->setVal(&width_ticks);
    
    if(_debug_) printf("TprTriggerYaml (%p): width (trg %x, width %8.8x) %lu\n", this, trigger, width_ticks, width_ticks);
}  


void TprTriggerYaml::report(void)
{
    printf("\tFPGA version: %8.8x\n", fpgaVersion()); 
    printf("\tUpTime count: %8.8x\n", upTimeCount());
    printf("\tStart of Frame count: %8.8x\n", sofCount());
    printf("\tEnd of Frame count:   %8.8x\n", eofCount());
    printf("\tFiducial count:       %8.8x\n", fidCount());
    printf("\tCRC Error count:      %8.8x\n", crcErrCount());
    printf("\tRX Clock count:       %8.8x\n", rxClkCount());
    printf("\tRX Link Reset count:  %8.8x\n", rxRstCount());
    printf("\tRX 8b/10b Decode Error count: %8.8x\n", rxDecErrCount());
    printf("\tRX Disparity Error count:     %8.8x\n", rxDspErrCount());
    printf("\tRX Link Up status:    %s\n", rxLinkStatus()?"Up":"Down");
    printf("\tFrame Version Error:  %s\n", versionErr()?"Error":"Good");
    printf("\tFrame Version:        %8.8x\n", frameVersion());
    
    for(int i = 0; i < NUM_CHANNELS; i++) {
        printf("\tChannel [%x]\n", i);
        printf("\t\tEnable:             %8.8x\n", channelEnable(i));
        printf("\t\tRate Select:        %8.8x\n", channelRateSelect(i));
        printf("\t\tDestination Select: %8.8x\n", channelDestSelect(i));
        printf("\t\tChannel Count:      %8.8x\n", channelCount(i));  
    }
    
    for(int i =0; i < NUM_TRIGGERS; i++) {
        printf("\tTrigger [%x]\n", i);
        printf("\t\tEnable:             %8.8x\n", triggerEnable(i));
        printf("\t\tSource Mask:        %8.8x\n", triggerSourceMask(i));
        printf("\t\tPolarity:           %8.8x\n", triggerPolarity(i));
        printf("\t\tWidth:              %8.8x\n", triggerWidth(i));
        printf("\t\tDelay:              %8.8x\n", triggerDelay(i));
    }
    
}

uint32_t TprTriggerYaml::channelEnable(int channel)
{
    uint32_t        val;
    IndexRange      rng(0);
    
    _chnEnable[channel]->getVal(&val, 1,  &rng);
    
    return val;
}

uint32_t TprTriggerYaml::channelRateSelect(int channel)
{
    uint32_t        val;
    IndexRange      rng(0);
    
    _chnRateSelect[channel]->getVal(&val, 1, &rng);
    
    return val;
}

uint32_t TprTriggerYaml::channelDestSelect(int channel)
{
    uint32_t        val;
    IndexRange      rng(0);
    
    _chnDestSelect[channel]->getVal(&val, 1, &rng);
    
    return val;
}

uint32_t TprTriggerYaml::channelCount(int channel)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _chnCount[channel]->getVal(&val, 1, &rng);
    
    return val;
}

uint32_t TprTriggerYaml::triggerEnable(int trigger)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _trgEnable[trigger]->getVal(&val, 1, &rng);
    
    return val;
}

uint32_t TprTriggerYaml::triggerSourceMask(int trigger)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _trgSourceMask[trigger]->getVal(&val, 1, &rng);
    
    return val;
}

uint32_t TprTriggerYaml::triggerPolarity(int trigger)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _trgPolarity[trigger]->getVal(&val, 1, &rng);
    
    return val;

}

uint32_t TprTriggerYaml::triggerWidth(int trigger)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _trgWidth[trigger]->getVal(&val, 1, &rng);
    
    return val;
}

uint32_t TprTriggerYaml::triggerDelay(int trigger)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _trgDelay[trigger]->getVal(&val, 1, &rng);
    
    return val;
}


uint32_t TprTriggerYaml::fpgaVersion(void)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _fpgaVersion->getVal(&val,1,&rng);
    
    return val;
}

uint32_t TprTriggerYaml::upTimeCount(void)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _upTimeCnt->getVal(&val,1,&rng);
    
    return val;
}


uint32_t TprTriggerYaml::sofCount(void)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _sofCount->getVal(&val,1,&rng);
    
    return val;
}

uint32_t TprTriggerYaml::eofCount(void)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _eofCount->getVal(&val,1,&rng);
    
    return val;
}

uint32_t TprTriggerYaml::fidCount(void)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _fidCount->getVal(&val,1,&rng);
    
    return val;
}

uint32_t TprTriggerYaml::crcErrCount(void)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _crcErrCount->getVal(&val,1,&rng);
    
    return val;
}

uint32_t TprTriggerYaml::rxClkCount(void)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    
    _rxClkCount->getVal(&val,1,&rng);
    
    return val;
}

uint32_t TprTriggerYaml::rxRstCount(void)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _rxRstCount->getVal(&val,1,&rng);
    
    return val;
}

uint32_t TprTriggerYaml::rxDecErrCount(void)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _rxDecErrCount->getVal(&val,1,&rng);
    
    return val;
}

uint32_t TprTriggerYaml::rxDspErrCount(void)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _rxDspErrCount->getVal(&val,1,&rng);
    
    return val;
}

uint32_t TprTriggerYaml::rxLinkStatus(void)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _rxLinkUp->getVal(&val,1,&rng);
    
    return val;
}

uint32_t TprTriggerYaml::versionErr(void)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _versionErr->getVal(&val,1,&rng);
    
    return val;
}


uint32_t TprTriggerYaml::frameVersion(void)
{
    uint32_t      val;
    IndexRange    rng(0);
    
    _frameVersion->getVal(&val,1,&rng);
    
    return val;
}
