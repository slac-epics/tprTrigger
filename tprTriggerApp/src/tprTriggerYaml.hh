#ifndef TPR_TRIGGER_YAML_hh
#define TPR_TRIGGER_YAML_hh

#include <cpsw_api_builder.h>

#include <stdint.h>
#include <vector>


#define NUM_CHANNELS 16
#define NUM_TRIGGERS 16



namespace Tpr {

    class TprTriggerYaml {
        protected:
            /* required paths */
            Path _path;
            Path _path_axiVersion;
            Path _path_axiSy56040;
            Path _path_timingFrameRx;
            Path _path_evrV2ChnReg[NUM_CHANNELS];
            Path _path_evrV2TrgReg[NUM_TRIGGERS];

            /* registers for Axiversion */
            ScalVal_RO     _fpgaVersion;
            ScalVal_RO     _upTimeCnt;
            /* registers for AxiSy56040, Crossbar switch */ 
            ScalVal        _outputConfig[4];
            /* registers for TimeFrameRx */
            ScalVal_RO     _sofCount;
            ScalVal_RO     _eofCount;
            ScalVal_RO     _fidCount;
            ScalVal_RO     _crcErrCount;
            ScalVal_RO     _rxClkCount;
            ScalVal_RO     _rxRstCount;
            ScalVal_RO     _rxDecErrCount;
            ScalVal_RO     _rxDspErrCount;
            ScalVal        _rxCountReset;
            ScalVal_RO     _rxLinkUp;
            ScalVal        _rxPolarity;
            ScalVal        _rxReset;
            ScalVal        _clkSel;
            ScalVal        _rxDown;
            ScalVal        _bypassRst;
            ScalVal_RO     _versionErr;
            ScalVal        _msgNoDelay;
            ScalVal        _msgDelay;
            ScalVal_RO     _txClkCount;
            ScalVal_RO     _bypassDoneCount;
            ScalVal_RO     _bypassResetCount;
            ScalVal_RO     _frameVersion; 
            /* registers for Channels */
            ScalVal        _chnEnable[NUM_CHANNELS];
            ScalVal        _chnBsaEnable[NUM_CHANNELS];
            ScalVal        _chnDmaEnable[NUM_CHANNELS];
            ScalVal        _chnRateSelect[NUM_CHANNELS];
            ScalVal        _chnDestSelect[NUM_CHANNELS];
            ScalVal_RO     _chnCount[NUM_CHANNELS];
            ScalVal        _chnBsaWindowDelay[NUM_CHANNELS];
            ScalVal        _chnBsaWindowSetup[NUM_CHANNELS];
            ScalVal        _chnBsaWindowWidth[NUM_CHANNELS];
            /* registers for Triggers */
            ScalVal        _trgEnable[NUM_TRIGGERS];
            ScalVal        _trgSourceMask[NUM_TRIGGERS];
            ScalVal        _trgPolarity[NUM_TRIGGERS];
            ScalVal        _trgDelay[NUM_TRIGGERS];
            ScalVal        _trgWidth[NUM_TRIGGERS];
            ScalVal        _trgDelayTap[NUM_TRIGGERS];
            ScalVal_RO     _trgDelayTapReadback[NUM_TRIGGERS];
            


        public:
            int _debug_;
            TprTriggerYaml(Path core);
            void InitLCLS1Mode(void);
            void InitLCLS2Mode(void);
            
            void SetClkSel(uint32_t clock_mode);
            void SetMsgDelay(uint32_t delay_ticks);
            
            /* Set channel control registers */
            void ChannelEnable(int channel, uint32_t enable);
            void SetEventCode(int channel, uint32_t event);
            void SetFixedRate(int channel, uint32_t rate);
            void SetACRate(int channel, uint32_t ts_mask, uint32_t rate);
            void SetSeqBit(int channel, uint32_t seq_bit);
            void SetInclusionMask(int channel, uint32_t mask);
            void SetExclusionMask(int channel, uint32_t mask);
            void SetDontCareMask(int channel);
            
            /* Set trigger control registers */
            void TriggerEnable(int trigger, uint32_t enable);
            void SetSourceMask(int trigger, uint32_t source);
            void SetPolarity(int trigger, uint32_t polarity);
            void SetDelay(int trigger, uint32_t delay_ticks);
            void SetWidth(int trigger, uint32_t width_ticks);
            
            void report(void);
            
            uint32_t channelEnable(int channel);
            uint32_t channelRateSelect(int channel);
            uint32_t channelDestSelect(int channel);
            uint32_t channelCount(int channel);
            
            uint32_t triggerEnable(int trigger);
            uint32_t triggerSourceMask(int trigger);
            uint32_t triggerPolarity(int trigger);
            uint32_t triggerWidth(int trigger);
            uint32_t triggerDelay(int trigger);
            
            uint32_t fpgaVersion(void);
            uint32_t upTimeCount(void);
            uint32_t sofCount(void);
            uint32_t eofCount(void);
            uint32_t fidCount(void);
            uint32_t crcErrCount(void);
            uint32_t rxClkCount(void);
            uint32_t rxRstCount(void);
            uint32_t rxDecErrCount(void);
            uint32_t rxDspErrCount(void);
            uint32_t rxLinkStatus(void);
            uint32_t versionErr(void);
            uint32_t frameVersion(void);
            

    };



};

#endif
