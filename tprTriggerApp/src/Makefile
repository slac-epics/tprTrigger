TOP=../..

include $(TOP)/configure/CONFIG

USR_CXXFLAGS += -DHAVE_YAML
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#========================================
INCLUDES += $(addprefix -I,$(HPSTPR_INCLUDE))
INCLUDES += $(addprefix -I,$(YAML_INCLUDE))
INCLUDES += $(addprefix -I,$(BOOST_INCLUDE))

## this part is just for yaml loader driver
INCLUDES += $(addprefix -I,$(CPSW_FRAMEWORK_INCLUDE))



INC += tprTriggerAsynDriver.h
DBD += tprTriggerAsynDriver.dbd

LIBRARY_IOC = tprTrigger

tprTrigger_DBD  += tprTriggerAsynDriver.dbd
tprTrigger_SRCS += tprTriggerAsynDriver.cpp

yaml-cpp_DIR += $(YAML_LIB)
tprTrigger_LIBS += yaml-cpp

cpsw_DIR = $(CPSW_FRAMEWORK_LIB)
tprTrigger_LIBS += cpsw

hpstpr_DIR = $(HPSTPR_LIB)
tprTrigger_LIBS += hpstpr

tprTrigger_LIBS += asyn
tprTrigger_LIBS += yamlLoader

tprTrigger_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

