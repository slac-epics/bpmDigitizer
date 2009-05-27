TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# build an ioc application

PROD_IOC_RTEMS = PhaseCavityTiming

DBD += PhaseCavityTiming.dbd
# PhaseCavityTiming.dbd will be made up from these files:
PhaseCavityTiming_DBD += base.dbd
#include definitions for any other support applications needed
PhaseCavityTiming_DBD += vmeDigiApp.dbd
PhaseCavityTiming_DBD += vmeDigiSupport.dbd
PhaseCavityTiming_DBD += IP231.dbd
PhaseCavityTiming_DBD += IP330.dbd
PhaseCavityTiming_DBD += Xy2445.dbd
PhaseCavityTiming_DBD += Xy2440.dbd
PhaseCavityTiming_DBD += evrSupport.dbd
PhaseCavityTiming_DBD += devMrfEr.dbd
PhaseCavityTiming_DBD += dbRestore.dbd
PhaseCavityTiming_DBD += Bx9000_MBT.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PhaseCavityTiming_SRCS += PhaseCavityTiming_registerRecordDeviceDriver.cpp
PhaseCavityTiming_SRCS_DEFAULT += PhaseCavityTimingMain.cpp
PhaseCavityTiming_SRCS_DEFAULT += multicastTestBld.cpp
PhaseCavityTiming_SRCS_vxWorks += -nil-

#add a definition for each support application used by this application
#PhaseCavityTiming_LIBS += xxx

PhaseCavityTiming_LIBS += multicastBLD
PhaseCavityTiming_LIBS += IP231
PhaseCavityTiming_LIBS += IP330
PhaseCavityTiming_LIBS += Xy2445
PhaseCavityTiming_LIBS += Xy2440
PhaseCavityTiming_LIBS += evrSupport
PhaseCavityTiming_LIBS += generalTime
PhaseCavityTiming_LIBS += devMrfEr
PhaseCavityTiming_LIBS += mrfVme64x
PhaseCavityTiming_LIBS += restore
PhaseCavityTiming_LIBS += Bx9000_MBT
PhaseCavityTiming_LIBS += ModBusTCPClnt
PhaseCavityTiming_LIBS_RTEMS += devBusMapped
PhaseCavityTiming_LIBS_RTEMS += vmeDigiDevSup
PhaseCavityTiming_LIBS_RTEMS += vmeDigi
PhaseCavityTiming_LIBS_RTEMS += Ipac
PhaseCavityTiming_LIBS_RTEMS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

