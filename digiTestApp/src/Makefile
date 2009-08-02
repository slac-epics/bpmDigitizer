TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# build an ioc application

PROD_IOC = digiTest

DBD += digiTest.dbd
# digiTest.dbd will be made up from these files:
digiTest_DBD += base.dbd
#include definitions for any other support applications needed
digiTest_DBD_RTEMS += vmeDigiApp.dbd
digiTest_DBD_RTEMS += vmeDigiSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
digiTest_SRCS += digiTest_registerRecordDeviceDriver.cpp
digiTest_SRCS_DEFAULT += digiTestMain.cpp
digiTest_SRCS_vxWorks += -nil-

#The following adds support from base/src/vxWorks
digiTest_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

#add a definition for each support application used by this application
digiTest_LIBS_RTEMS += vmeDigi
digiTest_LIBS_RTEMS += vmeDigiDevSup

#NOTES:
#      1)It is not possible to build sncExample both as a component of digiTest
#        and standalone. You must choose only one.
#      2)To build sncExample SNCSEQ must be defined in <top>/configure/RELEASE

#The following builds sncExample as a component of digiTest
#digiTest_SRCS += sncExample.stt
#digiTest_LIBS += seq pv

digiTest_LIBS += $(EPICS_BASE_IOC_LIBS)

#The following builds sncExample as a standalone application
#PROD_HOST += sncExample
#sncExample_SNCFLAGS += +m
#sncExample_SRCS += sncExample.stt
#sncExample_LIBS += seq pv
#sncExample_LIBS += $(EPICS_BASE_HOST_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

