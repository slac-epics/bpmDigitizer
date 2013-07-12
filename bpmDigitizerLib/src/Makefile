TOP=../..

include $(TOP)/configure/CONFIG

CROSS_COMPILER_TARGET_ARCHS=RTEMS-beatnik
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

# Comment this line to build a host-ioc; you'd need to exclude
# several facilities (evr, micro, ...) for this to work.
BUILDFOR=_RTEMS

ifdef UDPCOMM
PROD_IOC_RTEMS     += vmeDigiComm
endif
PROD_IOC_RTEMS     += vmeDigiApp
PROD_IOC_RTEMS     += vmeDigiDevSupMod
# create a fake library containing dependecies of vmeDigiDevSupMod
#LIB_INSTALLS_RTEMS += libvmeDigiDevSupMod-symdeps.a

LIBRARY_IOC_RTEMS  += vmeDigi
LIBRARY_IOC_RTEMS  += vmeDigiDevSup

vmeDigi_SRCS       = vme64x.c vmeDigi.c

vmeDigiDevSup_SRCS = devVmeDigiSupport.c devWfVmeDigi.c devWfVmeDigiSup.c
ifdef UDPCOMM
vmeDigiDevSup_SRCS += devWfVmeDigiSim.c
endif

vmeDigiComm_SRCS   = vmeDigiComm.c $(vmeDigi_SRCS)

DBD                = vmeDigiApp.dbd
DBD               += vmeDigiSupport.dbd
vmeDigiDevSupMod_SRCS = vmeDigiSupport_registerRecordDeviceDriver.cpp
vmeDigiDevSupMod_SRCS+= $(vmeDigiDevSup_SRCS)

vmeDigiApp_DBD   = base.dbd
vmeDigiApp_DBD  += vmeDigiWf.dbd
vmeDigiApp_DBD  += devBusMapped.dbd

vmeDigiApp_SRCS  = vmeDigiApp_registerRecordDeviceDriver.cpp

vmeDigiApp_LIBS += vmeDigiDevSup
vmeDigiApp_LIBS += vmeDigi
vmeDigiApp_LIBS += devBusMapped

vmeDigiApp_LIBS += $(EPICS_BASE_IOC_LIBS)

#USR_CFLAGS_RTEMS  = -I$(TOP)/include -I$(RTEMS_BASE)/

INC += vmeDigi.h vmeDigiComm.h vme64x.h

DB              += vmeDigiApp.db
DB              += vmeDigiSim.db
#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
