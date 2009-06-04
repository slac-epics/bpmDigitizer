#------------------------------------------------------------------------------
# $(TIMING)/mrfApp/src/Makefile
#
# Author:  Eric Bjorklund
# Date:    19 December 2005
#
#------------------------------------------------------------------------------
# This file contains the EPICS make rules and definitions for making and
# installing the driver support modules for the Micro-Research Finland
# Series 200 event receiver module.
#
#------------------------------------------------------------------------------
# Modification History:
#
# 19 Dec 2005   Bjo     Original Release
# 21 Aug 2006   Saa     Changes to build on RTEMS too.
#
#------------------------------------------------------------------------------

#---------------------
# Set up the Environment
#
TOP = ../..
SOURCE_DIR := $(patsubst %../.., %., $(TOP))

include $(TOP)/configure/CONFIG
include $(SOURCE_DIR)/MRF_CONFIG

#USR_CFLAGS += -pedantic
USR_CFLAGS += -DDEBUG_PRINT
ifeq ($(BSP_SUPPORTS_PCI), YES)
    USR_CFLAGS += -DPCI
endif

ifneq ($(T_A),linux-x86)
MRFAPI_DIR=$(MRFAPI)/$(T_A)
else
MRFAPI_DIR=$(MRFAPI)/linux-i686
endif

#---------------------
# Only build for "hardware" IOC's
#
#ifneq ($(findstring Host, $(VALID_BUILDS)),)
#    VALID_BUILDS = Host
#endif

#---------------------
# Build the record support modules
#
CUSTOM_RECORDS += egRecord
CUSTOM_RECORDS += egeventRecord
CUSTOM_RECORDS += erRecord
CUSTOM_RECORDS += ereventRecord

DBDINC += $(CUSTOM_RECORDS)

#---------------------
# Install DBD files
#
DBD += drvMrfEg.dbd
DBD += devMrfEg.dbd
DBD += devMrfEgEvent.dbd
DBD += drvMrfEr.dbd
DBD += devMrfEr.dbd
DBD += devMrfEgLib.dbd
DBD += devMrfErLib.dbd
devMrfEgLib_DBD += devMrfEg.dbd
devMrfEgLib_DBD += devMrfEgEvent.dbd
devMrfErLib_DBD += base.dbd
devMrfErLib_DBD += devMrfEr.dbd

#---------------------
# Build the Event Generator Library
#
PROD_IOC_vxWorks += devMrfEgLib
LIBRARY_IOC      += devMrfEg

# Source code common to all platforms
devMrfEg_OBJS += devMrfEg
devMrfEg_OBJS += egRecord
devMrfEg_OBJS += egeventRecord
# RTEMS source
devMrfEg_OBJS_RTEMS += drvMrfEg
# VxWorks source
devMrfEg_OBJS_vxWorks += drvMrfEg
# Linux source
devMrfEg_OBJS_Linux += drvLinuxEvg

devMrfEgLib_OBJS += $(devMrfEg_OBJS)
devMrfEgLib_SRCS += devMrfEgLib_registerRecordDeviceDriver.cpp

OBJS_IOC_vxWorks += $(devMrfEg_OBJS)

#---------------------
# Build the Event Receiver Library
#
PROD_IOC_vxWorks += devMrfErLib
LIBRARY_IOC      += devMrfEr

# Source code common to all platforms
devMrfEr_OBJS += devMrfEr
devMrfEr_OBJS += erRecord
devMrfEr_OBJS += ereventRecord
# RTEMS source
devMrfEr_OBJS_RTEMS += drvMrfEr
ifeq ($(BSP_SUPPORTS_PCI), YES)
devMrfEr_OBJS_RTEMS += devLibPMC
devMrfEr_OBJS_RTEMS += plx9080_eeprom.c
devMrfEr_OBJS_RTEMS += evrEEPROMFixup
endif
# VxWorks source
devMrfEr_OBJS_vxWorks += drvMrfEr
ifeq ($(BSP_SUPPORTS_PCI), YES)
devMrfEr_OBJS_vxWorks += devLibPMC
devMrfEr_OBJS_vxWorks += plx9080_eeprom.c
devMrfEr_OBJS_vxWorks += evrEEPROMFixup
endif
# Linux source
devMrfEr_OBJS_Linux += drvLinuxEvr

devMrfErLib_OBJS += $(devMrfEr_OBJS)
devMrfErLib_SRCS += devMrfErLib_registerRecordDeviceDriver.cpp

OBJS_IOC_vxWorks += $(devMrfEr_OBJS)

#---------------------
# Build the Library for Accessing VME-64X CR/CSR space
#
PROD_IOC_vxWorks  += drvMrfLib
LIBRARY_IOC      += drvMrf

drvMrf_OBJS_RTEMS += mrfVme64x
drvMrf_OBJS_RTEMS += $(CRCSR_PROBE_NAME)
drvMrf_OBJS_vxWorks += mrfVme64x
drvMrf_OBJS_vxWorks += $(CRCSR_PROBE_NAME)
drvMrf_OBJS_Linux += drvLinux
drvMrf_OBJS_Linux += $(MRFAPI_DIR)/erapi.o
CFLAGS+=-I$(MRFAPI_DIR)

drvMrfLib_OBJS += $(drvMrf_OBJS)

OBJS_IOC_vxWorks  += $(drvMrf_OBJS)

#---------------------
# Testing
#
#DBD += devMrf.dbd
#devMrf_DBD += devMrfEg.dbd
#devMrf_DBD += devMrfEr.dbd
#
#PROD_IOC  += devMrf
#
#devMrf_SRCS += devMrf_registerRecordDeviceDriver.cpp
#
#devMrf_LIBS += devMrfEg devMrfEr mrfVme64x
#devMrf_LIBS += $(EPICS_BASE_IOC_LIBS)

#---------------------
# Install include files
#
INC += mrfCommon.h
INC += mrfVme64x.h

INC += drvMrfEg.h
INC += drvMrfEr.h
INC += devMrfEr.h

INC += debugPrint.h

#=======================================================================#
#                               RULES                                   #
#=======================================================================#

#---------------------
# Generic EPICS build rules
#
include $(TOP)/configure/RULES
