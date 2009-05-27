TOP = ../../..
include $(TOP)/configure/CONFIG

SHARED_LIBRARIES = YES

USR_CXXFLAGS_Linux += -DSINGLE_THREADED=1 -D__epics__=1
USR_CXXFLAGS_Darwin += -DSINGLE_THREADED=1 -D__epics__=1
USR_CXXFLAGS_Darwin += -D__solaris__ -DSINGLE_THREADED=1 -D__epics__=1
USR_CXXFLAGS_hpux11_11_mt += -DHP_UX=1111 -DSINGLE_THREADED=1 -D__epics__=1


INSTALL_INCLUDE = $(INSTALL_LOCATION)/include/edm
INC += pv.h
INC += epicsPv.h

Epics_LIBS += cfcaa62e-8199-11d3-a77f-00104b8742df

Epics_SYS_LIBS_Linux += pthread dl
Epics_SYS_LIBS_Darwin += pthread dl
Epics_SYS_LIBS_solaris += pthread dl 
Epics_SYS_LIBS_hpux11_11_mt += pthread

LIBRARY = Epics

USR_LIBS_DEFAULT = ca Com
USR_LIBS_Darwin = -nil-  

USR_INCLUDES += -I../../lib


LIB_SRCS += reg_pvs.cc
LIB_SRCS += epicsPv.cc
LIB_SRCS += pv.cc
  
include $(TOP)/configure/RULES

ifneq (,$(strip $(INSTALL_EDM)))
buildInstall:	installLibEdmBase $(INSTALL_EDM)/edmPvObjects

#	EDMOBJECTS=$(INSTALL_EDM) $(INSTALL_BIN)/edm$(EXE) -add $(INSTALL_EDM_LIB)/libEdmBase$(SHRLIB_SUFFIX)
#	EDMOBJECTS=$(INSTALL_EDM) $(INSTALL_BIN)/edm$(EXE) -add $(INSTALL_EDM_LIB)/libEdmBase$(LIB_SUFFIX)
installLibEdmBase: $(INSTALL_BIN)/edm$(EXE)
	@echo "Installing edmObjects for libEdmBase"
ifneq (,$(strip $(SHRLIB_SUFFIX)))
	$(EDM_CMD) -add $(INSTALL_EDM_LIB)/libEdmBase$(SHRLIB_SUFFIX)
else
	$(EDM_CMD) -add $(INSTALL_EDM_LIB)/libEdmBase$(LIB_SUFFIX)
endif	# SHRLIBNAME
	sed -i -e 's^$(INSTALL_EDM_LIB)^$$\(EDMLIBS\)^' $(INSTALL_EDM)/edmObjects

endif	# INSTALL_EDM
