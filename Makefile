#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS += configure
DIRS += bpmDigitizerLib
DIRS += digiTestApp
include $(TOP)/configure/RULES_TOP
