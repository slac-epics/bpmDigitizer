TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
DB += vmeDigiApp.db
DB += rfCavity.db

# Generate db files from substitutions
DB += IP231.db
DB += IP330.db
DB += ip440.db
DB += ip445.db
DB += phaseCavityevr.db
DB += Bx9000.db
DB += KL3314.db
DB += iocAdmin.db

# Nothing to build, just install from autosave/db to PhaseCavityTiming/db
DB_INSTALLS += $(AUTOSAVE)/db/save_restoreStatus.db

# Specify template dependencies
# TODO: AddDevGuide.pdf says this should work, but it doesn't.  Fix me.
Bx9000_DEPENDS			=	Bx9000Info.template Bx9000InstalledBTs.template
IP231_DEPENDS			=	IP231.template
IP330_DEPENDS			=	IP330.template
KL3314_DEPENDS			=	KL3314.template
iocAdmin_DEPENDS		=	access.db ioc.db iocCluster.db
ip440_DEPENDS			=	ip440.template
ip445_DEPENDS			=	ip445.template
phaseCavityevr_DEPENDS	=	evrWithExtDelays.template evrLabels.template	\
							evrEventCtrl.template evrEventRate.template

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

