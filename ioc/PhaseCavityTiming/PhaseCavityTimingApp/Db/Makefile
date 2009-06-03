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
DB += IP231.template
DB += IP330.template
DB += ip440.template
DB += ip445.template
DB += rfCavity.db
DB += Bx9000Info.template
DB += Bx9000InstalledBTs.template
DB += KL3314.template

# Generate db files from substitutions
DB += IP231.db
DB += IP330.db
DB += ip440.db
DB += ip445.db
DB += phaseCavityevr.db
DB += Bx9000.db
DB += KL3314.db

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

