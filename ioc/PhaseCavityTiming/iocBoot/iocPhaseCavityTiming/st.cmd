## RTEMS startup script
## Install this st.cmd in the top of the project directory

ld( "bin/RTEMS-beatnik/PhaseCavityTiming.obj" )

#< envPaths

# Startup for Bill Ross' 120MSPS VME Digitizer Test Driver
# (simple EPICS waveform)
# Make sure we can transfer arrays > 16k.
# Must be > max # elements * 8 since data
# may be converted to doubles on IOC if
# the client requests it.
setenv("EPICS_CA_MAX_ARRAY_BYTES", "2000000", 1)

epicsEnvSet( "ENGINEER", "Bruce Hill" )
epicsEnvSet( "LOCATION", "UND:R02:IOC:19" )

# Silence BSP warnings
bspExtVerbosity = 0

# Configure VME digitizers
#
#   devVmeDigiConfig(card_no, slot_no, vme_a32_addr, int_vec, int_lvl)
#
# card_no:
#     'unit number' associated with a particular card (1..8).
#
# slot_no:
#     VME slot the card is sitting in (1-based, i.e., a number from 1..21).
#
# vme_a32_addr:
#     Address in VME A32 space where card's sample memory is to be mapped.
#
# int_vec:
#     Unique VME interupt vector.
#
# int_lvl:
#     VME interrupt level (may be shared among cards).
#
# NOTE: 'slot_no' may be zero. In this case the VME bus is scanned
#       for instance 'card_no'. E.g., if digitizers are in slot 5 + 7
#       and 'card_no==2', 'slot_no==0' then the card in slot 7 is
#       be configured and associated with card/unit number '2'.
#
#       If both, 'card_no' and 'slot_no' are zero then the VME bus
#       is scanned and all cards that are found are configured and
#       assigned unique card numbers (from 1..N).
#       The sample memories of the cards are mapped at consecutive
#       blocks starting at vme_a32_addr. Interrupt vectors are assigned
#       starting with 'int_vec'. E.g., if 'int_vec==0x30' then cards
#       1..3 will use vectors 0x30, 0x31 and 0x32. The same 'int_lvl'
#       is shared by all cards.
#
devVmeDigiConfig(0,0,0x20000000,0x40,3)

# Initialize Acromag IPAC carrier
# Slot 0:	IP440AE   Digital Input
# Slot 1:	IP445E    Digital Output
# Slot 2:	IP330AE   Analog  Input
# Slot 3:	IP231-16E Analog  Output
ipacAddCarrier(&xy9660,"0x0000")

# Initialize IP440 digital input module
# xy2440Create( <name>, <card>, <slot>, <modeName>, <intHndlr>, <usrFunc>, <vector>, <event>, <debounce> )
xy2440Create( "di0", 0, 0, "STANDARD", "LEVEL", 0x0, 0x80, 0x0, 0x00 )

# Initialize IP445 digital output module
# xy2445Create( <name>, <card>, <slot> )
xy2445Create( "do0", 0, 1 )

#	Initialize IP330 analog input module
#	See the Acromag IP330 User's manual in ip330/documentation
#	for details on scan modes, triggers, timer, and other options.
#	ip330Create(
#		char	*	cardname,	Unique Identifier "ip330-1"
#		UINT16		carrier,	Ipac Driver carrier card number
#		UINT16		slot,		Slot number on IP carrier
#		char	*	a2d_range,	One of:	"-5to5D" "-10to10D" "0to5D" "0to10D"
#										"-5to5S" "-10to10S" "0to5S" "0to10S"
#		char	*	channels,	Format is "ch%d-ch%d", Ex: "ch0-ch15" or "ch21-ch31"
#								Available range is 0 to 15/31 depending on Differential or
#								Single inputs.
#		UINT32		gainL,		The lower two bits is gain for channel0, then channel 1 ... 15
#		UINT32		gainH,		The lower two bits is gain for channel16, then channel 17 ... 31
#		char	*	scanmode,	Format is "%s-%s-%s", <mode>-<trigger>-Avg<avgCount>
#								mode can be "uniformCont", "uniformSingle", "burstCont",
#									"burstSingle", or "cvtOnExt"
#								trigger can be "Input" or "Output"
#									If mode is "cvtOnExt", trigger must be "Input"
#								avgCount range is 1 to 256 and specifies how many samples to
#									average over
#								For low frequency applications without exacting timing
#									requirements, a scanmode of "burstCont-Output-Avg1" is typical.
#		char	*	timer,		Format is "x*y@8MHz" where x must be [64,255], y must be [1,65535]
#								The timing interval can be calculated from (x*y)/8 in microseconds.
#		UINT8		vector )	Interrupt vector should be 0x66
ip330Create( "ai0", 0, 2, "-10to10D", "ch0-ch1", 0, 0, "burstCont-Output-Avg1", "80*3@8MHz", 0x66 )
ip330StartConvertByName( "ai0" )

# Initialize IP231 analog output module
# ip231Create( <name>, <card>, <slot>, <dacmode> )
# <dacmode> must be "transparent" or "simultaneous"
# In transparent mode, outputs are updated when written
# In simultaneous mode, multiple output values can be setup,
# and all outputs update when ip231SimulTrigger(<cardNo>) is called
ip231Create( "ao0", 0, 3, "transparent" )

# Configure EVR if it exists.
# Choose the proper configuration - PMC or VME.
# Note - see README_evrTest for hardware setup and instructions for PMC 
# one-time configuration.
#
#ErConfigure(0,0x000000,0x00,0,1)  # PMC type
#ErConfigure(0,0x300000,0x60,4,0)  # VME type.
#
#    VME: ErConfigure(<instance>,<address>,<vector>,<level>,0)
#    PMC: ErConfigure(<instance>,    0    ,    0   ,   0   ,1)
#
#    where instance = EVR instance, starting from 0, incrementing by 1
#                     for each subsequent card
#    and   address  = VME card address, starting from 0x300000, 
#                     incrementing by 0x100000 for each subsequent card
#                     (0 for PMC)
#    and   vector   = VME interrupt vector, starting from 0x60, 
#                     incrementing by 0x02 for each subsequent card
#                     (0 for PMC)
#    and   level    = VME interrupt level (set to 4 - can be the same 
#                     for all EVRs)
#                     (0 for PMC)
#    and   0        = VME
#       or 1        = PMC
ErConfigure( 0, 0, 0, 0, 1 )

# Initialize Beckhoff Coupler
# Bx9000_Coupler_Add( UINT8 * cplrname, UINT8 * ipaddr, UINT8 * init_string)
Bx9000_Coupler_Add( "UND-R01-BHC-05", "172.21.10.70", 0 )

# Add terminals to the coupler
# Bx9000_Terminal_Add( UINT8 * cplrname, UINT16 slot, UINT8 * btname, UINT8 * init_string)
Bx9000_Terminal_Add("UND-R01-BHC-05", 1, "KL3314", 0)
Bx9000_Terminal_Add("UND-R01-BHC-05", 2, "KL3314", 0)
Bx9000_Terminal_Add("UND-R01-BHC-05", 3, "KL9010", 0)

# Load EPICS database definition
dbLoadDatabase("dbd/PhaseCavityTiming.dbd",0,0)

## Register all support components
PhaseCavityTiming_registerRecordDeviceDriver(pdbbase) 

## Load EPICS records
dbLoadRecords( "db/IP231.db", "IOC=UND:R02:IOC:19,CARD=ao0" )
dbLoadRecords( "db/IP330.db", "IOC=UND:R02:IOC:19,CARD=ai0" )
dbLoadRecords( "db/ip440.db", "IOC=UND:R02:IOC:19,CARD=di0" )
dbLoadRecords( "db/ip445.db", "IOC=UND:R02:IOC:19,CARD=do0" )
dbLoadRecords( "db/rfCavity.db", "IOC=UND:R02:IOC:19" )
dbLoadRecords( "db/vmeDigiApp.db", "digi=UND:R02:IOC:19:dig1,card=1,nelm=4096" )
dbLoadRecords( "db/phaseCavityevr.db", "IOC=UND:R02:IOC:19,EVR=UND:R02:EVR:19" )
dbLoadRecords( "db/Bx9000.db", "COUPLER=UND-R01-BHC-05,COUPLER_LOC=UND:R01:BHC:05" )
dbLoadRecords( "db/KL3314.db", "COUPLER=UND-R01-BHC-05,COUPLER_LOC=UND:R01:BHC:05" )
dbLoadRecords( "db/iocAdmin.db", "IOC=UND:R02:IOC:19" )
dbLoadRecords( "db/save_restoreStatus.db", "IOC=UND:R02:IOC:19" )

# Setup access control
#asSetFilename( "/home/bhill/wa2/epics/modules/iocAdmin/db/baseSecurity.acf" )
#asSetSubstitutions( "IOC=UND:R02:IOC:19" )

# Setup autosave
set_savefile_path( "/home/bhill/iocPhaseCavity/autosave/iocData" )
set_requestfile_path( "/home/bhill/iocPhaseCavity/autosave" )
save_restoreSet_IncompleteSetsOk( 1 )
save_restoreSet_DatedBackupFiles( 1 )
set_pass0_restoreFile( "autosave.sav" )
set_pass1_restoreFile( "autosave.sav" )

# Load the gdb stub and start it
ld( "/boot/rtems/rtems-4.7.1-p2/target/ssrlApps/powerpc-rtems/beatnik/bin/rtems-gdb-stub.obj" )
rtems_gdb_start()

# Already being loaded
#ld( "/boot/rtems/rtems-4.7.1-p2/target/ssrlApps/powerpc-rtems/beatnik/bin/miscUtils.obj" )
# Try spawnUtil()

# Print list of loaded binaries (helpful for debugging)
lsmod()

# An EDM panel for the phase cavity timing IOC can be
# started with the command:
# edm -x -m "IOC=<ioc_name>" display/phaseCavity.edl
# Example:
# edm -x -m "IOC=UND:R02:IOC:19" display/phaseCavity.edl

# Convenience aliases
reboot=rtemsReboot
mon=rtemsMonitor

iocInit()

# Start autosave backups
create_monitor_set( "autosave.req", 5, "IOC=UND:R02:IOC:19" )
