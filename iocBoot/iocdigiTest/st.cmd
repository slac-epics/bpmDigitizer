## Example RTEMS startup script

## You may have to change digiTest to something else
## everywhere it appears in this file

#< envPaths

## Register all support components
dbLoadDatabase("../../dbd/digiTest.dbd",0,0)
digiTest_registerRecordDeviceDriver(pdbbase) 

## Load record instances
dbLoadRecords("../../db/digiTest.db","user=bhill")

iocInit()

## Start any sequence programs
#seq sncdigiTest,"user=bhill"
