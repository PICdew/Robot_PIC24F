#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Robot_PIC24F.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Robot_PIC24F.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=FreeRTOS/Source/portable/MemMang/heap_2.c FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.S FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.c FreeRTOS/Source/croutine.c FreeRTOS/Source/list.c FreeRTOS/Source/queue.c FreeRTOS/Source/tasks.c FreeRTOS/Source/timers.c main.c hook.c lcd_i2c_1602.c i2c_func.c gyro_mpu6050.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o ${OBJECTDIR}/FreeRTOS/Source/croutine.o ${OBJECTDIR}/FreeRTOS/Source/list.o ${OBJECTDIR}/FreeRTOS/Source/queue.o ${OBJECTDIR}/FreeRTOS/Source/tasks.o ${OBJECTDIR}/FreeRTOS/Source/timers.o ${OBJECTDIR}/main.o ${OBJECTDIR}/hook.o ${OBJECTDIR}/lcd_i2c_1602.o ${OBJECTDIR}/i2c_func.o ${OBJECTDIR}/gyro_mpu6050.o
POSSIBLE_DEPFILES=${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o.d ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.d ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o.d ${OBJECTDIR}/FreeRTOS/Source/croutine.o.d ${OBJECTDIR}/FreeRTOS/Source/list.o.d ${OBJECTDIR}/FreeRTOS/Source/queue.o.d ${OBJECTDIR}/FreeRTOS/Source/tasks.o.d ${OBJECTDIR}/FreeRTOS/Source/timers.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/hook.o.d ${OBJECTDIR}/lcd_i2c_1602.o.d ${OBJECTDIR}/i2c_func.o.d ${OBJECTDIR}/gyro_mpu6050.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o ${OBJECTDIR}/FreeRTOS/Source/croutine.o ${OBJECTDIR}/FreeRTOS/Source/list.o ${OBJECTDIR}/FreeRTOS/Source/queue.o ${OBJECTDIR}/FreeRTOS/Source/tasks.o ${OBJECTDIR}/FreeRTOS/Source/timers.o ${OBJECTDIR}/main.o ${OBJECTDIR}/hook.o ${OBJECTDIR}/lcd_i2c_1602.o ${OBJECTDIR}/i2c_func.o ${OBJECTDIR}/gyro_mpu6050.o

# Source Files
SOURCEFILES=FreeRTOS/Source/portable/MemMang/heap_2.c FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.S FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.c FreeRTOS/Source/croutine.c FreeRTOS/Source/list.c FreeRTOS/Source/queue.c FreeRTOS/Source/tasks.c FreeRTOS/Source/timers.c main.c hook.c lcd_i2c_1602.c i2c_func.c gyro_mpu6050.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/Robot_PIC24F.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24FJ256GB110
MP_LINKER_FILE_OPTION=,--script=p24FJ256GB110.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o: FreeRTOS/Source/portable/MemMang/heap_2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source/portable/MemMang 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/portable/MemMang/heap_2.c  -o ${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o: FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.c  -o ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FreeRTOS/Source/croutine.o: FreeRTOS/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/croutine.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/croutine.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/croutine.c  -o ${OBJECTDIR}/FreeRTOS/Source/croutine.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/croutine.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/croutine.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FreeRTOS/Source/list.o: FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/list.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/list.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/list.c  -o ${OBJECTDIR}/FreeRTOS/Source/list.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/list.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/list.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FreeRTOS/Source/queue.o: FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/queue.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/queue.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/queue.c  -o ${OBJECTDIR}/FreeRTOS/Source/queue.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/queue.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/queue.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FreeRTOS/Source/tasks.o: FreeRTOS/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/tasks.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/tasks.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/tasks.c  -o ${OBJECTDIR}/FreeRTOS/Source/tasks.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/tasks.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/tasks.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FreeRTOS/Source/timers.o: FreeRTOS/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/timers.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/timers.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/timers.c  -o ${OBJECTDIR}/FreeRTOS/Source/timers.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/timers.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/timers.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hook.o: hook.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/hook.o.d 
	@${RM} ${OBJECTDIR}/hook.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hook.c  -o ${OBJECTDIR}/hook.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hook.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hook.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/lcd_i2c_1602.o: lcd_i2c_1602.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/lcd_i2c_1602.o.d 
	@${RM} ${OBJECTDIR}/lcd_i2c_1602.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  lcd_i2c_1602.c  -o ${OBJECTDIR}/lcd_i2c_1602.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/lcd_i2c_1602.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/lcd_i2c_1602.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/i2c_func.o: i2c_func.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/i2c_func.o.d 
	@${RM} ${OBJECTDIR}/i2c_func.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  i2c_func.c  -o ${OBJECTDIR}/i2c_func.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/i2c_func.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/i2c_func.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/gyro_mpu6050.o: gyro_mpu6050.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/gyro_mpu6050.o.d 
	@${RM} ${OBJECTDIR}/gyro_mpu6050.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  gyro_mpu6050.c  -o ${OBJECTDIR}/gyro_mpu6050.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/gyro_mpu6050.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/gyro_mpu6050.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o: FreeRTOS/Source/portable/MemMang/heap_2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source/portable/MemMang 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/portable/MemMang/heap_2.c  -o ${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o.d"      -g -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/portable/MemMang/heap_2.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o: FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.c  -o ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o.d"      -g -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/port.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FreeRTOS/Source/croutine.o: FreeRTOS/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/croutine.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/croutine.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/croutine.c  -o ${OBJECTDIR}/FreeRTOS/Source/croutine.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/croutine.o.d"      -g -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/croutine.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FreeRTOS/Source/list.o: FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/list.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/list.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/list.c  -o ${OBJECTDIR}/FreeRTOS/Source/list.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/list.o.d"      -g -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/list.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FreeRTOS/Source/queue.o: FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/queue.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/queue.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/queue.c  -o ${OBJECTDIR}/FreeRTOS/Source/queue.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/queue.o.d"      -g -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/queue.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FreeRTOS/Source/tasks.o: FreeRTOS/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/tasks.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/tasks.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/tasks.c  -o ${OBJECTDIR}/FreeRTOS/Source/tasks.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/tasks.o.d"      -g -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/tasks.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FreeRTOS/Source/timers.o: FreeRTOS/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/timers.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/timers.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FreeRTOS/Source/timers.c  -o ${OBJECTDIR}/FreeRTOS/Source/timers.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/timers.o.d"      -g -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/timers.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -g -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hook.o: hook.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/hook.o.d 
	@${RM} ${OBJECTDIR}/hook.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hook.c  -o ${OBJECTDIR}/hook.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hook.o.d"      -g -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hook.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/lcd_i2c_1602.o: lcd_i2c_1602.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/lcd_i2c_1602.o.d 
	@${RM} ${OBJECTDIR}/lcd_i2c_1602.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  lcd_i2c_1602.c  -o ${OBJECTDIR}/lcd_i2c_1602.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/lcd_i2c_1602.o.d"      -g -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/lcd_i2c_1602.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/i2c_func.o: i2c_func.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/i2c_func.o.d 
	@${RM} ${OBJECTDIR}/i2c_func.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  i2c_func.c  -o ${OBJECTDIR}/i2c_func.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/i2c_func.o.d"      -g -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/i2c_func.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/gyro_mpu6050.o: gyro_mpu6050.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/gyro_mpu6050.o.d 
	@${RM} ${OBJECTDIR}/gyro_mpu6050.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  gyro_mpu6050.c  -o ${OBJECTDIR}/gyro_mpu6050.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/gyro_mpu6050.o.d"      -g -omf=elf -ffunction-sections -fdata-sections -mlarge-data -O0 -I"FreeRTOS/Source/include" -I"." -DMPLAB_PIC24_PORT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/gyro_mpu6050.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o: FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.S  -o ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.d"  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -Wa,-MD,"${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.asm.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.d" "${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.asm.d"  -t $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o: FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.d 
	@${RM} ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.S  -o ${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.d"  -omf=elf -Wa,-MD,"${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.asm.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.d" "${OBJECTDIR}/FreeRTOS/Source/portable/MPLAB/PIC24_dsPIC/portasm_PIC24.o.asm.d"  -t $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Robot_PIC24F.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Robot_PIC24F.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf  -mreserve=data@0x800:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x825 -mreserve=data@0x826:0x84F   -Wl,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,$(MP_LINKER_FILE_OPTION),--heap=1024,--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--cref,--warn-section-align$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Robot_PIC24F.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Robot_PIC24F.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -Wl,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--heap=1024,--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--cref,--warn-section-align$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Robot_PIC24F.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
