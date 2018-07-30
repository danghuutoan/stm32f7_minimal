TARGET=main
all: burn

PREFIX=arm-none-eabi

CC=$(PREFIX)-gcc
LD=$(PREFIX)-gcc
AS=$(PREFIX)-as
CP=$(PREFIX)-objcopy
OD=$(PREFIX)-objdump


OBJCOPYFLAGS = -O binary

BIN=$(CP) -O ihex 

BSP = Drivers/BSP/STM32746G-Discovery
CMSIS = Drivers/CMSIS
FREERTOS = Middlewares/Third_Party/FreeRTOS
FATFS = Middlewares/Third_Party/FatFs

DEFS =  -DSTM32F746xx  -DUSE_STM32746G_DISCOVERY -DUSE_HAL_DRIVER
STARTUP = startup_stm32f746xx.s
STLIB = Drivers/STM32F7xx_HAL_Driver
MCU = cortex-m7
MCFLAGS = -mcpu=$(MCU) -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 

STM32_INCLUDES = 	-I$(CMSIS)/Include \
					-I$(CMSIS)/Device/ST/STM32F7xx/Include \
 					-I$(BSP) \
					-I. \
					-I$(STLIB)/Inc \
					-I$(FATFS)/src \
					-I$(FATFS)/src.drivers \
					-I$(FREERTOS)/Source/portable/GCC/ARM_CM7/r0p1 \
					-I$(FREERTOS)/Source/include \
					-I$(FREERTOS)/Source/CMSIS_RTOS \

OPTIMIZE       = -Os

CFLAGS	= $(MCFLAGS)  $(OPTIMIZE)  $(DEFS) -I. -I./ $(STM32_INCLUDES)   -specs=nosys.specs -specs=nano.specs -Wl,-T,STM32F746NGHx_FLASH.ld
CFLAGS+=-DDEBUG

AFLAGS	= $(MCFLAGS) 

SRC = main.c \
	stm32f7xx_it.c \
	system_stm32f7xx.c \
	sd_diskio_dma_rtos.h \
	log.c \
	$(BSP)/stm32746g_discovery_audio.c \
	$(BSP)/stm32746g_discovery_sd.c \
	$(BSP)/stm32746g_discovery.c \
	Drivers/BSP/Components/wm8994/wm8994.c \
	$(STLIB)/Src/stm32f7xx_hal_cortex.c \
	$(STLIB)/Src/stm32f7xx_hal_dma_ex.c \
	$(STLIB)/Src/stm32f7xx_hal_gpio.c \
	$(STLIB)/Src/stm32f7xx_hal_i2c_ex.c \
	$(STLIB)/Src/stm32f7xx_hal_i2c.c \
	$(STLIB)/Src/stm32f7xx_hal_pwr_ex.c \
	$(STLIB)/Src/stm32f7xx_hal_pwr.c \
	$(STLIB)/Src/stm32f7xx_hal_rcc_ex.c \
	$(STLIB)/Src/stm32f7xx_hal_rcc.c \
	$(STLIB)/Src/stm32f7xx_hal_sai.c \
	$(STLIB)/Src/stm32f7xx_hal_sd.c \
	$(STLIB)/Src/stm32f7xx_hal_tim_ex.c \
	$(STLIB)/Src/stm32f7xx_hal_tim.c \
	$(STLIB)/Src/stm32f7xx_hal_uart.c \
	$(STLIB)/Src/stm32f7xx_hal.c \
	$(STLIB)/Src/stm32f7xx_hal_dma.c \
	$(STLIB)/Src/stm32f7xx_ll_fmc.c \
	$(STLIB)/Src/stm32f7xx_ll_sdmmc.c \
	$(FATFS)/src/diskio.c \
	$(FATFS)/src/ff_gen_drv.c \
	$(FATFS)/src/ff.c \
	sd_diskio_dma_rtos.c \
	$(FATFS)/src/option/syscall.c \
	$(FATFS)/src/option/unicode.c \
	$(FREERTOS)/Source/CMSIS_RTOS/cmsis_os.c \
	$(FREERTOS)/Source/portable/MemMang/heap_4.c \
	$(FREERTOS)/Source/portable/GCC/ARM_CM7/r0p1/port.c \
	$(FREERTOS)/Source/list.c \
	$(FREERTOS)/Source/queue.c \
	$(FREERTOS)/Source/tasks.c \
	$(FREERTOS)/Source/timers.c \


burn : $(TARGET).bin
	# openocd -f flash.cfg #-d3
	st-flash write $(TARGET).bin 0x08000000

$(TARGET).bin : $(TARGET).out
	$(CP) $(OBJCOPYFLAGS) $< $@

$(TARGET).hex: $(EXECUTABLE)
	$(CP) -O ihex $^ $@

$(TARGET).out : $(SRC) $(STARTUP)
	$(CC) $(CFLAGS) $^ -lm -lc -lnosys  -o $@

clean:
	rm -f $(TARGET).lst $(TARGET).out $(TARGET).hex $(TARGET).bin $(TARGET).map  $(EXECUTABLE)
