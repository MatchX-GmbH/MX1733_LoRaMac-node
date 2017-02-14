PROJ=		minimal
OBJDIR?=	obj
SDKVER?=	1.0.6.968
SDKDIR?=	../DA1468x_SDK_BTLE_v_$(SDKVER)
#PROD_ID?=	DA14681-00

#SRCS=	main.c $(SDKDIR)/sdk/bsp/startup/config.c
OBJS+=	$(OBJDIR)/main.o \
	$(OBJDIR)/os-hooks.o

OBJS+=	$(OBJDIR)/lmic/aes.o $(OBJDIR)/lmic/lmic.o \
	$(OBJDIR)/lmic/oslmic.o $(OBJDIR)/lmic/radio.o

OBJS+=	$(OBJDIR)/sdk/bsp/startup/config.o \
	$(OBJDIR)/sdk/bsp/startup/startup_ARMCM0.o \
	$(OBJDIR)/sdk/bsp/startup/system_ARMCM0.o \
	$(OBJDIR)/sdk/bsp/startup/vector_table.o \
	$(OBJDIR)/sdk/bsp/adapters/src/ad_uart.o \
	$(OBJDIR)/sdk/bsp/memory/src/qspi_automode.o \
	$(OBJDIR)/sdk/bsp/osal/resmgmt.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_cpm.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_dma.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_gpio.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_gpadc.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_hard_fault.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_otpc.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_qspi.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_timer1.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_uart.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_usb_charger.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_watchdog.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_wkup.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/sys_tcs.o \
	$(OBJDIR)/sdk/bsp/system/sys_man/sys_charger.o \
	$(OBJDIR)/sdk/bsp/system/sys_man/sys_clock_mgr.o \
	$(OBJDIR)/sdk/bsp/system/sys_man/sys_power_mgr.o \
	$(OBJDIR)/sdk/bsp/system/sys_man/sys_rtc.o \
	$(OBJDIR)/sdk/bsp/system/sys_man/sys_watchdog.o \
	$(OBJDIR)/sdk/bsp/free_rtos/event_groups.o \
	$(OBJDIR)/sdk/bsp/free_rtos/list.o \
	$(OBJDIR)/sdk/bsp/free_rtos/tasks.o \
	$(OBJDIR)/sdk/bsp/free_rtos/timers.o \
	$(OBJDIR)/sdk/bsp/free_rtos/queue.o \
	$(OBJDIR)/sdk/bsp/free_rtos/portable/MemMang/heap_4.o \
	$(OBJDIR)/sdk/bsp/free_rtos/portable/GCC/ARM_CM0/port.o

CC=		arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb
#CFLAGS+=	-Wall -Wextra -Werror
CFLAGS+=	-std=gnu11 -Wall -Werror
CFLAGS+=	-Os -fsigned-char -ffunction-sections -fdata-sections
CFLAGS+=	-Ddg_configBLACK_ORCA_IC_REV=BLACK_ORCA_IC_REV_A \
		-Ddg_configBLACK_ORCA_IC_STEP=BLACK_ORCA_IC_STEP_D \
		-DCONFIG_AT45DB011D=1 -DCONFIG_24LC256=1 -DCONFIG_FM75=1 \
		-DRELEASE_BUILD
CFLAGS+=	-I$(SDKDIR)/sdk/bsp/include -I$(SDKDIR)/sdk/bsp/config \
		-I$(SDKDIR)/sdk/bsp/peripherals/include \
		-I$(SDKDIR)/sdk/bsp/memory/include \
		-I$(SDKDIR)/sdk/bsp/adapters/include \
		-I$(SDKDIR)/sdk/bsp/system/sys_man/include \
		-I$(SDKDIR)/sdk/bsp/osal \
		-I$(SDKDIR)/sdk/bsp/free_rtos/include \
		-I$(SDKDIR)/sdk/middleware/console/include
CFLAGS+=	-includecustom-config.h
LDFLAGS=	-Os -Xlinker --gc-sections -L$(SDKDIR)/sdk/bsp/misc \
		--specs=nano.specs --specs=nosys.specs \
		-Tldscripts/mem.ld -Tldscripts/sections.ld

all: $(OBJDIR)/$(PROJ).bin
	arm-none-eabi-size -B $(OBJDIR)/$(PROJ).elf

.PHONY: all flash clean scope

.SUFFIXES: .bin .elf

.elf.bin:
	arm-none-eabi-objcopy -O binary $< $@

#%.o: $(OBJDIR)/%.c
#.c.o:
#	arm-none-eabi-gcc -c $<

#sdk/bsp/startup/config.o: $(SDKDIR)/sdk/bsp/startup/config.c

$(OBJDIR)/%.o: %.c
	mkdir -p `dirname $@`
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)/%.o: $(SDKDIR)/%.c
	mkdir -p `dirname $@`
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)/%.o: $(SDKDIR)/%.S
	mkdir -p `dirname $@`
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)/$(PROJ).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS)

flash: all
	$(SDKDIR)/utilities/scripts/qspi/program_qspi_jtag.sh \
		$(OBJDIR)/$(PROJ).bin
	sleep 1
	$(SDKDIR)/utilities/scripts/qspi/reboot_device.sh

#	/opt/SEGGER/JLink/JLinkGDBServer -if swd -device Cortex-M0 -endian little -speed 8000 -port 2331 -swoport 2332 -telnetport 2333 -vd -ir -localhostonly 1 -log jlink.log -s &
#	$(SDKDIR)/binaries/cli_programmer --prod-id $(PROD_ID) \
#		gdbserver write_qspi 0 $(OBJDIR)/$(PROJ).bin
#	sleep 1

clean:
	-rm -r $(OBJDIR) $(CLEANFILES)

TOOLCHAINVER?=	5_4-2016q3-20160926
TOOLCHAINDIR?=	../gcc-arm-none-eabi-$(TOOLCHAINVER)
LIBCDIR?=	$(TOOLCHAINDIR)/src/newlib/newlib

scope:
	find . $(SDKDIR)/sdk $(LIBCDIR) -name '*.[chyl]' -print | cscope -bqki-
