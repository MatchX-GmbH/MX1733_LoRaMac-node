PROJ=		mx1733-loramac-node
OBJDIR?=	obj
SDKVER?=	1.0.14.1081
SDKDIR?=	../DA1468x_DA15xxx_SDK_$(SDKVER)
#PROD_ID?=	DA14681-00

ELFTARGET=	$(OBJDIR)/$(PROJ).elf
TARGET=		$(OBJDIR)/$(PROJ).bin
IMGTARGET=	$(OBJDIR)/$(PROJ).img
MAPTARGET=	$(OBJDIR)/$(PROJ).map

OBJS+=	$(OBJDIR)/main.o \
	$(OBJDIR)/ble/ble.o \
	$(OBJDIR)/hw/button.o \
	$(OBJDIR)/hw/cons.o \
	$(OBJDIR)/hw/i2c.o \
	$(OBJDIR)/hw/iox.o \
	$(OBJDIR)/hw/led.o \
	$(OBJDIR)/hw/power.o \
	$(OBJDIR)/lora/boards/mx1733/delay-board.o\
	$(OBJDIR)/lora/boards/mx1733/gpio-board.o\
	$(OBJDIR)/lora/boards/mx1733/mx1733.o\
	$(OBJDIR)/lora/boards/mx1733/rtc-board.o\
	$(OBJDIR)/lora/boards/mx1733/spi-board.o\
	$(OBJDIR)/lora/boards/mx1733/sx1276-board.o \
	$(OBJDIR)/lora/boards/mx1733/utilities.o\
	$(OBJDIR)/lora/mac/region/Region.o \
	$(OBJDIR)/lora/mac/region/RegionCommon.o \
	$(OBJDIR)/lora/mac/region/RegionEU868.o \
	$(OBJDIR)/lora/mac/LoRaMac.o \
	$(OBJDIR)/lora/mac/LoRaMacAdr.o \
	$(OBJDIR)/lora/mac/LoRaMacClassB.o \
	$(OBJDIR)/lora/mac/LoRaMacCommands.o \
	$(OBJDIR)/lora/mac/LoRaMacConfirmQueue.o \
	$(OBJDIR)/lora/mac/LoRaMacCrypto.o \
	$(OBJDIR)/lora/mac/LoRaMacParser.o \
	$(OBJDIR)/lora/mac/LoRaMacSerializer.o \
	$(OBJDIR)/lora/radio/sx1276/sx1276.o \
	$(OBJDIR)/lora/system/soft-se/aes.o \
	$(OBJDIR)/lora/system/soft-se/cmac.o \
	$(OBJDIR)/lora/system/soft-se/soft-se.o \
	$(OBJDIR)/lora/system/delay.o \
	$(OBJDIR)/lora/system/gpio.o \
	$(OBJDIR)/lora/system/systime.o \
	$(OBJDIR)/lora/system/timer.o \
	$(OBJDIR)/lora/ad_lora.o \
	$(OBJDIR)/lora/lora.o \
	$(OBJDIR)/lora/param.o \
	$(OBJDIR)/lora/proto.o \
	$(OBJDIR)/lora/upgrade.o \
	$(OBJDIR)/sensor/bat.o \
	$(OBJDIR)/sensor/gps.o \
	$(OBJDIR)/sensor/light.o \
	$(OBJDIR)/sensor/sensor.o \
	$(OBJDIR)/sensor/temp.o \
	$(OBJDIR)/strtonum.o

OBJS+=	$(OBJDIR)/sdk/bsp/startup/config.o \
	$(OBJDIR)/sdk/bsp/startup/startup_ARMCM0.o \
	$(OBJDIR)/sdk/bsp/startup/system_ARMCM0.o \
	$(OBJDIR)/sdk/bsp/startup/vector_table.o \
	$(OBJDIR)/sdk/bsp/adapters/src/ad_battery.o \
	$(OBJDIR)/sdk/bsp/adapters/src/ad_crypto.o \
	$(OBJDIR)/sdk/bsp/adapters/src/ad_flash.o \
	$(OBJDIR)/sdk/bsp/adapters/src/ad_gpadc.o \
	$(OBJDIR)/sdk/bsp/adapters/src/ad_nvms.o \
	$(OBJDIR)/sdk/bsp/adapters/src/ad_nvms_direct.o \
	$(OBJDIR)/sdk/bsp/adapters/src/ad_nvms_ves.o \
	$(OBJDIR)/sdk/bsp/adapters/src/ad_nvparam.o \
	$(OBJDIR)/sdk/bsp/adapters/src/ad_rf.o \
	$(OBJDIR)/sdk/bsp/adapters/src/ad_temp_sens.o \
	$(OBJDIR)/sdk/bsp/free_rtos/event_groups.o \
	$(OBJDIR)/sdk/bsp/free_rtos/list.o \
	$(OBJDIR)/sdk/bsp/free_rtos/portable/GCC/ARM_CM0/port.o \
	$(OBJDIR)/sdk/bsp/free_rtos/portable/MemMang/heap_4.o \
	$(OBJDIR)/sdk/bsp/free_rtos/queue.o \
	$(OBJDIR)/sdk/bsp/free_rtos/tasks.o \
	$(OBJDIR)/sdk/bsp/free_rtos/timers.o \
	$(OBJDIR)/sdk/bsp/memory/src/qspi_automode.o \
	$(OBJDIR)/sdk/bsp/osal/resmgmt.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_cpm.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_crypto.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_dma.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_ecc.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_ecc_curves.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_ecc_ucode.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_gpio.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_gpadc.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_hard_fault.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_i2c.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_otpc.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_qspi.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_rf.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_spi.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_tempsens.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_timer0.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_timer1.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_timer2.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_trng.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_uart.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_usb_charger.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_watchdog.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/hw_wkup.o \
	$(OBJDIR)/sdk/bsp/peripherals/src/sys_tcs.o \
	$(OBJDIR)/sdk/bsp/system/sys_man/sys_charger.o \
	$(OBJDIR)/sdk/bsp/system/sys_man/sys_clock_mgr.o \
	$(OBJDIR)/sdk/bsp/system/sys_man/sys_power_mgr.o \
	$(OBJDIR)/sdk/bsp/system/sys_man/sys_rtc.o \
	$(OBJDIR)/sdk/bsp/system/sys_man/sys_trng.o \
	$(OBJDIR)/sdk/bsp/system/sys_man/sys_watchdog.o \
	$(OBJDIR)/sdk/interfaces/ble/src/adapter/ad_ble.o \
	$(OBJDIR)/sdk/interfaces/ble/src/ble_common.o \
	$(OBJDIR)/sdk/interfaces/ble/src/ble_gap.o \
	$(OBJDIR)/sdk/interfaces/ble/src/ble_gatts.o \
	$(OBJDIR)/sdk/interfaces/ble/src/ble_l2cap.o \
	$(OBJDIR)/sdk/interfaces/ble/src/ble_storage.o \
	$(OBJDIR)/sdk/interfaces/ble/src/ble_uuid.o \
	$(OBJDIR)/sdk/interfaces/ble/src/manager/ble_mgr.o \
	$(OBJDIR)/sdk/interfaces/ble/src/manager/ble_mgr_ad_msg.o \
	$(OBJDIR)/sdk/interfaces/ble/src/manager/ble_mgr_cmd.o \
	$(OBJDIR)/sdk/interfaces/ble/src/manager/ble_mgr_common.o \
	$(OBJDIR)/sdk/interfaces/ble/src/manager/ble_mgr_gap.o \
	$(OBJDIR)/sdk/interfaces/ble/src/manager/ble_mgr_gattc.o \
	$(OBJDIR)/sdk/interfaces/ble/src/manager/ble_mgr_gatts.o \
	$(OBJDIR)/sdk/interfaces/ble/src/manager/ble_mgr_gtl.o \
	$(OBJDIR)/sdk/interfaces/ble/src/manager/ble_mgr_helper.o \
	$(OBJDIR)/sdk/interfaces/ble/src/manager/ble_mgr_l2cap.o \
	$(OBJDIR)/sdk/interfaces/ble/src/manager/storage.o \
	$(OBJDIR)/sdk/interfaces/ble/src/manager/storage_flash.o \
	$(OBJDIR)/sdk/interfaces/ble/src/stack/ip/ble/ll/src/rwble/rwble.o \
	$(OBJDIR)/sdk/interfaces/ble/src/stack/ip/ble/profiles/prf.o \
	$(OBJDIR)/sdk/interfaces/ble/src/stack/modules/nvds/src/nvds.o \
	$(OBJDIR)/sdk/interfaces/ble/src/stack/plf/black_orca/src/arch/main/ble/arch_main.o \
	$(OBJDIR)/sdk/interfaces/ble/src/stack/plf/black_orca/src/arch/main/ble/jump_table.o \
	$(OBJDIR)/sdk/interfaces/ble/src/stack/plf/black_orca/src/driver/rf/src/rf_ble_functions.o \
	$(OBJDIR)/sdk/interfaces/ble/src/util/list.o \
	$(OBJDIR)/sdk/interfaces/ble/src/util/queue.o \
	$(OBJDIR)/sdk/interfaces/ble_services/src/ble_service.o \
	$(OBJDIR)/sdk/interfaces/ble_services/src/dis.o \
	$(OBJDIR)/sdk/interfaces/ble_services/src/dlg_suota.o \
	$(OBJDIR)/sdk/interfaces/ble_services/src/ias.o \
	$(OBJDIR)/sdk/interfaces/ble_services/src/lls.o \
	$(OBJDIR)/sdk/interfaces/ble_services/src/tps.o \
	$(OBJDIR)/sdk/middleware/segger_tools/SEGGER/SEGGER_RTT.o \
	$(OBJDIR)/sdk/middleware/segger_tools/SEGGER/SEGGER_RTT_printf.o

CONFIG_H=	custom_config.h
DEPS=		$(OBJS:.o=.d)

LDSCRIPTS=	obj/mem.ld obj/sections.ld
LDSCRIPTFLAGS=	$(LDSCRIPTS:%=-T%)

CC=		arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb
CFLAGS+=	-std=gnu11 -Wall
CFLAGS+=	-Wno-expansion-to-defined -Wno-missing-field-initializers
CFLAGS+=	-g -Os -fsigned-char -ffunction-sections -fdata-sections -flto
CFLAGS+=	-Ddg_configBLACK_ORCA_IC_REV=BLACK_ORCA_IC_REV_A \
		-Ddg_configBLACK_ORCA_IC_STEP=BLACK_ORCA_IC_STEP_E \
		-DCONFIG_AT45DB011D=1 -DCONFIG_24LC256=1 -DCONFIG_FM75=1
CFLAGS+=	-DRELEASE_BUILD
CFLAGS+=	-I. -Ilora -Ilora/boards -Ilora/mac \
			-Ilora/radio -Ilora/radio/sx1276 \
			-Ilora/system -Ilora/system/soft-se \
		-I$(SDKDIR)/sdk/bsp/include -I$(SDKDIR)/sdk/bsp/config \
		-I$(SDKDIR)/sdk/bsp/peripherals/include \
		-I$(SDKDIR)/sdk/bsp/memory/include \
		-I$(SDKDIR)/sdk/bsp/adapters/include \
		-I$(SDKDIR)/sdk/bsp/system/sys_man/include \
		-I$(SDKDIR)/sdk/bsp/osal \
		-I$(SDKDIR)/sdk/interfaces/ble/config \
		-I$(SDKDIR)/sdk/interfaces/ble/include \
		-I$(SDKDIR)/sdk/interfaces/ble/include/adapter \
		-I$(SDKDIR)/sdk/interfaces/ble/include/manager \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/config \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/att \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/att/attc \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/att/attm \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/att/atts \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/gap \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/gap/gapc \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/gap/gapm \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/gatt \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/gatt/gattc \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/gatt/gattm \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/l2c/l2cc \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/smp \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/smp/smpc \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/host/smp/smpm \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/hl/src/rwble_hl \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/ll/src/controller/em \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/ll/src/controller/llc \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/ll/src/controller/lld \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/ll/src/controller/llm \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/ll/src/rwble \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ble/profiles \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/ea/api \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/em/api \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/ip/hci/api \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/modules/common/api \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/modules/dbg/api \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/modules/gtl/api \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/modules/h4tl/api \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/modules/ke/api \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/modules/ke/src \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/modules/nvds/api \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/modules/rwip/api \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/plf/black_orca/src/arch \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/plf/black_orca/src/arch/boot/armgcc_4_8 \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/plf/black_orca/src/arch/compiler/armgcc_4_8 \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/plf/black_orca/src/build/ble-full/reg/fw \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/plf/black_orca/src/driver/flash \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/plf/black_orca/src/driver/reg \
		-I$(SDKDIR)/sdk/interfaces/ble/src/stack/plf/black_orca/src/driver/rf/api \
		-I$(SDKDIR)/sdk/interfaces/ble_services/include \
		-I$(SDKDIR)/sdk/middleware/console/include \
		-I$(SDKDIR)/sdk/middleware/segger_tools/Config \
		-I$(SDKDIR)/sdk/middleware/segger_tools/SEGGER \
		-I$(SDKDIR)/sdk/bsp/free_rtos/include
CFLAGS+=	-include$(CONFIG_H)
WARNFLAGS=	-Wextra
LDFLAGS=	-g -Os -Xlinker --gc-sections -Xlinker -Map=$(MAPTARGET) \
		-fmessage-length=0 -fsigned-char -ffunction-sections \
		-fdata-sections -Wall \
		-L$(SDKDIR)/sdk/bsp/misc \
		-L$(SDKDIR)/sdk/interfaces/ble_stack/DA14681-01-Release \
		--specs=nano.specs --specs=nosys.specs \
		$(LDSCRIPTFLAGS)
LDADD=		-lble_stack_da14681_01 -lm

all: $(TARGET)
	arm-none-eabi-size -B $(ELFTARGET)

image: $(IMGTARGET)

.PHONY: all image install flash firstflash run clean scope

.SUFFIXES: .img .bin .elf

.elf.bin:
	arm-none-eabi-objcopy -O binary $< $@

.bin.img:
	$(SDKDIR)/utilities/scripts/suota/v11/mkimage.sh $< $@

#%.o: $(OBJDIR)/%.c
#.c.o:
#	arm-none-eabi-gcc -c $<

$(OBJS) $(LDSCRIPTS): $(CONFIG_H)

$(OBJDIR)/%.o: %.c
	mkdir -p `dirname $@`
	$(CC) $(CFLAGS) $(WARNFLAGS) -c -MMD -MP -MF"$(@:%.o=%.d)" -o $@ $<

$(OBJDIR)/%.o: $(SDKDIR)/%.c
	mkdir -p `dirname $@`
	$(CC) $(CFLAGS) -c -MMD -MP -MF"$(@:%.o=%.d)" -o $@ $<

$(OBJDIR)/%.o: $(SDKDIR)/%.S
	mkdir -p `dirname $@`
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)/%.ld: ldscripts/%.ld.h
	mkdir -p `dirname $@`
	$(CC) $(CFLAGS) -E -P -c "$<" -o "$@"

$(ELFTARGET): $(OBJS) $(LDSCRIPTS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDADD)

flash install: all
	$(SDKDIR)/utilities/scripts/suota/v11/initial_flash.sh --nobootloader $(TARGET)

firstflash: all
	$(SDKDIR)/utilities/scripts/suota/v11/initial_flash.sh $(TARGET)

erase:
	JLinkGDBServer -if swd -device Cortex-M0 -endian little -speed 4000 &
	$(SDKDIR)/binaries/cli_programmer gdbserver chip_erase_qspi

debug:
	JLinkGDBServer -if swd -device Cortex-M0 -endian little -speed 8000 \
		-port 2331 -swoport 2332 -telnetport 2333 -vd -ir \
		-localhostonly 1 -log jlink.log -s &
	arm-none-eabi-gdb \
		-ex 'target remote :2331' \
		-ex 'eval "monitor exec SetRTTAddr %p", &_SEGGER_RTT' \
		-ex 'monitor reset' \
		-ex continue \
		$(ELFTARGET)

clean:
	-rm -r $(OBJDIR) $(CLEANFILES)

TOOLCHAINVER?=	5_4-2016q3-20160926
TOOLCHAINDIR?=	../gcc-arm-none-eabi-$(TOOLCHAINVER)
LIBCDIR?=	$(TOOLCHAINDIR)/src/newlib/newlib

scope:
	find . $(SDKDIR)/sdk $(LIBCDIR) -name '*.[chyl]' -print | cscope -bqki-

-include $(DEPS)
