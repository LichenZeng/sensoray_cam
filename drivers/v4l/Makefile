KERNEL_SRC= /lib/modules/$(shell uname -r)/build

ifeq ($(KERNELRELEASE),)
SUBDIR = $(shell pwd)
else 
SUBDIR = $(SUBDIRS)
endif

driver: modules
all: modules
modules_install: backup modinst
#v4l2_install: backup v4l2_inst




obj-m := s2255drv.o

s2255v-objs = s2255drv.o

EXTRA_CFLAGS += -Wall -O2


backup:
	mkdir -p kernel_backup
	find /lib/modules/$(shell uname -r)/ -name 's2255drv.ko' | xargs -I{} cp --parent {} kernel_backup


modules:
ifeq (,$(wildcard $(KERNEL_SRC)/include/media/videobuf2-core.h))
	echo "[videobuf1]"
	rm -f s2255drv.c
	cp s2255drv_vbuf1.c s2255drv.c
	$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(SUBDIR) $@
else
	echo "[videobuf2]"
	rm -f s2255drv.c
	cp s2255drv_vbuf2.c s2255drv.c
	$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(SUBDIR) $@
endif


modinst: 
	find /lib/modules/$(shell uname -r)/ -name 's2255drv.ko' | sudo xargs rm -f
	find /lib/modules/$(shell uname -r)/ -name 's2255v.ko' | sudo xargs rm -f
	find /lib/modules/$(shell uname -r)/ -name 's2255.ko' | sudo xargs rm -f
	sudo mkdir -p /lib/modules/$(shell uname -r)/extra
	sudo install f2255usb.bin /lib/firmware
	sudo install s2255drv.ko /lib/modules/$(shell uname -r)/extra
	sudo depmod -a

clean:
	sudo rm -rf $(s2255v-objs)

