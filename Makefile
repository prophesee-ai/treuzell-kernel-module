# SPDX-License-Identifier: GPL-2.0
#
# USB gadget allowing to control an Asynchronous Time-Based Image Sensor (ATIS)
# interface, along with the interface driver, as a kernel module.
#

ccflags-y			:= -I$(srctree)/drivers/usb/gadget/
ccflags-y			+= -I$(srctree)/drivers/usb/gadget/udc/
ccflags-y			+= -I$(srctree)/drivers/usb/gadget/function/

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
