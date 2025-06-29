obj-m := rt_monitor_module_spb.o

KERNELDIR ?= /usr/src/linux-headers-6.8.1-1023-realtime

PWD := $(shell pwd)

all: 
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules 

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
