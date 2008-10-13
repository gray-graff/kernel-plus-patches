#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

ccflags-y = -Iinclude/drm
mach64-y := mach64_drv.o mach64_dma.o mach64_irq.o mach64_state.o

obj-$(CONFIG_DRM_MACH64) += mach64.o
