#ifndef __ASM_ARCH_REGS_USB_H
#define __ASM_ARCH_REGS_USB_H

#define S3C2410_USBHREG(x) ((x) + S3C2410_VA_USBHOST)

#define S3C2410_UHC_REVISION		S3C2410_USBHREG(0x00)
#define S3C2410_UHC_CONTROL		S3C2410_USBHREG(0x04)
#define S3C2410_UHC_COMMON_STATUS	S3C2410_USBHREG(0x08)
#define S3C2410_UHC_INTERRUPT_STATUS	S3C2410_USBHREG(0x0c)
#define S3C2410_UHC_INTERRUPT_ENABLE	S3C2410_USBHREG(0x10)
#define S3C2410_UHC_INTERRUPT_DISABLE	S3C2410_USBHREG(0x14)

#define S3C2410_UHC_HCCA		S3C2410_USBHREG(0x18)

#endif


