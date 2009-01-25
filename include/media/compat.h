/*
 * $Id: compat.h,v 1.44 2006/01/15 09:35:16 mchehab Exp $
 */

#ifndef _COMPAT_H
#define _COMPAT_H
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24)
#define	KERN_CONT	""
#endif

/* To allow I2C compatibility code to work */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
#include <linux/i2c-dev.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
#ifdef CONFIG_PROC_FS
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#endif
#endif

/* To allow alsa code to work */
#ifdef NEED_SOUND_DRIVER_H
#include <sound/driver.h>
#endif

#ifdef NEED_ALGO_CONTROL
#include <linux/i2c.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#define set_freezable()
#define cancel_delayed_work_sync cancel_rearming_delayed_work
#endif

#ifndef __pure
#  define __pure __attribute__((pure))
#endif

#ifndef I2C_M_IGNORE_NAK
# define I2C_M_IGNORE_NAK 0x1000
#endif

/* device_create/destroy added in 2.6.18 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
/* on older kernels, class_device_create will in turn be a compat macro */
# define device_create(a, b, c, d, e, f, g) class_device_create(a, NULL, c, b, d, e, f, g)
# define device_destroy(a, b) class_device_destroy(a, b)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#define IRQF_SHARED		SA_SHIRQ
#define IRQF_DISABLED		SA_INTERRUPT
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#define PCIAGP_FAIL 0
#define vmalloc_32_user(a) vmalloc_32(a)
#endif

#ifdef NEED_BOOL_TYPE
/* bool type and enum-based definition of true and false was added in 2.6.19 */
typedef int bool;
#define true 1
#define false 0
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
#define sony_pic_camera_command(a,b) sonypi_camera_command(a,b)

#define SONY_PIC_COMMAND_SETCAMERAAGC        SONYPI_COMMAND_SETCAMERAAGC
#define SONY_PIC_COMMAND_SETCAMERABRIGHTNESS SONYPI_COMMAND_SETCAMERABRIGHTNESS
#define SONY_PIC_COMMAND_SETCAMERACOLOR      SONYPI_COMMAND_SETCAMERACOLOR
#define SONY_PIC_COMMAND_SETCAMERACONTRAST   SONYPI_COMMAND_SETCAMERACONTRAST
#define SONY_PIC_COMMAND_SETCAMERAHUE        SONYPI_COMMAND_SETCAMERAHUE
#define SONY_PIC_COMMAND_SETCAMERAPICTURE    SONYPI_COMMAND_SETCAMERAPICTURE
#define SONY_PIC_COMMAND_SETCAMERASHARPNESS  SONYPI_COMMAND_SETCAMERASHARPNESS
#define SONY_PIC_COMMAND_SETCAMERA           SONYPI_COMMAND_SETCAMERA
#endif

/* pci_dev got a new revision field in 2.6.23-rc1 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23) && defined(LINUX_PCI_H)
/* Just make it easier to subsitute pci_dev->revision with
 * v4l_compat_pci_rev(pci_dev).  It's too bad there isn't some kind of context
 * sensitive macro in C that could do this for us.  */
static inline u8 v4l_compat_pci_rev(struct pci_dev *pci)
{ u8 rev; pci_read_config_byte(pci, PCI_REVISION_ID, &rev); return rev; }
#endif

#if defined(COMPAT_PCM_TO_RATE_BIT) && defined(__SOUND_PCM_H)
/* New alsa core utility function */
static inline unsigned int snd_pcm_rate_to_rate_bit(unsigned int rate)
{
	static const unsigned int rates[] = { 5512, 8000, 11025, 16000, 22050,
		32000, 44100, 48000, 64000, 88200, 96000, 176400, 192000 };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rates); i++)
		if (rates[i] == rate)
			return 1u << i;
	return SNDRV_PCM_RATE_KNOT;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
# define task_pid_nr(current) ((current)->pid)

# define sg_init_table(a,b)
# define sg_page(p) (sg->page)
# define sg_set_page(sglist,pg,sz,off)					\
do {									\
	struct scatterlist *p=sglist;					\
	p->page   = pg;							\
	p->length = sz;							\
	p->offset = off;						\
} while (0)

#define pr_err(fmt, arg...) \
	printk(KERN_ERR fmt, ##arg)
#endif

#ifndef BIT_MASK
# define BIT_MASK(nr)            (1UL << ((nr) % BITS_PER_LONG))
# define BIT_WORD(nr)            ((nr) / BITS_PER_LONG)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
#define i2c_verify_client(dev)	\
	((dev->bus == &i2c_bus_type) ? to_i2c_client(dev) : NULL)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
#define i2c_verify_client(dev) \
	((dev->bus && 0 == strcmp(dev->bus->name, "i2c")) ? to_i2c_client(dev) : NULL)
#endif

#ifndef USB_DEVICE_AND_INTERFACE_INFO
# define USB_DEVICE_AND_INTERFACE_INFO(vend,prod,cl,sc,pr) \
	.match_flags = USB_DEVICE_ID_MATCH_INT_INFO \
		| USB_DEVICE_ID_MATCH_DEVICE, \
	.idVendor = (vend), .idProduct = (prod), \
	.bInterfaceClass = (cl), \
	.bInterfaceSubClass = (sc), .bInterfaceProtocol = (pr)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
#define get_unaligned_be16(a)					\
	be16_to_cpu(get_unaligned((unsigned short *)(a)))
#define put_unaligned_be16(r, a)				\
	put_unaligned(cpu_to_be16(r), ((unsigned short *)(a)))
#define get_unaligned_le16(a)					\
	le16_to_cpu(get_unaligned((unsigned short *)(a)))
#define put_unaligned_le16(r, a)				\
	put_unaligned(cpu_to_le16(r), ((unsigned short *)(a)))
#define get_unaligned_be32(a)					\
	be32_to_cpu(get_unaligned((u32 *)(a)))
#define put_unaligned_be32(r, a)				\
	put_unaligned(cpu_to_be32(r), ((u32 *)(a)))
#define get_unaligned_le32(a)					\
	le32_to_cpu(get_unaligned((u32 *)(a)))
#define put_unaligned_le32(r, a)				\
	put_unaligned(cpu_to_le32(r), ((u32 *)(a)))
#define get_unaligned_le64(a)					\
	le64_to_cpu(get_unaligned((u64 *)(a)))
#define put_unaligned_le64(r, a)				\
	put_unaligned(cpu_to_le64(r), ((u64 *)(a)))
#endif

#ifdef NEED_PROC_CREATE
#ifdef CONFIG_PROC_FS
static inline struct proc_dir_entry *proc_create(const char *a,
	mode_t b, struct proc_dir_entry *c, const struct file_operations *d)
{
	struct proc_dir_entry *e;

	e = create_proc_entry(a, b, c);
	if (e) {
		e->owner = THIS_MODULE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 17)
		e->proc_fops = d;
#else
		e->proc_fops = (struct file_operations *)d;
#endif
	}
	return e;
}
#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
#ifdef CONFIG_PROC_FS
static inline struct proc_dir_entry *proc_create_data(const char *a,
	mode_t b, struct proc_dir_entry *c, const struct file_operations *d,
	void *f)
{
	struct proc_dir_entry *e;

	e = create_proc_entry(a, b, c);
	if (e) {
		e->owner = THIS_MODULE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 17)
		e->proc_fops = d;
#else
		e->proc_fops = (struct file_operations *)d;
#endif
		e->data = f;
	}
	return e;
}
#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
#define hweight64(x)  generic_hweight64(x)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24)
typedef unsigned long uintptr_t;
#endif

#ifdef NEED_IS_SINGULAR
static inline int list_is_singular(const struct list_head *head)
{
	return !list_empty(head) && (head->next == head->prev);
}
#endif

#ifdef NEED_CLAMP
#define clamp( x, l, h )        max_t( __typeof__( x ),		\
				      ( l ),			\
				      min_t( __typeof__( x ),	\
					     ( h ),        	\
					     ( x ) ) )
#endif

#ifdef NEED_ALGO_CONTROL
static inline int dummy_algo_control(struct i2c_adapter *adapter,
			     unsigned int cmd, unsigned long arg)
{
	return 0;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
#define div64_u64(a,b) div64_64(a,b)

#define dev_name(dev)	((dev)->bus_id)

#define dev_set_name(dev, fmt, arg...) ({	\
	snprintf((dev)->bus_id, sizeof((dev)->bus_id), fmt , ## arg); \
	0;					\
})
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#define current_uid() (current->uid)
#endif

#ifndef WARN
#define WARN(condition, format...) ({					\
	int __ret_warn_on = !!(condition);				\
	if (unlikely(__ret_warn_on))					\
		printk(KERN_WARNING format);				\
	unlikely(__ret_warn_on);					\
})
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 28)

#define pci_ioremap_bar(pci, a)				\
	 ioremap_nocache(pci_resource_start(pci, a),	\
			 pci_resource_len(pci, a))
#endif

#ifndef PCI_DEVICE_ID_MARVELL_88ALP01_CCIC
#define PCI_DEVICE_ID_MARVELL_88ALP01_CCIC     0x4102
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
#ifdef __LINUX_USB_H
/*
 * usb_endpoint_* functions
 *
 * Included in Linux 2.6.19
 * Backported to 2.6.18 in Red Hat Enterprise Linux 5.2
 */

#ifdef RHEL_RELEASE_CODE
#if RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(5, 2)
#define RHEL_HAS_USB_ENDPOINT
#endif
#endif

#ifndef RHEL_HAS_USB_ENDPOINT
static inline int
usb_endpoint_dir_in(const struct usb_endpoint_descriptor *epd)
{
	return (epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN;
}

static inline int
usb_endpoint_xfer_int(const struct usb_endpoint_descriptor *epd)
{
	return (epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		USB_ENDPOINT_XFER_INT;
}

static inline int
usb_endpoint_xfer_isoc(const struct usb_endpoint_descriptor *epd)
{
	return (epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		USB_ENDPOINT_XFER_ISOC;
}

static inline int
usb_endpoint_xfer_bulk(const struct usb_endpoint_descriptor *epd)
{
	return (epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		USB_ENDPOINT_XFER_BULK;
}

static inline int
usb_endpoint_is_int_in(const struct usb_endpoint_descriptor *epd)
{
	return usb_endpoint_xfer_int(epd) && usb_endpoint_dir_in(epd);
}
#endif /* RHEL_HAS_USB_ENDPOINT */
#endif /* __LINUX_USB_H */
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 22)
/*
 * Linked list API
 */
#define list_first_entry(ptr, type, member) \
	list_entry((ptr)->next, type, member)

/*
 * uninitialized_var() macro
 */
#define uninitialized_var(x) x = x
#endif

#endif
