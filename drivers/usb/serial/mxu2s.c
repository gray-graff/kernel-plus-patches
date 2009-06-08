
/*
 * MOXA USB to Serial Hub Linux Driver
 *
 *   Copyright (C) 2002 Moxa Technologies Co., Ltd.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 * All information about the device was acquired using SniffUSB ans snoopUSB
 * on Windows98.
 * It was written out of frustration with the PalmConnect USB Serial adapter
 * sold by Palm Inc.
 *
 * It seems that KLSI bought some silicon-design information from ScanLogic, 
 * whose SL11R processor is at the core of the KL5KUSB chipset from KLSI.
 * KLSI has firmware available for their devices; it is probable that the
 * firmware differs from that used by KLSI in their products. If you have an
 * original KLSI device and can provide some information on it, I would be 
 * most interested in adding support for it here. If you have any information 
 * on the protocol used (or find errors in my reverse-engineered stuff), please
 * let me know.
 *
 * The code was only tested with a PalmConnect USB adapter; if you
 * are adventurous, try it with any KLSI-based device and let me know how it
 * breaks so that I can fix it!
 */

/* History:
 * Date		Author		Version		Comment
 * 06/04/2009	Michail Yakushin 1.0(2.6.29) 	Modify for kernel 2.6.29
 * 06/30/2005   Enzo Chen       1.0(2.6)        Modify for Kernel under 2.6.9
 * 12/30/2002   George Liu      1.0             Write it.
 */

/* TODO:
 *	check modem line signals
 *	implement handshaking or decide that we do not support it
 */

/* History:
 *   2.0  - change to the asynchronous usb_submit_urb/usb_unlink_urb interface
 *   0.3a - implemented pools of write URBs
 *   0.3  - alpha version for public testing
 *   0.2  - TIOCMGET works, so autopilot(1) can be used!
 *   0.1  - can be used to to pilot-xfer -p /dev/ttyUSB0 -l
 *
 *   The driver skeleton is mainly based on mct_u232.c and various other 
 *   pieces of code shamelessly copied from the drivers/usb/serial/ directory.
 */


#include <linux/autoconf.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <asm/string.h>

#ifdef CONFIG_USB_SERIAL_DEBUG
 	static int debug = 1;
#else
 	static int debug;
#endif

#include <linux/usb/serial.h>
#include "mxu2s.h"


/*
 * Version Information
 */
#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "Enzo Chen <enzo_chen@moxa.com.tw>"
#define DRIVER_DESC "MOXA USB to Serial Hub Driver"


/*
 * Function prototypes
 */
static int  mxu2s_startup		(struct usb_serial *serial);
static void mxu2s_shutdown		(struct usb_serial *serial);
static int  mxu2s_open			(struct tty_struct *, struct usb_serial_port *port,
					 struct file *filp);
static void mxu2s_close			(struct tty_struct *,struct usb_serial_port *port,
					 struct file *filp);
static int  mxu2s_write			(struct tty_struct *,struct usb_serial_port *port,
					 const unsigned char *buf,
					 int count);
static void mxu2s_write_bulk_callback	(struct urb *urb);
static int  mxu2s_chars_in_buffer	(struct tty_struct *);
static int  mxu2s_write_room		(struct tty_struct *);
static void mxu2s_read_bulk_callback	(struct urb *urb);
static void mxu2s_set_termios		(struct tty_struct *, struct usb_serial_port*,
					 struct ktermios * old);
static int  mxu2s_ioctl			(struct tty_struct *tty,
					 struct file * file,
					 unsigned int cmd,
					 unsigned long arg);
static void mxu2s_throttle		(struct tty_struct *tty);
static void mxu2s_unthrottle		(struct tty_struct *tty);
static int  mxu2s_tiocmget		(struct tty_struct *,
					 struct file *file);
static int  mxu2s_tiocmset		(struct tty_struct *tty,
					 struct file *file, unsigned int set,
					 unsigned int clear);
static int mxu2s_control_msg		(struct usb_serial_port *port, 
					 unsigned char request, 
					 unsigned char dirction,
					 unsigned int value, void *buf,
					 unsigned int buf_len);
static void mxu2s_wait_until_sent	(struct usb_serial_port *port, 
					 long timeout);
static int mxu2s_get_queue_value	(struct usb_serial_port *port);
static void mxu2s_break_ctl		(struct tty_struct *, 
					 int break_state);

/*
 * All of the device info needed for the KLSI converters.
 */
static struct usb_device_id mxu2s_idtable_combined [] = {
	{ USB_DEVICE(MOXA_VID, NPort_1240_PID) },
	{ USB_DEVICE(MOXA_VID, NPort_1220_PID) },
	{ USB_DEVICE(MOXA_VID, NPort_1220I_PID) },
	{ }		/* Terminating entry */
};

static struct usb_device_id mxu2s_np1240_ids [] = {
	{ USB_DEVICE(MOXA_VID, NPort_1240_PID) },
	{ }		/* Terminating entry */
};

static struct usb_device_id mxu2s_np1220_ids [] = {
	{ USB_DEVICE(MOXA_VID, NPort_1220_PID) },
	{ }		/* Terminating entry */
};

static struct usb_device_id mxu2s_np1220i_ids [] = {
	{ USB_DEVICE(MOXA_VID, NPort_1220I_PID) },
	{ }		/* Terminating entry */
};

MODULE_DEVICE_TABLE (usb, mxu2s_idtable_combined);

static struct usb_driver mxu2s_driver = {
	.name =		"mxu2sd",
	.probe =	usb_serial_probe,
	.disconnect =	usb_serial_disconnect,
	.id_table =	mxu2s_idtable_combined,
};

static struct usb_serial_driver mxu2s_np1240_device = {
	.driver = {
		.owner=THIS_MODULE,
		.name="np1240",
	},
	.id_table =		mxu2s_np1240_ids,
	.num_ports =		1,
	.open =			mxu2s_open,
	.close =		mxu2s_close,
	.write =		mxu2s_write,
	.write_bulk_callback =	mxu2s_write_bulk_callback,
	.chars_in_buffer =	mxu2s_chars_in_buffer,
	.write_room =		mxu2s_write_room,
	.ioctl =		mxu2s_ioctl,
	.set_termios =		mxu2s_set_termios,
	.break_ctl =		mxu2s_break_ctl,
	.tiocmget =		mxu2s_tiocmget,
	.tiocmset =		mxu2s_tiocmset,
	.attach =		mxu2s_startup,
	.shutdown =		mxu2s_shutdown,
	.throttle =		mxu2s_throttle,
	.unthrottle =		mxu2s_unthrottle,
};

static struct usb_serial_driver mxu2s_np1220_device = {
	.driver = {
		.owner=THIS_MODULE,
		.name="np1220",
	}
	,.id_table =		mxu2s_np1220_ids,
	.num_ports =		1,
	.open =			mxu2s_open,
	.close =		mxu2s_close,
	.write =		mxu2s_write,
	.write_bulk_callback =	mxu2s_write_bulk_callback,
	.chars_in_buffer =	mxu2s_chars_in_buffer,
	.write_room =		mxu2s_write_room,
	.ioctl =		mxu2s_ioctl,
	.set_termios =		mxu2s_set_termios,
	.break_ctl =		mxu2s_break_ctl,
	.tiocmget =		mxu2s_tiocmget,
	.tiocmset =		mxu2s_tiocmset,
	.attach =		mxu2s_startup,
	.shutdown =		mxu2s_shutdown,
	.throttle =		mxu2s_throttle,
	.unthrottle =		mxu2s_unthrottle,
};


static struct usb_serial_driver mxu2s_np1220I_device = {

	.driver = {
		.owner=THIS_MODULE,
		.name="np1220I",
	},
	.id_table =		mxu2s_np1220i_ids,
	.num_ports =		1,
	.open =			mxu2s_open,
	.close =		mxu2s_close,
	.write =		mxu2s_write,
	.write_bulk_callback =	mxu2s_write_bulk_callback,
	.chars_in_buffer =	mxu2s_chars_in_buffer,
	.write_room =		mxu2s_write_room,
	.ioctl =		mxu2s_ioctl,
	.set_termios =		mxu2s_set_termios,
	.break_ctl =		mxu2s_break_ctl,
	.tiocmget =		mxu2s_tiocmget,
	.tiocmset =		mxu2s_tiocmset,
	.attach =		mxu2s_startup,
	.shutdown =		mxu2s_shutdown,
	.throttle =		mxu2s_throttle,
	.unthrottle =		mxu2s_unthrottle,
};


struct mxu2s_port_settings {
	__u16	baudrate;
	__u8	databits;
	__u8	parity;
	__u8	stopbits;
	__u16	msr;
};

#define URB_TRANSFER_BUFFER_SIZE	64
#define	MXU2S_MAX_POOL			4	

struct mxu2s_private {
	struct usb_serial_port		*port;
	struct mxu2s_port_settings	cfg;
	struct termios			termios;
	unsigned long			line_state; /* modem line settings */
	spinlock_t			txlock;
	spinlock_t			rxlock;
	unsigned long			bytes_in;
	unsigned long			bytes_out;
	struct urb			*write_urb_pool[MXU2S_MAX_POOL];
	struct urb			*read_urb;
	unsigned char			portnum;
//	struct semaphore		sem;
	unsigned char			chars[6];
	unsigned char			setflow[16];
	wait_queue_head_t		wait_throttle;
	unsigned int			throttled;
	unsigned int			opened;
	unsigned int			chars_in_buf;
	unsigned int			write_changed;

};


/************************************************************************/
/************************************************************************/
/*            U S B  C A L L B A C K   F U N C T I O N S                */
/*            U S B  C A L L B A C K   F U N C T I O N S                */
/************************************************************************/
/************************************************************************/


/************************************************************************
 * Read line control via vendor command and return result through
 * *line_state_p
 ************************************************************************/
static int mxu2s_get_line_state (struct usb_serial *serial,
				 unsigned long *line_state_p)
{
	int rc;
	unsigned char status;

	rc = mxu2s_control_msg(serial->port[0], MXU2S_GET_MDMSTS,
		USB_DIR_IN, 0, &status, sizeof(unsigned char));
	if(rc < 0)
		err("Reading modem status failed (error = %d)", rc);

	*line_state_p = ((status & MXU2S_RTS) ? TIOCM_RTS : 0) |
		((status & MXU2S_DTR) ? TIOCM_DTR : 0) |
		((status & MXU2S_CTS) ? TIOCM_CTS : 0) |
		((status & MXU2S_DSR) ? TIOCM_DSR : 0) |
		((status & MXU2S_DCD) ? TIOCM_CAR : 0) |
		((status & MXU2S_RI ) ? TIOCM_RI : 0);

        return rc;
}


/************************************************************************
 * mxu2s_startup
 ************************************************************************/
static int mxu2s_startup (struct usb_serial *serial)
{
	struct mxu2s_private *priv;
	int i;
	unsigned char ep_addr;

	dbg("%s", __FUNCTION__);

	/* allocate the private data structure */
	for(i=0; i<serial->num_ports; i++)
	{
		priv = kmalloc(sizeof(struct mxu2s_private), GFP_KERNEL);
		if(!priv)
		{
			dbg("%s kmalloc for mxu2s_private failed.", 
				__FUNCTION__);
			return -ENOMEM;
		}
		memset(priv, 0, sizeof(struct mxu2s_private));
		/* set initial values for control structures */
		priv->bytes_in = 0;
		priv->bytes_out	= 0;
		priv->opened = 0;
		priv->throttled = 0;

  ep_addr = serial->interface->cur_altsetting->endpoint->desc.bEndpointAddress;

		if(ep_addr & 0x80)
			priv->portnum = ((ep_addr & 0x0F) / 2) - 1;
		else
			priv->portnum = ((ep_addr&0x0F) / 2);

		usb_set_serial_port_data(serial->port[i], priv);
		priv->port = serial->port[i];
//		init_MUTEX(&priv->sem);
		spin_lock_init (&priv->txlock);
		spin_lock_init (&priv->rxlock);

		/* priv->termios is left uninitalized until port opening */
		init_waitqueue_head(&serial->port[i]->write_wait);
	}

	return 0;
} /* mxu2s_startup */


/************************************************************************
 * mxu2s_shutdown
 ************************************************************************/
static void mxu2s_shutdown (struct usb_serial *serial)
{
	int i;
	struct mxu2s_private *priv;

	dbg("%s", __FUNCTION__);

	/* stop reads and writes on all ports */
	for(i=0; i<serial->num_ports; ++i)
	{
		priv = usb_get_serial_port_data(serial->port[i]);

		kfree(priv);
		usb_set_serial_port_data(serial->port[i], NULL);
	}
	if(usb_get_serial_data(serial) != NULL)
		kfree (usb_get_serial_data(serial));
	usb_set_serial_data(serial, NULL);
} /* mxu2s_shutdown */


/************************************************************************
 * mxu2s_open
 * this function is called by the tty driver when a port is opened
 * If successful, we return 0
 * Otherwise we return a negative error number.
 ************************************************************************/
static int  mxu2s_open (struct tty_struct *tty,struct usb_serial_port *port, struct file *filp)
{
	struct mxu2s_private *priv = usb_get_serial_port_data(port);
	int retval = 0;
	int rc;
	int i;

	dbg("%s port %d", __FUNCTION__, port->number);

//	if(port_paranoia_check(port, __FUNCTION__))
//		return -ENODEV;

	/* set up termios structure */
	priv->termios.c_iflag = tty->termios->c_iflag;
	priv->termios.c_oflag = tty->termios->c_oflag;
	priv->termios.c_cflag = tty->termios->c_cflag;
	priv->termios.c_lflag = tty->termios->c_lflag;
	for(i=0; i<NCCS; i++)
		priv->termios.c_cc[i] = tty->termios->c_cc[i];

	tty->termios->c_iflag &= ~(IXON | IXOFF);
	tty->termios->c_cflag = CREAD | B9600 | HUPCL | CLOCAL | CS8;

	priv->read_urb = port->read_urb;

	if(tty)
		tty->low_latency = 1;

	/* reset device */
	rc = mxu2s_control_msg(port, MXU2S_RESET, USB_DIR_OUT, 0, NULL, 0);
	if(rc < 0)
	{
		retval = rc;
		goto openerr;
	}
	
	/* enable interface */
	rc = mxu2s_control_msg(port, MXU2S_IFC_ENABLE, USB_DIR_OUT,
			MXU2S_INTERFACE_ENABLE, NULL, 0);
	if(rc < 0)
	{
		retval = rc;
		goto openerr;
	}

	priv->cfg.baudrate = (__u16)MXU2S_BAUD_9600;
	priv->cfg.databits = (__u8)MXU2S_DATABITS_8;
	priv->cfg.parity = (__u8)MXU2S_PARITY_NONE;
	priv->cfg.stopbits = (__u8)MXU2S_STOPBITS_1;
	priv->cfg.msr = (__u16)(MXU2S_RTS_ON | MXU2S_DTR_ON);
	priv->throttled = 0;
	priv->chars_in_buf = 0;
	priv->write_changed = 0;


	/* set CHARS */
	for(i=0; i<4; i++)
		priv->chars[i] = 0;
	priv->chars[4] = START_CHAR(tty);
	priv->chars[5] = STOP_CHAR(tty);
	rc = mxu2s_control_msg(port, MXU2S_SET_CHARS, USB_DIR_OUT,
		0, priv->chars, 6);

	if(rc < 0)
	{
		retval = rc;
		goto openerr;
	}

	for(i=0; i<16; i++)
		priv->setflow[i] = 0;
	priv->setflow[0] = 0x01;
	priv->setflow[3] = 0x80;
	priv->setflow[4] = 0x40;
	priv->setflow[8] = 0x50;
	priv->setflow[12] = 0xC8;

	/* set DTR & RTS */
	priv->cfg.msr = (MXU2S_DTR_ON|MXU2S_RTS_ON);
	rc = mxu2s_control_msg(port, MXU2S_SET_MHS,
		USB_DIR_OUT, priv->cfg.msr, NULL, 0);
	if(rc < 0)
	{
		retval = rc;
		goto openerr;
	}


	/* purge */
	rc = mxu2s_control_msg(port, MXU2S_PURGE, USB_DIR_OUT,
		(MXU2S_PURGE_READ | MXU2S_PURGE_WRITE), NULL, 0);

	if(port->read_urb->status != -EINPROGRESS)
	{
		port->read_urb->dev = port->serial->dev;
		port->read_urb->transfer_buffer_length = 
			URB_TRANSFER_BUFFER_SIZE;
		
		usb_fill_bulk_urb(port->read_urb, port->serial->dev,
			usb_rcvbulkpipe(port->serial->dev,
				port->bulk_in_endpointAddress),
			port->read_urb->transfer_buffer,
			port->read_urb->transfer_buffer_length,
			mxu2s_read_bulk_callback, port);
		rc = usb_submit_urb(port->read_urb, GFP_KERNEL);
		if(rc < 0)
		{
			err("%s - failed submitting read urb, error %d", 
				__FUNCTION__, rc);
			retval = rc;
			goto openerr;
		}
	}

	for(i=0; i<MXU2S_MAX_POOL; i++)
	{
		priv->write_urb_pool[i] = usb_alloc_urb(0, GFP_KERNEL);
		if(priv->write_urb_pool[i] == NULL)
			err("No more urbs???");

		priv->write_urb_pool[i]->transfer_buffer =
			kmalloc (URB_TRANSFER_BUFFER_SIZE, GFP_KERNEL);
		if(priv->write_urb_pool[i]->transfer_buffer == NULL)
		    err("%s - out of memory for urb buffers.", __FUNCTION__);
	}

	priv->opened = 1;

	return 0;

openerr:
	return retval;
} /* mxu2s_open */


/************************************************************************
 * mxu2s_close
 * this function is called by the tty driver when a port is closed
 ************************************************************************/
static void mxu2s_close (struct tty_struct *tty, struct usb_serial_port *port, struct file *filp)
{
	struct usb_serial *serial;
	struct mxu2s_private *priv = usb_get_serial_port_data(port);
	int rc;
	int i;

	dbg("%s port %d", __FUNCTION__, port->number);

//	serial = get_usb_serial (port, __FUNCTION__);
	serial = port->serial;

	if(!serial)
		return;

	mxu2s_wait_until_sent(port, CLOSE_WAIT);
	/* set purge */
	mxu2s_control_msg(port, MXU2S_PURGE, USB_DIR_OUT, 
			(MXU2S_PURGE_READ | MXU2S_PURGE_WRITE), NULL, 0);

	if(tty->ldisc.ops->flush_buffer)
		tty->ldisc.ops->flush_buffer(tty);


	rc = mxu2s_control_msg(port, MXU2S_IFC_ENABLE, USB_DIR_OUT,
                       MXU2S_INTERFACE_DISABLE, NULL, 0);
        if(rc < 0)
                err("Disable Interface failed (error = %d)", rc);

	/* shutdown our bulk reads and writes */
#if 0 
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,8))
	usb_kill_urb(port->write_urb);
#else
	usb_unlink_urb(port->write_urb);
#endif
#endif

	for(i=0; i<MXU2S_MAX_POOL; i++)
	{
		if(priv->write_urb_pool[i]==NULL)
			continue;
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,8))
		usb_kill_urb(priv->write_urb_pool[i]);
#else
		usb_unlink_urb(priv->write_urb_pool[i]);
#endif
		if(priv->write_urb_pool[i]->transfer_buffer)
			kfree(priv->write_urb_pool[i]->transfer_buffer);
		usb_free_urb(priv->write_urb_pool[i]);
		priv->write_urb_pool[i] = NULL;
        }

	if(serial->dev)
	{
		if(port->read_urb->status == 0)
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,8))
			usb_kill_urb(port->read_urb);
#else
			usb_unlink_urb(port->read_urb);
#endif
	}

	priv->opened = 0;

	dbg("mxu2s port stats: %ld bytes in, %ld bytes out", 
		priv->bytes_in, priv->bytes_out);

} /* mxu2s_close */


/************************************************************************
 * mxu2s_write
 * this function is called by the tty driver when data should be written to
 * the port.
 * f successful, we return the number of bytes written, otherwise we return
 * a negative error number.
 ************************************************************************/
static int mxu2s_write (struct tty_struct *tty,struct usb_serial_port *port, 
			const unsigned char *buf, int count)
{
	struct mxu2s_private *priv = usb_get_serial_port_data(port);
	int result, size;
	int bytes_sent=0;
	struct urb *urb = NULL;
	int i;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10))
	unsigned long flags;
#endif

	dbg("%s - port %d", __FUNCTION__, port->number);
/* 2.6.10 fixed kernel oops */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10))
	spin_lock_irqsave(&priv->txlock, flags);
#endif
	while(count > 0)
	{
#if 0
		if(port->write_urb->status != -EINPROGRESS)
		{
			urb = port->write_urb;
			is_inprogress = 0;
		}
		else
#endif 

		urb = NULL;
		for(i=0; i<MXU2S_MAX_POOL; i++)
		{
			if(priv->write_urb_pool[i]==NULL)
				continue;
			if(priv->write_urb_pool[i]->status != -EINPROGRESS)
			{
				urb = priv->write_urb_pool[i];
				break;
			}
		}

		if(urb==NULL)
		{
			dbg("%s - no more free urbs", __FUNCTION__);
			goto write_exit;
		}

		if(urb->transfer_buffer == NULL)
		{
			urb->transfer_buffer = 
				kmalloc(URB_TRANSFER_BUFFER_SIZE, GFP_ATOMIC);
			if(urb->transfer_buffer == NULL)
			{
				err("%s - no more kernel memory...", 
					__FUNCTION__);
				goto write_exit;
			}
		}

		size = min (count, port->bulk_out_size);
		size = min (size, URB_TRANSFER_BUFFER_SIZE);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
		if(from_user)
		{
			if(copy_from_user(urb->transfer_buffer, buf, size))
				return -EFAULT;
		}
		else
#endif
		{
			memcpy (urb->transfer_buffer, buf, size);
		}

		/* set up our urb */
		usb_fill_bulk_urb(urb, port->serial->dev,
			      usb_sndbulkpipe(port->serial->dev,
					      port->bulk_out_endpointAddress),
			      urb->transfer_buffer,
			      size,
			      mxu2s_write_bulk_callback,
			      port);
		result = usb_submit_urb(urb, GFP_ATOMIC);
		if (result)
		{
			err("%s - failed submitting write urb, error %d", 
				__FUNCTION__, result);

			goto write_exit;
		}

		buf += size;
		bytes_sent += size;
		count -= size;
	}
write_exit:
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10))
	spin_unlock_irqrestore(&priv->txlock, flags);
#endif
	/* lockless, but it's for debug info only... */
	priv->bytes_out += bytes_sent;
	priv->write_changed = 1;
	return bytes_sent;	/* that's how much we wrote */
} /* mxu2s_write */


/************************************************************************
 * mxu2s_write_bulk_callback
 * this is the callback function for when we have finished sending serial data
 * on the bulk out endpoint.
 ************************************************************************/
static void mxu2s_write_bulk_callback (struct urb *urb)
{
	struct usb_serial_port *port = (struct usb_serial_port *)urb->context;
	struct mxu2s_private *priv = usb_get_serial_port_data(port);

	dbg("%s - port %d", __FUNCTION__, port->number);
	
	if(urb->status)
	{
		dbg("%s - nonzero write bulk status received: %d", 
			__FUNCTION__, urb->status);
		return;
	}

	priv->write_changed = 2;

	/* from generic_write_bulk_callback */
	schedule_work(&port->work);
} /* mxu2s_write_bulk_callback */


/************************************************************************
 * mxu2s_chars_in_buffer
 * this function is called by the tty driver when it wants to know how many
 * bytes of data we currently have outstanding in the port (data that has
 * been written, but hasn't made it out the port yet)
 * If successful, we return the number of bytes left to be written in the
 * system,
 * Otherwise we return a negative error number.
 ************************************************************************/
static int mxu2s_chars_in_buffer (struct tty_struct *tty)
{
	int chars = 0;
	struct usb_serial_port *port=tty->driver_data;
	struct mxu2s_private *priv = usb_get_serial_port_data(port);
	int i;
	unsigned long flags;

	dbg("%s - port %d", __FUNCTION__, port->number);
#if 1 
	if(priv->write_changed != 0)
	{
		priv->chars_in_buf = mxu2s_get_queue_value(port);
	}
	chars += priv->chars_in_buf;
#endif

#if 0
	if(port->write_urb->status == -EINPROGRESS)
		chars += URB_TRANSFER_BUFFER_SIZE;
#endif

	spin_lock_irqsave(&priv->txlock, flags);
	for(i=0; i<MXU2S_MAX_POOL; i++)
	{
		if(priv->write_urb_pool[i]==NULL)
			continue;
		if(priv->write_urb_pool[i]->status == -EINPROGRESS)
			chars += URB_TRANSFER_BUFFER_SIZE;
	}
	spin_unlock_irqrestore(&priv->txlock, flags);

	dbg("%s - returns %d", __FUNCTION__, chars);
	return chars;
} /* mxu2s_chars_in_buffer */


/************************************************************************
 * mxu2s_write_room
 * this function is called by the tty driver when it wants to know how many
 * bytes of data we can accept for a specific port.
 * If successful, we return the amount of room that we have for this port
 * (the txCredits),
 * Otherwise we return a negative error number.
 ************************************************************************/
static int mxu2s_write_room (struct tty_struct *tty)
{
	int room = 0;
	struct usb_serial_port *port=tty->driver_data;
	struct mxu2s_private *priv = usb_get_serial_port_data(port);
	int i;
	unsigned long flags;

	dbg("%s - port %d", __FUNCTION__, port->number);
#if 0
	if((port->tty->stopped == 0) &&
		(port->write_urb->status != -EINPROGRESS))
	{
		room += URB_TRANSFER_BUFFER_SIZE;
	}
	else if (((priv->wp_head + 1) % MXU2S_MAX_POOL) != priv->wp_tail)
#endif

	spin_lock_irqsave(&priv->txlock, flags);
	for(i=0; i<MXU2S_MAX_POOL; i++)
	{
		if(priv->write_urb_pool[i]==NULL)
			continue;
		if(priv->write_urb_pool[i]->status != -EINPROGRESS)
			room += URB_TRANSFER_BUFFER_SIZE;
	}
	spin_unlock_irqrestore(&priv->txlock, flags);

	dbg("%s - returns %d", __FUNCTION__, room);

	return room;
} /* mxu2s_write_room */


/************************************************************************
 * mxu2s_read_bulk_callback
 * this is the callback function for when we have received data on the
 * bulk in endpoint.
 ************************************************************************/
static void mxu2s_read_bulk_callback (struct urb *urb)
{
	struct usb_serial_port *port = (struct usb_serial_port *)urb->context;
	struct usb_serial *serial = port->serial;
	struct mxu2s_private *priv = usb_get_serial_port_data(port);
	struct tty_struct *tty;
	unsigned char *data = urb->transfer_buffer;
	int rc;
	int i;
	int bytes_sent;


        dbg("%s - port %d", __FUNCTION__, port->number);

//	if(serial_paranoia_check (serial, __FUNCTION__))
//	{
//		dbg("%s - paranoia_check error", __FUNCTION__);
//		return;
//	}

	if(priv->opened == 0)
		return;

	/* The urb might have been killed. */
        if(urb->status)
	{
                dbg("%s - nonzero read bulk status received: %d", __FUNCTION__,
		    urb->status);
		if(urb->status != -EILSEQ)
	                return;
        }
	if(!serial)
	{
		dbg("%s - bad serial pointer, exiting", __FUNCTION__);
		return;
	}
	
	if(urb->actual_length != 0)
	{
		bytes_sent = urb->actual_length;

                
                /*
                tty layer threshold is 128 bytes, and the max
                data from usb is 64 bytes. Each time we call
                receive_buf will check and call throttle if
                needed. So I think we don't need to check
                receive_room.
                */
                /*
		if(tty->ldisc.receive_room(tty)<bytes_sent)
			printk("overrun\n");
		*/

		tty = tty_port_tty_get(&port->port);
		if (tty && urb->actual_length) {
			tty_buffer_request_room(tty, urb->actual_length + 1);
			/* overrun is special, not associated with a char */
			for (i = 0; i < urb->actual_length; ++i)
				tty_insert_flip_char(tty, data[i], TTY_NORMAL);
			tty_flip_buffer_push(tty);
		}
		tty_kref_put(tty);

		/* again lockless, but debug info only */
		priv->bytes_in += bytes_sent;
	/* Continue trying to always read  */
	}
	spin_lock(&priv->rxlock);
	if((port->read_urb->status != -EINPROGRESS) &&
		(priv->opened == 1) && (priv->throttled == 0))
	{
		rc = usb_submit_urb(port->read_urb, GFP_ATOMIC); 
		if(rc < 0)
		{
			err("%s - failed resubmitting read urb, error %d", 
				__FUNCTION__, rc);
		}
	}
	spin_unlock(&priv->rxlock);

} /* mxu2s_read_bulk_callback */


/************************************************************************
 * mxu2s_set_termios
 * this function is called by the tty driver when it wants to change
 * the termios structure
 ************************************************************************/
static void mxu2s_set_termios (struct tty_struct *tty,struct usb_serial_port *port,
				struct ktermios *old_termios)
{
	struct mxu2s_private *priv = usb_get_serial_port_data(port);
	unsigned int iflag = tty->termios->c_iflag;
//	unsigned int old_iflag = old_termios->c_iflag;
	unsigned int cflag = tty->termios->c_cflag;
	unsigned int old_cflag = old_termios->c_cflag;
	struct mxu2s_port_settings cfg;
	int rc;

	dbg("%s - port %d", __FUNCTION__, port->number);

	/*
	 * Update baud rate
	 */
//	if((cflag & (CBAUD | CBAUDEX)) != (old_cflag & (CBAUD | CBAUDEX)))
	{
	        /* reassert DTR and (maybe) RTS on transition from B0 */
		if((old_cflag & (CBAUD | CBAUDEX)) == B0)
			dbg("%s: baud was B0", __FUNCTION__);

		switch(cflag & (CBAUD | CBAUDEX))
		{
		case B0: /* handled below */
			priv->cfg.msr = (MXU2S_DTR_ON|MXU2S_RTS_ON);
			rc = mxu2s_control_msg(port, MXU2S_SET_MHS,
					USB_DIR_OUT, priv->cfg.msr, NULL, 0);
			if(rc < 0)
				err("Setting modem control failed (%d)", rc);
			break;
		case B600: priv->cfg.baudrate = MXU2S_BAUD_600;
			break;
		case B1200: priv->cfg.baudrate = MXU2S_BAUD_1200;
			break;
		case B1800: priv->cfg.baudrate = MXU2S_BAUD_1800;
			break;
		case B2400: priv->cfg.baudrate = MXU2S_BAUD_2400;
			break;
		case B4800: priv->cfg.baudrate = MXU2S_BAUD_4800;
			break;
		case B9600: priv->cfg.baudrate = MXU2S_BAUD_9600;
			break;
		case B19200: priv->cfg.baudrate = MXU2S_BAUD_19200;
			break;
		case B38400: priv->cfg.baudrate = MXU2S_BAUD_38400;
			break;
		case B57600: priv->cfg.baudrate = MXU2S_BAUD_57600;
			break;
		case B115200: priv->cfg.baudrate = MXU2S_BAUD_115200;
			break;
		case B230400: priv->cfg.baudrate = MXU2S_BAUD_230400;
			break;
		default:
			err(" MOXA USB->Serial converter:"
			    " unsupported baudrate request, using default"
			    " of 9600");
			priv->cfg.baudrate = MXU2S_BAUD_9600;
			break;
		}
		old_termios->c_cflag &= ~CBAUD;
		if(priv->cfg.baudrate == MXU2S_BAUD_9600)
			old_termios->c_cflag |= B9600;
		else
			old_termios->c_cflag |= (cflag & CBAUD);
		rc = mxu2s_control_msg(port, MXU2S_SET_BAUDDIV, USB_DIR_OUT,
				priv->cfg.baudrate, NULL, 0);

		if(rc < 0)
		{
			err("Change baudrate failed (error = %d)", rc);
			dbg("%s - baudrate %x", __FUNCTION__, 
				priv->cfg.baudrate);
		}
		dbg("%s - baudrate %d", __FUNCTION__, priv->cfg.baudrate);
	}

//	if((cflag & CSIZE) != (old_cflag & CSIZE))
	{
		/* set the number of data bits */
		switch(cflag & CSIZE)
		{
		case CS7:
			priv->cfg.databits = MXU2S_DATABITS_7;
			break;
		case CS8:
			priv->cfg.databits = MXU2S_DATABITS_8;
			break;
		default:
			err("CSIZE was not CS7-CS8, using default of 8");
			priv->cfg.databits = MXU2S_DATABITS_8;
			break;
		}
		old_termios->c_cflag &= ~CSIZE;
		old_termios->c_cflag |= (cflag & CSIZE);
	}

	/*
	 * Update line control register (LCR)
	 */
//	if((cflag & (PARENB|PARODD)) != (old_cflag & (PARENB|PARODD))
//	    || (cflag & CSTOPB) != (old_cflag & CSTOPB) )
	{
		priv->cfg.parity = 0;
		
		if(cflag & PARENB)
		{
			priv->cfg.parity |= (cflag & PARODD) ?
				MXU2S_PARITY_ODD : MXU2S_PARITY_EVEN;
		}
		else
		{
			priv->cfg.parity |= MXU2S_PARITY_NONE;
		}
		old_termios->c_cflag &= ~(PARENB|PARODD);
		old_termios->c_cflag |= (cflag & (PARENB|PARODD));
	}
	
//	if((old_cflag & CSTOPB) != (cflag & CSTOPB))
	{
		priv->cfg.stopbits |= (cflag & CSTOPB) ?
			MXU2S_STOPBITS_2 : MXU2S_STOPBITS_1;

		old_termios->c_cflag &= ~CSTOPB;
		old_termios->c_cflag |= (cflag & CSTOPB);
	}
	rc = mxu2s_control_msg(port, MXU2S_SET_LINE_CTL, USB_DIR_OUT, 
		(priv->cfg.databits<<8)|priv->cfg.parity|priv->cfg.stopbits,
		NULL,0);
	if(rc < 0)
	{
		err("Change port settings failed (error = %d)", rc);
		dbg("%s - databits %d, parity %d, stopbits %d", __FUNCTION__,
		    priv->cfg.databits, priv->cfg.parity, priv->cfg.stopbits);
		return;
	}

	/*
	 * Set flow control: well, I do not really now how to handle DTR/RTS.
	 * Just do what we have seen with SniffUSB on Win98.
	 */
//	if((iflag & IXOFF) != (old_iflag & IXOFF)
//	    || (iflag & IXON) != (old_iflag & IXON)
//	    ||  (cflag & CRTSCTS) != (old_cflag & CRTSCTS))
	{
		
//		for(i=0; i<16; i++)
//			priv->setflow[i] = 0;
//		priv->setflow[3] = 0x80;
//		priv->setflow[8] = 0x50;
//		priv->setflow[12] = 0xC8;

		if(cflag & CRTSCTS)
		{
			priv->setflow[0] |= 0x08;
			priv->setflow[4] |= 0x80;
			priv->setflow[4] &= ~0x40;
			old_termios->c_cflag |= CRTSCTS;
		}
		else
		{
			priv->setflow[0] &= ~0x08;
//			priv->setflow[4] &= ~0xC0;
			priv->setflow[4] &= ~0x80;
			priv->setflow[4] |= 0x40; // statically active
			old_termios->c_cflag &= ~CRTSCTS;
		}

		if((iflag & IXON) || (iflag & IXOFF))
		{
			priv->setflow[11] = 0x30;
			priv->setflow[15] = 0x30;
		}
		
		if(iflag & IXON)
		{
			priv->setflow[4] |= 0x01;
			old_termios->c_iflag |= IXON;
		}
		else
		{
			priv->setflow[4] &= ~(0x01);
			old_termios->c_iflag &= ~IXON;
		}

		if(iflag & IXOFF)
		{
			priv->setflow[4] |= 0x02;
			old_termios->c_iflag |= IXOFF;
		}
		else
		{
			priv->setflow[4] &= ~(0x02);
			old_termios->c_iflag &= ~IXOFF;
		}


		priv->setflow[0] |= 0x1;
		priv->setflow[0] &= ~0x2;

		rc = mxu2s_control_msg(port, MXU2S_SET_FLOW, USB_DIR_OUT,
				0, priv->setflow, 16);
		if(rc < 0)
			err("Setting flow control failed (error = %d)", rc);
		
	}

	memcpy (&cfg, &priv->cfg, sizeof(cfg));

} /* mxu2s_set_termios */


/************************************************************************
 * mxu2s_tiocmget
 ************************************************************************/
static int mxu2s_tiocmget (struct tty_struct *tty, struct file *file)
{
	struct usb_serial_port *port=tty->driver_data;
	struct usb_serial *serial = port->serial;
	struct mxu2s_private *priv = usb_get_serial_port_data(port);
	int rc;
	unsigned long line_state;

	dbg("%s - request, just guessing", __FUNCTION__);

	rc = mxu2s_get_line_state(serial, &line_state);
	if(rc < 0)
	{
		err("Reading line control failed (error = %d)", rc);
		/* better return value? EAGAIN? */
		return rc;
	}

	priv->line_state = line_state;
	dbg("%s - read line state 0x%lx", __FUNCTION__, line_state);
	return (int)line_state;
} /* mxu2s_tiocmget */


/************************************************************************
 * mxu2s_tiocmset
 ************************************************************************/
static int mxu2s_tiocmset (struct tty_struct *tty, struct file *file,
				unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;
	struct mxu2s_private *priv = usb_get_serial_port_data(port);
	int rc;
	
	dbg("%s", __FUNCTION__);

	if(set & TIOCM_RTS)
	{
		priv->line_state |= TIOCM_RTS;
		priv->setflow[4] |= 0x40;
		priv->setflow[4] &= ~0x80;
	}
	if (set & TIOCM_DTR)
	{
		priv->line_state |= TIOCM_DTR;
		priv->setflow[0] |= 0x01;
		priv->setflow[0] &= ~0x02;
	}
	if (clear & TIOCM_RTS)
	{
		priv->line_state &= ~TIOCM_RTS;
		priv->setflow[4] &= ~0xC0;
	}
	if (clear & TIOCM_DTR)
	{
		priv->line_state &= ~TIOCM_DTR;
		priv->setflow[0] &= ~0x03;
	}

	rc = mxu2s_control_msg(port, MXU2S_SET_FLOW, USB_DIR_OUT,
			0, priv->setflow, 16);
	if(rc < 0)
	{
		err("Setting flow control failed (error = %d)", rc);
		return -EINVAL;
	}

//	rc = mxu2s_control_msg(port, MXU2S_SET_MHS,
//		USB_DIR_OUT, priv->cfg.msr, NULL, 0);
//	if(rc < 0)
//	{
//		err("Setting MHS failed (error = %d)", rc);
//		return -EINVAL;
//	}

	return 0;
} /* mxu2s_tiocmset */

					
/************************************************************************
 * mxu2s_ioctl
 * this function handles any ioctl calls to the driver
 ************************************************************************/
static int mxu2s_ioctl (struct tty_struct *tty, struct file * file,
			unsigned int cmd, unsigned long arg)
{
	struct usb_serial_port *port=tty->driver_data;
	struct mxu2s_private *priv = usb_get_serial_port_data(port);
	int i;
	int rc;
	unsigned long flags;

	dbg("%scmd=0x%x", __FUNCTION__, cmd);

	/* Based on code from acm.c and others */
	switch (cmd) {
	case TIOCMIWAIT:
		/* wait for any of the 4 modem inputs (DCD,RI,DSR,CTS)*/
		/* TODO */
		dbg("%s - TIOCMIWAIT not handled", __FUNCTION__);
		return -ENOIOCTLCMD;

	case TIOCGICOUNT:
		/* return count of modemline transitions */
		/* TODO */
		dbg("%s - TIOCGICOUNT not handled", __FUNCTION__);
		return -ENOIOCTLCMD;

	case TCFLSH:
		switch(arg)
		{
		case TCIFLUSH:
			rc = mxu2s_control_msg(port, MXU2S_PURGE,
				USB_DIR_OUT, MXU2S_PURGE_READ, NULL, 0);
			if(rc < 0)
				err("Purge failed (error = %d)", rc);
			break;
		case TCIOFLUSH:
			spin_lock_irqsave(&priv->txlock, flags);
			for(i=0; i<MXU2S_MAX_POOL; i++)
			{
				if(priv->write_urb_pool[i]==NULL)
					continue;
				if(priv->write_urb_pool[i]->status != 
					-EINPROGRESS)
					continue;
				{
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,8))
					usb_kill_urb(priv->write_urb_pool[i]);
#else
					usb_unlink_urb(priv->write_urb_pool[i]);
#endif
				}
			}
			spin_unlock_irqrestore(&priv->txlock, flags);

			rc = mxu2s_control_msg(port, MXU2S_PURGE, USB_DIR_OUT, 
				(MXU2S_PURGE_READ | MXU2S_PURGE_WRITE),
				NULL, 0);
			if(rc < 0)
				err("Purge failed (error = %d)", rc);

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,8))
			/* cause unlink will be blocked, purge again */
			rc = mxu2s_control_msg(port, MXU2S_PURGE,
				USB_DIR_OUT, MXU2S_PURGE_WRITE, NULL, 0);
			if(rc < 0)
				err("Purge failed (error = %d)", rc);
#endif
			break;
		case TCOFLUSH:
			spin_lock_irqsave(&priv->txlock, flags);
			for(i=0; i<MXU2S_MAX_POOL; i++)
			{
				if(priv->write_urb_pool[i]==NULL)
					continue;
				if(priv->write_urb_pool[i]->status != 
					-EINPROGRESS)
				{
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,8))
					usb_kill_urb(priv->write_urb_pool[i]);
#else
					usb_unlink_urb(priv->write_urb_pool[i]);
#endif
				}
			}
			spin_unlock_irqrestore(&priv->txlock, flags);

			rc = mxu2s_control_msg(port, MXU2S_PURGE,
				USB_DIR_OUT, MXU2S_PURGE_WRITE, NULL, 0);
			if(rc < 0)
				err("Purge failed (error = %d)", rc);

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,8))	
			/* cause unlink will be blocked, purge again */
			rc = mxu2s_control_msg(port, MXU2S_PURGE,
				USB_DIR_OUT, MXU2S_PURGE_WRITE, NULL, 0);
			if(rc < 0)
				err("Purge failed (error = %d)", rc);
#endif
			break;
		}
		break;

	default:
		dbg("%s: arg not supported - 0x%04x", __FUNCTION__,cmd);
		return(-ENOIOCTLCMD);
		break;
	}
	return (-ENOIOCTLCMD);
} /* mxu2s_ioctl */


/************************************************************************
 * mxu2s_throttle
 * this function is called by the tty driver when it wants to stop the data
 * being read from the port.
 ************************************************************************/
static void mxu2s_throttle (struct tty_struct *tty)
{
	struct usb_serial_port *port=tty->driver_data;
	struct mxu2s_private *priv = usb_get_serial_port_data(port);

	dbg("%s - port %d", __FUNCTION__, port->number);

	if(!tty)
	{
		dbg ("%s - no tty available", __FUNCTION__);
		return ;
	}

	/* if we are implementing XON/XOFF, send the stop character */
//	if(I_IXOFF(tty))
	{
		priv->throttled = 1;
	}

#if 0
	/* if we are implementing RTS/CTS, toggle that line */
	if(tty->termios->c_cflag & CRTSCTS)
	{
		priv->setflow[4] &= ~0xC0;
		rc = mxu2s_control_msg(port, MXU2S_SET_FLOW, USB_DIR_OUT,
			0, priv->setflow, 16);
		if(rc < 0)
			err("Setting flow control failed (error = %d)", rc);
	}
#endif
} /* mxu2s_throttle */


/************************************************************************
 * mxu2s_unthrottle
 * this function is called by the tty driver when it wants to resume the data
 * being read from the port (called after SerialThrottle is called)
 ************************************************************************/
static void mxu2s_unthrottle (struct tty_struct *tty)
{
	struct usb_serial_port *port=tty->driver_data;
	struct mxu2s_private *priv = usb_get_serial_port_data(port);
	int rc;

	dbg("%s - port %d", __FUNCTION__, port->number);

	if(!tty)
	{
		dbg("%s - no tty available", __FUNCTION__);
		return ;
	}

	/* if we are implementing XON/XOFF, send the start character */
//	if(I_IXOFF(tty))
	{
		priv->throttled = 0;

		if(port->read_urb->status != -EINPROGRESS)
		{
			rc = usb_submit_urb(port->read_urb, GFP_KERNEL);
			if(rc < 0)
				dbg("unthrottle submit urb error\n");
		}
	}

#if 0
	/* if we are implementing RTS/CTS, toggle that line */
	if(tty->termios->c_cflag & CRTSCTS)
	{
		priv->setflow[4] |= 0x80;
		rc = mxu2s_control_msg(port, MXU2S_SET_FLOW, USB_DIR_OUT,
			0, priv->setflow, 16);
		if(rc < 0)
			dbg("Setting flow control failed (error = %d)", rc);
	}
#endif
} /* mxu2s_unthrottle */


/************************************************************************
 * mxu2s_break_ctl
 * this function sends a break to the port
 ************************************************************************/
static void mxu2s_break_ctl (struct tty_struct *tty, int break_state)
{
	int rc;
	struct usb_serial_port *port=tty->driver_data;
	dbg("%s port %d", __FUNCTION__, port->number);

	rc = mxu2s_control_msg(port, MXU2S_SET_BREAK, USB_DIR_OUT,
		(break_state ? MXU2S_BREAK_SEND : MXU2S_BREAK_RESET),
		NULL, 0);
	if(rc < 0)
		dbg("Set break failed (error = %d)", rc);
} /* mxu2s_break_ctl */



/***********************************************************************/
/***********************************************************************/
/***********************************************************************/

/************************************************************************
 * Handle vendor specific USB requests
 ************************************************************************/

/************************************************************************
 * mxu2s_control_msg
 * submit control messages
 ************************************************************************/
static int mxu2s_control_msg (struct usb_serial_port *port, 
	unsigned char request, unsigned char direction, 
	unsigned int value, void *buf, unsigned int buf_len)
{
	struct usb_serial *serial = port->serial;
	int rc;
	struct mxu2s_private *priv = usb_get_serial_port_data(port);

//	cli();

	if((direction & USB_DIR_IN))
	{
		rc = usb_control_msg(serial->dev, 
			usb_rcvctrlpipe(serial->dev, 0), request, 
			direction | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,
			value, priv->portnum, buf, buf_len, MOXA_TIMEOUT);
	}
	else
	{
		rc = usb_control_msg(serial->dev,
			usb_sndctrlpipe(serial->dev, 0), request,
			direction | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,
			value, priv->portnum, buf, buf_len, MOXA_TIMEOUT);
	}

//	sti();

	if(rc < 0)
		dbg("Send control message %d failed (error = %d)", request, rc);

	return rc;
} /* mxu2s_control_msg */


/************************************************************************
 * mxu2s_wait_until_sent
 * wait until data in out FIFO or urb have been sent
 ************************************************************************/
static void mxu2s_wait_until_sent (struct usb_serial_port *port, long timeout)
{
	int len,old_len=0;

	if(!timeout)
		timeout = MAX_SCHEDULE_TIMEOUT;
	
	len = mxu2s_get_queue_value(port);
	if(len <= 0)
		return;
	
	while(1)
	{
		if(len && len != old_len)
		{
			timeout = jiffies + timeout;
			old_len = len;
		}
		schedule_timeout(1);
		if(jiffies > timeout)
			break;
		len = mxu2s_get_queue_value(port);
		if(len <= 0)
			break;
	}
	return;
} /* mxu2s_wait_until_sent */


/************************************************************************
 * mxu2s_get_queue_value
 * check the FIFO on hardware
 ************************************************************************/
static int mxu2s_get_queue_value (struct usb_serial_port *port)
{
	int rc;
	unsigned char comm[16];
	int try;
	int i;

	for(try = 0; try < 10; try++)
	{
		rc = mxu2s_control_msg(port, MXU2S_GET_COMM_STATUS,
			USB_DIR_IN, 0, comm, 16);

		if(rc >= 0)
			break;

		for(i=0; i<1000; i++);
		// software delay, may slowdown throughput
	}

	if(rc < 0)
	{
		err("Get comm status failed (error = %d)", rc);
		return rc;
	}

	return (int)comm[12];
} /* mxu2s_get_queue_value */


/************************************************************************
 * mxu2s_init
 * This is called by the module subsystem, or on startup to initialize us
 ************************************************************************/
static int __init mxu2s_init (void)
{
	int retval;

	retval = usb_serial_register(&mxu2s_np1240_device);
	if(retval)
		goto failed_np1240_device_register;
	retval = usb_serial_register(&mxu2s_np1220_device);
	if(retval)
		goto failed_np1220_device_register;
	retval = usb_serial_register(&mxu2s_np1220I_device);
	if(retval)
		goto failed_np1220i_device_register;
	retval = usb_register(&mxu2s_driver);
	if(retval)
		goto failed_moxa_device_register;

	dbg(DRIVER_DESC " " DRIVER_VERSION);
	return 0;

failed_moxa_device_register:
	usb_serial_deregister(&mxu2s_np1220I_device);
failed_np1220i_device_register:
	usb_serial_deregister(&mxu2s_np1220_device);
failed_np1220_device_register:
	usb_serial_deregister(&mxu2s_np1240_device);
failed_np1240_device_register:

	return retval;
} /* mxu2s_init */


/************************************************************************
 * mxu2s_exit
 * Called when the driver is about to be unloaded.
 ************************************************************************/
static void __exit mxu2s_exit (void)
{
	usb_deregister(&mxu2s_driver);
	usb_serial_deregister(&mxu2s_np1240_device);
	usb_serial_deregister(&mxu2s_np1220_device);
	usb_serial_deregister(&mxu2s_np1220I_device);
} /* mxu2s_exit */


module_init (mxu2s_init);
module_exit (mxu2s_exit);

MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL"); 


module_param(debug, int,S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(debug, "enable extensive debugging messages");


