/*
 * Definitions for the MOXA serial port adapter
 */



#define URB_TRANSFER_BUFFER_SIZE	64
#define URB_RECEIVE_BUFFER_SIZE		64

#define MOXA_TIMEOUT			( HZ * 5 )
#define CLOSE_WAIT			(5 * HZ)

#define PORTNO(x) (MINOR((x)->device) - (x)->driver.minor_start)

/* vendor/product pairs that are known to contain this chipset */
#define MOXA_VID                0x110A
#define NPort_1240_PID          0x1040
#define NPort_1220_PID          0x1320
#define NPort_1220I_PID         0x1321


/* Vendor commands: */
/* set_baud_div */
#define	MXU2S_BAUD_230400	16
#define MXU2S_BAUD_115200	32
#define MXU2S_BAUD_57600	64
#define MXU2S_BAUD_38400	96
#define MXU2S_BAUD_19200	192
#define MXU2S_BAUD_9600		384
#define MXU2S_BAUD_4800		768
#define MXU2S_BAUD_2400		1536
#define MXU2S_BAUD_1800		2048
#define MXU2S_BAUD_1200		3072
#define MXU2S_BAUD_600		6144

/* set_line_ctl */
#define MXU2S_DATABITS_8	0x08
#define MXU2S_DATABITS_7	0x07
#define MXU2S_DATABITS_6	0x06
#define MXU2S_DATABITS_5	0x05

#define MXU2S_STOPBITS_1	0x00
#define MXU2S_STOPBITS_15	0x01
#define MXU2S_STOPBITS_2	0x02

#define MXU2S_PARITY_NONE	0x00
#define MXU2S_PARITY_ODD	0x10
#define MXU2S_PARITY_EVEN	0x20
#define MXU2S_PARITY_MARK	0x30
#define MXU2S_PARITY_SPACE	0x40

/* control pipe messages */
#define MXU2S_IFC_ENABLE		0x00
#define MXU2S_SET_BAUDDIV		0x01
#define	MXU2S_SET_LINE_CTL		0x03
#define MXU2S_GET_LINE_CTL		0x04
#define MXU2S_SET_BREAK			0x05
#define MXU2S_IMM_CHAR			0x06
#define MXU2S_SET_MHS			0x07
#define MXU2S_GET_MDMSTS		0x08
#define MXU2S_SET_XON			0x09
#define MXU2S_SET_XOFF			0x0A
#define MXU2S_GET_PROPS			0x0F
#define	MXU2S_GET_COMM_STATUS		0x10
#define MXU2S_RESET			0x11
#define MXU2S_PURGE			0x12
#define MXU2S_SET_FLOW			0x13
#define MXU2S_GET_FLOW			0x14
#define MXU2S_GET_EVENT_STATE		0x16
#define MXU2S_GET_RECEIVE		0x18
#define MXU2S_SET_CHARS			0x19

/* control pipe messages value */
#define MXU2S_INTERFACE_ENABLE		0x01
#define MXU2S_INTERFACE_DISABLE		0x00

#define MXU2S_BREAK_SEND		0x01
#define MXU2S_BREAK_RESET		0x00

#define MXU2S_FLOW_ENABLE		0x01
#define MXU2S_FLOW_DISABLE		0x00

#define MXU2S_DTR_ON			0x0101
#define MXU2S_DTR_OFF			0x0100
#define MXU2S_RTS_ON			0x0202
#define MXU2S_RTS_OFF			0x0200

#define MXU2S_PURGE_WRITE		0x05
#define MXU2S_PURGE_READ		0x0A

/* mxnode ioctl */
#define MXNODE_GET_PORT_CFG		0x0400
#define MXNODE_GET_TOTAL_PORT		0x0401

/* Interpretation of modem status lines */
#define MXU2S_DTR	0x01
#define MXU2S_RTS	0x02
#define MXU2S_CTS	0x10
#define MXU2S_DSR	0x20
#define MXU2S_RI	0x40
#define MXU2S_DCD	0x80

#define MXU2S_TTY_PORTS		128
#define USB_TTY_PORTS		256

/* port table -- the chip supports up to 4 channels */

/* baud rates */

enum {
  kl5kusb105a_sio_b115200 = 0,
  kl5kusb105a_sio_b57600  = 1,
  kl5kusb105a_sio_b38400  = 2,
  kl5kusb105a_sio_b19200  = 4,
  kl5kusb105a_sio_b14400  = 5,
  kl5kusb105a_sio_b9600   = 6,
  kl5kusb105a_sio_b4800   = 8,   /* unchecked */
  kl5kusb105a_sio_b2400   = 9,   /* unchecked */
  kl5kusb105a_sio_b1200   = 0xa,  /* unchecked */
  kl5kusb105a_sio_b600    = 0xb   /* unchecked */
};

/* data bits */
#define kl5kusb105a_dtb_7   7
#define kl5kusb105a_dtb_8   8



/* requests: */
#define KL5KUSB105A_SIO_SET_DATA  1
#define KL5KUSB105A_SIO_POLL      2
#define KL5KUSB105A_SIO_CONFIGURE      3
/* values used for request KL5KUSB105A_SIO_CONFIGURE */
#define KL5KUSB105A_SIO_CONFIGURE_READ_ON      3
#define KL5KUSB105A_SIO_CONFIGURE_READ_OFF     2

/* Interpretation of modem status lines */
/* These need sorting out by individually connecting pins and checking
 * results. FIXME!
 * When data is being sent we see 0x30 in the lower byte; this must
 * contain DSR and CTS ...
 */
#define KL5KUSB105A_DSR			((1<<4) | (1<<5))
#define KL5KUSB105A_CTS			((1<<5) | (1<<4))

#define KL5KUSB105A_WANTS_TO_SEND	0x30
//#define KL5KUSB105A_DTR			/* Data Terminal Ready */
//#define KL5KUSB105A_CTS			/* Clear To Send */
//#define KL5KUSB105A_CD			/* Carrier Detect */
//#define KL5KUSB105A_DSR			/* Data Set Ready */
//#define KL5KUSB105A_RxD			/* Receive pin */

//#define KL5KUSB105A_LE
//#define KL5KUSB105A_RTS		
//#define KL5KUSB105A_ST		
//#define KL5KUSB105A_SR		
//#define KL5KUSB105A_RI			/* Ring Indicator */

/* vim: set ts=8 sts=8: */

