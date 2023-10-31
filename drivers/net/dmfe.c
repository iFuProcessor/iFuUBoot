// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2007-2009 Michal Simek
 * (C) Copyright 2003 Xilinx Inc.
 *
 * Michal SIMEK <monstr@monstr.eu>
 */

#include <common.h>
#include <net.h>
#include <config.h>
#include <dm.h>
#include <console.h>
#include <malloc.h>
#include <asm/io.h>
#include <fdtdec.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <asm/addrspace.h>


//#define MAC_DEBUG
#ifdef MAC_DEBUG
#define pr_debug(...)	printf(__VA_ARGS__)
#else
#define pr_debug(...)
#endif

/*
 * NOTE!  On the Alpha, we have an alignment constraint.  The
 * card DMAs the packet immediately following the RFA.  However,
 * the first thing in the packet is a 14-byte Ethernet header.
 * This means that the packet is misaligned.  To compensate,
 * we actually offset the RFA 2 bytes into the cluster.  This
 * aligns the packet after the Ethernet header at a 32-bit
 * boundary.  HOWEVER!  This means that the RFA is misaligned!
 */

#ifdef BADPCIBRIDGE
#define BADPCIBRIDGE
#define	RFA_ALIGNMENT_FUDGE	4
#else
#define	RFA_ALIGNMENT_FUDGE	2
#endif

#undef DEBUG_SROM
#undef DEBUG_SROM2

#undef UPDATE_SROM

/* PCI Registers.
 */
#define PCI_CFDA_PSM		0x43

#define CFRV_RN		0x000000f0	/* Revision Number */

#define WAKEUP		0x00		/* Power Saving Wakeup */
#define SLEEP		0x80		/* Power Saving Sleep Mode */

#define DC2114x_BRK	0x0020		/* CFRV break between DC21142 & DC21143 */

/* Ethernet chip registers.
 */
#define DE4X5_BMR	0x000		/* Bus Mode Register, CSR0 */
#define DE4X5_TPD	0x008		/* Transmit Poll Demand Reg, CSR1 */
#define CSR2		0x010		
#define DE4X5_RRBA	0x018		/* RX Ring Base Address Reg, CSR3*/
#define DE4X5_TRBA	0x020		/* TX Ring Base Address Reg, CSR4*/
#define DE4X5_STS	0x028		/* Status Register, CSR5 */
#define DE4X5_OMR	0x030		/* Operation Mode Register, CSR6 */
#define DE4X5_SICR	0x068		/* SIA Connectivity Register, */
#define DE4X5_APROM	0x048		/* Ethernet Address PROM */

/* Register bits.
 */
#define BMR_SWR		0x00000001	/* Software Reset */
#define STS_TS		0x00700000	/* Transmit Process State */
#define STS_RS		0x000e0000	/* Receive Process State */
#define OMR_ST		0x00002000	/* Start/Stop Transmission Command */
#define OMR_SR		0x00000002	/* Start/Stop Receive */
#define OMR_PS		0x00040000	/* Port Select */
#define OMR_SDP		0x02000000	/* SD Polarity - MUST BE ASSERTED */
#define OMR_PM		0x00000080	/* Pass All Multicast */
#define OMR_FD		0x00000200	/* full duplex */
#define OMR_PR		0x00000040	/* promise mode */
#define OMR_TTM   0x00400000  /* 10M */
#define OMR_SF    0x00200000  /*     */

/* Descriptor bits.
 */
#define R_OWN		0x80000000	/* Own Bit */
#define RD_RER		0x02000000	/* Receive End Of Ring */
#define RD_LS		0x00000100	/* Last Descriptor */
#define RD_ES		0x00008000	/* Error Summary */
#define TD_TER		0x02000000	/* Transmit End Of Ring */
#define TD_TCH		0x01000000	/* Second address chained */
#define T_OWN		0x80000000	/* Own Bit */
#define TD_LS		0x40000000	/* Last Segment */
#define TD_FS		0x20000000	/* First Segment */
#define TD_ES		0x00008000	/* Error Summary */
#define TD_SET		0x08000000	/* Setup Packet */

/* */


/* CR9 definition: SROM/MII */
#define DCR9           0x48
#define CR9_SROM_READ   0x4800
#define CR9_SRCS        0x1
#define CR9_SRCLK       0x2
#define CR9_CRDOUT      0x8
#define SROM_DATA_0     0x0
#define SROM_DATA_1     0x4
#define PHY_DATA_1      0x20000
#define PHY_DATA_0      0x00000
#define MDCLKH          0x10000

#define PHY_POWER_DOWN  0x800

#define SROM_V41_CODE   0x14


/* The EEPROM commands include the alway-set leading bit. */
#define SROM_WRITE_CMD	5
#define SROM_READ_CMD	6
#define SROM_ERASE_CMD	7

#define SROM_HWADD	    0x0014	/* Hardware Address offset in SROM */
#define SROM_RD		0x00004000	/* Read from Boot ROM */
#define EE_DATA_WRITE	      0x04	/* EEPROM chip data in. */
#define EE_WRITE_0	    0x4801
#define EE_WRITE_1	    0x4805
#define EE_DATA_READ	      0x08	/* EEPROM chip data out. */
#define SROM_SR		0x00000800	/* Select Serial ROM when set */

#define DT_IN		0x00000004	/* Serial Data In */
#define DT_CLK		0x00000002	/* Serial ROM Clock */
#define DT_CS		0x00000001	/* Serial ROM Chip Select */

#define POLL_DEMAND	1


typedef int  s32;

struct eth_device {
	char dev_addr[6];
    u8   addr_len;   /* hardware address length  */
	u32 iobase;
	char *packet;
} ;
#define udelay udelay



#define CONFIG_TULIP_FIX_DAVICOM

#ifdef CONFIG_TULIP_FIX_DAVICOM
#define RESET_DM9102(dev) {\
    unsigned long i;\
    i=INL(dev, 0x0);\
    udelay(1000);\
    OUTL(dev, i | BMR_SWR, DE4X5_BMR);\
    udelay(1000);\
}
#else
#define RESET_DE4X5(dev) {\
    int i;\
    i=INL(dev, DE4X5_BMR);\
    udelay(1000);\
    OUTL(dev, i | BMR_SWR, DE4X5_BMR);\
    udelay(1000);\
    OUTL(dev, i, DE4X5_BMR);\
    udelay(1000);\
    for (i=0;i<5;i++) {INL(dev, DE4X5_BMR); udelay(10000);}\
    udelay(1000);\
}
#endif

#define START_DE4X5(dev) {\
    s32 omr; \
    omr = INL(dev, DE4X5_OMR);\
    omr |= OMR_ST | OMR_SR | OMR_PR | OMR_FD | OMR_TTM | OMR_SF;\
    OUTL(dev, omr, DE4X5_OMR);		/* Enable the TX and/or RX */\
}

#define STOP_DE4X5(dev) {\
    s32 omr; \
    omr = INL(dev, DE4X5_OMR);\
    omr &= ~(OMR_ST|OMR_SR);\
    OUTL(dev, omr, DE4X5_OMR);		/* Disable the TX and/or RX */ \
}

#define NUM_RX_DESC 8			/* Number of Rx descriptors */
#define NUM_TX_DESC 8			/* Number of TX descriptors */
#define RX_BUFF_SZ  1520 		//yanhua, should aligned to word
#define TX_BUFF_SZ  1520

#define TOUT_LOOP   100000

#define SETUP_FRAME_LEN 192
#define ETH_ALEN	6

struct de4x5_desc {
	volatile s32 status;
	u32 des1;
	u32 buf;
	u32 next;
};

static struct de4x5_desc _rx_ring[NUM_RX_DESC] __attribute__ ((aligned(32),section(".bss.align32"))); /* RX descriptor ring         */
static struct de4x5_desc _tx_ring[NUM_TX_DESC] __attribute__ ((aligned(32),section(".bss.align32"))); /* TX descriptor ring         */
static volatile struct de4x5_desc *rx_ring;
static volatile struct de4x5_desc *tx_ring;

char __NetRxPackets[NUM_RX_DESC][RX_BUFF_SZ] __attribute__((aligned(32),section(".bss.align32"))); //16
char (*NetRxPackets)[RX_BUFF_SZ];

char __NetTxPackets[NUM_TX_DESC][TX_BUFF_SZ] __attribute__((aligned(32),section(".bss.align32"))); //16
char (*NetTxPackets)[TX_BUFF_SZ];


static int rx_new;                             /* RX descriptor ring pointer */
static int tx_new;                             /* TX descriptor ring pointer */

static char rxRingSize;
static char txRingSize;

static int   dc21x4x_init(struct eth_device* dev);
static void  read_hw_addr(struct eth_device *dev);


static int INL(struct eth_device* dev, u_long addr)
{
	return (*(volatile u_long *)(addr + dev->iobase));
}

static void OUTL(struct eth_device* dev, int command, u_long addr)
{
	*(volatile u_long *)(addr + dev->iobase) = (command);
}

int dc21x4x_initialize(struct eth_device* dev)
{

	/* Ensure we're not sleeping. */

	read_hw_addr(dev);

	dc21x4x_init(dev);

	return 0;
}
#define next_tx(x) (((x+1)==NUM_TX_DESC)?0:(x+1))
#define next_rx(x) (((x+1)==NUM_RX_DESC)?0:(x+1))

static void send_setup_frame(struct eth_device* dev);
/*
 * init function
 */
static int dc21x4x_init(struct eth_device* dev)
{
	int		i;
    pr_debug("\r\n%s, %d iobase:%02x\r\n",__func__,__LINE__,dev->iobase);
	/* Ensure we're not sleeping. */

#ifdef CONFIG_TULIP_FIX_DAVICOM
	RESET_DM9102(dev);
#else
	RESET_DE4X5(dev);
#endif

	if ((INL(dev, DE4X5_STS) & (STS_TS | STS_RS)) != 0) {
		pr_debug("Error: Cannot reset ethernet controller.\n");
		pr_debug("Error: read 0x%x.\n", INL(dev, DE4X5_STS));
		return 0;
	}

#ifdef CONFIG_TULIP_SELECT_MEDIA
	dc21x4x_select_media(dev);
#else
	OUTL(dev, OMR_SDP | OMR_PS , DE4X5_OMR);
#endif

	/*
	 * initialise rx descriptors
	 * use it as chain structure
	 */
	rxRingSize = NUM_RX_DESC;
	txRingSize = NUM_TX_DESC;
	
	rx_ring = (struct de4x5_desc *)(CACHED_TO_UNCACHED((unsigned long)_rx_ring));
	tx_ring = (struct de4x5_desc *)(CACHED_TO_UNCACHED((unsigned long)_tx_ring));

	NetRxPackets = (char (*)[RX_BUFF_SZ])(CACHED_TO_UNCACHED((unsigned long)__NetRxPackets));
	NetTxPackets = (char (*)[TX_BUFF_SZ])(CACHED_TO_UNCACHED((unsigned long)__NetTxPackets));
	
	for (i = 0; i < NUM_RX_DESC; i++) {
		rx_ring[i].status = (R_OWN); //Initially MAC owns it.
		rx_ring[i].des1 = RX_BUFF_SZ;
		rx_ring[i].buf = CACHED_TO_UNCACHED((u32) NetRxPackets[i]);//XXX
#ifdef CONFIG_TULIP_FIX_DAVICOM
		rx_ring[i].next = CACHED_TO_UNCACHED((u32) &rx_ring[next_rx(i)]);
#else
		rx_ring[i].next = 0;
#endif
	}

	/*
	 * initialize tx descriptors
	 * use it as chain structure
	 */
	for (i=0; i < NUM_TX_DESC; i++) {
		tx_ring[i].status = 0;
		tx_ring[i].des1 = 0xe1000000;//TD_TCH;
		tx_ring[i].buf = CACHED_TO_UNCACHED((u32) NetTxPackets[i]);

#ifdef CONFIG_TULIP_FIX_DAVICOM
		tx_ring[i].next = (CACHED_TO_UNCACHED((u32) &tx_ring[next_tx(i)]));
#else
		tx_ring[i].next = 0;
#endif
	}


	/* Write the end of list marker to the descriptor lists. */
	rx_ring[rxRingSize - 1].des1 |= RD_RER; //Receive end of ring

	/* Tell the adapter where the TX/RX rings are located. */
	pr_debug("rx ring %x\n", (u32)rx_ring);
	pr_debug("tx ring %x\n", (u32)tx_ring);
	OUTL(dev, (u32) rx_ring, DE4X5_RRBA);
	OUTL(dev, (u32) tx_ring, DE4X5_TRBA);

	udelay(100);
	START_DE4X5(dev);

	tx_new = 0;
	rx_new = 0;

	pr_debug("DE4X5_BMR= %x\n",  INL(dev, DE4X5_BMR));
	pr_debug("DE4X5_TPD= %x\n",  INL(dev, DE4X5_TPD));
	pr_debug("DE4X5_RRBA= %x\n", INL(dev, DE4X5_RRBA));
	pr_debug("DE4X5_TRBA= %x\n", INL(dev, DE4X5_TRBA));
	pr_debug("DE4X5_STS= %x\n",  INL(dev, DE4X5_STS));
	pr_debug("DE4X5_OMR= %x\n",  INL(dev, DE4X5_OMR));
	send_setup_frame(dev);
    pr_debug("After setup\n");
	pr_debug("DE4X5_BMR= %x\n",  INL(dev, DE4X5_BMR));
	pr_debug("DE4X5_TPD= %x\n",  INL(dev, DE4X5_TPD));
	pr_debug("DE4X5_RRBA= %x\n", INL(dev, DE4X5_RRBA));
	pr_debug("DE4X5_TRBA= %x\n", INL(dev, DE4X5_TRBA));
	pr_debug("DE4X5_STS= %x\n",  INL(dev, DE4X5_STS));
	pr_debug("DE4X5_OMR= %x\n",  INL(dev, DE4X5_OMR));

	return 1;
}

/*
 *  gethex(vp,p,n) 
 *      convert n hex digits from p to binary, result in vp, 
 *      rtn 1 on success
 */
static int gethex(int32_t *vp, char *p, int n)
{
    u_long v;
    int digit;

    for (v = 0; n > 0; n--) {
        if (*p == 0)
            return (0);
        if (*p >= '0' && *p <= '9')
            digit = *p - '0';
        else if (*p >= 'a' && *p <= 'f')
            digit = *p - 'a' + 10;
        else if (*p >= 'A' && *p <= 'F')
            digit = *p - 'A' + 10;
        else
            return (0);

        v <<= 4;
        v |= digit;
        p++;
    }
    *vp = v;
    return (1);
}                                                                 

static void read_hw_addr(struct eth_device *dev)
{
	static char maddr[ETH_ALEN]={0x00, 0x98, 0x76, 0x64, 0x32, 0x19};
	char *p = &dev->dev_addr[0];
	int i;
	{
		int i;
		int32_t v;
	char *s=NULL;
	if(s){
		for(i = 0; i < 6; i++) {
			gethex(&v, s, 2);
			maddr[i] = v;
			s += 3;         /* Don't get to fancy here :-) */
		} 
	 }
	} 

	for (i = 0; i < ETH_ALEN; i++) 
		*p++ = maddr[i];

	return;

}

static char	setup_frame[SETUP_FRAME_LEN];
static void send_setup_frame(struct eth_device* dev)
{
	int		i;
	char 	*pa = (char *)(&setup_frame[0]);

	memset(pa, 0x00, SETUP_FRAME_LEN);

	for (i = 0; i < ETH_ALEN; i++) {
		*(pa + (i & 1)) = dev->dev_addr[i];
		if (i & 0x01) {
			pa += 4;
		}
	}

	for(i = 0; tx_ring[tx_new].status & (T_OWN); i++) {
		if (i >= TOUT_LOOP) {
			pr_debug("tx error buffer not ready tx_ring[%d]=%02x\n", i, tx_ring[tx_new].status);
			goto Done;
		}
	}

	tx_ring[tx_new].buf = (CACHED_TO_UNCACHED((u32) &setup_frame[0]));
	tx_ring[tx_new].des1 = (TD_TCH | TD_SET| SETUP_FRAME_LEN);
	tx_ring[tx_new].status = (T_OWN);

    pr_debug("\r\nbuf:%x, des1:%x, status:%x",tx_ring[tx_new].buf,tx_ring[tx_new].des1,tx_ring[tx_new].status);                                                                     

	OUTL(dev, POLL_DEMAND, DE4X5_TPD);
	udelay(1000);

    pr_debug("\r\nnew:%d ,status:%x\r\n",tx_new,tx_ring[tx_new].status);
	for(i = 0; tx_ring[tx_new].status & (T_OWN); i++) {
		if (i >= TOUT_LOOP) {
			pr_debug("tx buffer not ready\n");
			goto Done;
		}
	}

	if ((tx_ring[tx_new].status) != 0x7FFFFFFF) {
		pr_debug("TX error status2 = 0x%08X\n", (tx_ring[tx_new].status));
	}

	tx_ring[tx_new].des1 = 0xe1000000;//TD_TCH;
	tx_ring[tx_new].buf = CACHED_TO_UNCACHED((u32) NetTxPackets[tx_new]);
	tx_ring[tx_new].next = (CACHED_TO_UNCACHED((u32) &tx_ring[next_tx(tx_new)]));
	tx_new = next_tx(tx_new);

#if 0
	{
		/* Send to mac */
		int my_frame = 0x12345678;
		int len, i;

		len = sizeof(my_frame);
		tx_ring[tx_new].buf = (CACHED_TO_UNCACHED((u32) &my_frame));
		tx_ring[tx_new].des1 = 0xe1000000 | len;
		tx_ring[tx_new].status = (T_OWN);

		OUTL(dev, POLL_DEMAND, DE4X5_TPD);
		udelay(1000);

	}
#endif

Done:
	return;
}

static int dmfe_start(struct udevice *dev)
{
	struct eth_device *priv = dev_get_priv(dev);
    dc21x4x_initialize(priv);
	return 0;
}

static int dmfe_send(struct udevice *dev, void *ptr, int len)
{
	struct eth_device *priv = dev_get_priv(dev);
	
    if (len > PKTSIZE_ALIGN)
		len = PKTSIZE_ALIGN;

    if((tx_ring[tx_new].status & (T_OWN)))return -1;

    memset((void *)NetTxPackets[tx_new], 0 , len);
    memcpy((void *)NetTxPackets[tx_new], ptr, len);
    

    tx_ring[tx_new].des1   = (0xe1000000 | len); //frame in a single TD
    tx_ring[tx_new].buf = CACHED_TO_UNCACHED((u32) NetTxPackets[tx_new]);
    tx_ring[tx_new].next = (CACHED_TO_UNCACHED((u32) &tx_ring[next_tx(tx_new)]));
    tx_ring[tx_new].status = (T_OWN);


	/*
	 * command the mac to start transmit process
	 */
    OUTL(priv, POLL_DEMAND, DE4X5_TPD);
    tx_new = next_tx(tx_new);
    
    udelay(1000);
	return 0;
}

static int dmfe_recv(struct udevice *dev, int flags, uchar **packetp)
{
	s32	 status;
	int	 length;
	int  received=0;

	for ( ; !received; ) {
		length    = 0;
			
		status = (s32)(rx_ring[rx_new].status);
		
		if (status & R_OWN) {
			break;
		}

		rx_ring[rx_new].next = CACHED_TO_UNCACHED((u32) &rx_ring[next_rx(rx_new)]);

		if (status & RD_LS) {
			/* Valid frame status.
			 */
			if (status & RD_ES) {

				/* There was an error.
				 */
			    //pr_debug("RX error status = 0x%08X\n", status);
			} else {
				/* A valid frame received.
				 */
				length = (rx_ring[rx_new].status >> 16) & 0x3fff;

				/* Pass the packet up to the protocol
				 * layers.
				 */
				//pr_debug("received a packet status %x\n", status);

				{
                    *packetp = (uchar *)NetRxPackets[rx_new];
				}
				received =1;

			}

			/* Change buffer ownership for this frame, back
			 * to the adapter.
			 */
			rx_ring[rx_new].status = (R_OWN);
		}

		/* Update entry information.
		 */
		rx_new = next_rx(rx_new);
	}

	return length;
}

static void dmfe_stop(struct udevice *dev)
{
	struct eth_device *priv = dev_get_priv(dev);
	STOP_DE4X5(priv);
	OUTL(priv, 0, DE4X5_SICR);

	return ;
}

static int dmfe_probe(struct udevice *dev)
{

	return 0;
}

static int dmfe_remove(struct udevice *dev)
{
	return 0;
}

static const struct eth_ops dmfe_ops = {
	.start = dmfe_start,
	.send = dmfe_send,
	.recv = dmfe_recv,
	.stop = dmfe_stop,
};

static int dmfe_ofdata_to_platdata(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_platdata(dev);
	struct eth_device *priv = dev_get_priv(dev);

	pdata->iobase = (phys_addr_t)devfdt_get_addr(dev);
    priv->iobase = pdata->iobase;

	return 0;
}

static const struct udevice_id dmfe_ids[] = {
	{ .compatible = "ls,ls-dmfe" },
	{ }
};

U_BOOT_DRIVER(dmfe) = {
	.name   = "dmfe",
	.id     = UCLASS_ETH,
	.of_match = dmfe_ids,
	.ofdata_to_platdata = dmfe_ofdata_to_platdata,
	.probe  = dmfe_probe,
	.remove = dmfe_remove,
	.ops    = &dmfe_ops,
	.priv_auto_alloc_size = sizeof(struct eth_device),
	.platdata_auto_alloc_size = sizeof(struct eth_pdata),
};
