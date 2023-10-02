#ifndef __VIA_UHCI_DEV_H__
#define __VIA_UHCI_DEV_H__

#include "usb.h"
#include <stdbool.h>

#define NB_PORTS 2

// UCHI regs

#define UHCI_CMD_FGR      (1 << 4)
#define UHCI_CMD_EGSM     (1 << 3)
#define UHCI_CMD_GRESET   (1 << 2)
#define UHCI_CMD_HCRESET  (1 << 1)
#define UHCI_CMD_RS       (1 << 0)

#define UHCI_STS_HCHALTED (1 << 5)
#define UHCI_STS_HCPERR   (1 << 4)
#define UHCI_STS_HSERR    (1 << 3)
#define UHCI_STS_RD       (1 << 2)
#define UHCI_STS_USBERR   (1 << 1)
#define UHCI_STS_USBINT   (1 << 0)

#define TD_CTRL_SPD     (1 << 29)
#define TD_CTRL_ERROR_SHIFT  27
#define TD_CTRL_IOS     (1 << 25)
#define TD_CTRL_IOC     (1 << 24)
#define TD_CTRL_ACTIVE  (1 << 23)
#define TD_CTRL_STALL   (1 << 22)
#define TD_CTRL_BABBLE  (1 << 20)
#define TD_CTRL_NAK     (1 << 19)
#define TD_CTRL_TIMEOUT (1 << 18)

#define UHCI_PORT_SUSPEND (1 << 12)
#define UHCI_PORT_RESET (1 << 9)
#define UHCI_PORT_LSDA  (1 << 8)
#define UHCI_PORT_RSVD1 (1 << 7)
#define UHCI_PORT_RD    (1 << 6)
#define UHCI_PORT_ENC   (1 << 3)
#define UHCI_PORT_EN    (1 << 2)
#define UHCI_PORT_CSC   (1 << 1)
#define UHCI_PORT_CCS   (1 << 0)

#define UHCI_PORT_READ_ONLY    (0x1bb)
#define UHCI_PORT_WRITE_CLEAR  (UHCI_PORT_CSC | UHCI_PORT_ENC)

#define NANOSECONDS_PER_SECOND 1000000000LL
#define FRAME_TIMER_FREQ 1000

#define QH_VALID		32
#define MAX_FRAMES_PER_TICK (QH_VALID / 2)

// version of QEMU QTAILQ_FOREACH_SAFE
#define TAILQ_FOREACH_SAFE(var, head, field, next_var)			\
	for ((var) = ((head)->tqh_first);							\
		(var) && ((next_var) = ((var)->field.tqe_next), 1);		\
		(var) = (next_var))



typedef struct UHCIQueue UHCIQueue;
typedef struct UHCIAsync UHCIAsync;

typedef struct UHCIPort {
	USBPort port;
	uint16_t ctrl;
} UHCIPort;


typedef struct UHCIState {
	// PCIDevice dev
	uint8_t* config;
	// MemoryRegion io_bar
	uint16_t cmd; 			// command register
	uint16_t status;
	uint8_t status2;
	uint16_t intr; 			// interrupt enable register
	uint16_t frnum; 		// frame number
	uint16_t fl_base_addr; 	// frame list base address
	uint8_t sof_timing;

	int irq;


	struct itimerspec* frame_timer;
	//QEMUBH *bh;
	uint32_t frame_bytes;
	bool completions_only;

	timer_t timer_id;
	long expire_time;

	TAILQ_HEAD(, UHCIQueue) queues;

	UHCIPort ports[NB_PORTS];
	uint32_t pending_int_mask;
	uint32_t maxframes;

} UHCIState;

void uhci_attach(USBPort *port1);
void uhci_detach(USBPort *port1);
void timer_callback(int signum);
static void uhci_resume(void *state);
static void uhci_update_irq(UHCIState *s);

UHCIState* state;
USBPort* usb_port;


#endif