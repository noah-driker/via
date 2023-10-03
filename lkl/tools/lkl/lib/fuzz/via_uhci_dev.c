#include <lkl_host.h>
#include <string.h>
#include <stdio.h>
#include "../iomem.h"
#include "via_uhci_dev.h"


#include <stdbool.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
#include <linux/ethtool.h>
#include <net/if.h>
#include <pthread.h>
#include <libconfig.h>
#include <lkl.h>

#include <signal.h>
#include <time.h>
#include <sys/queue.h>


struct UHCIAsync {
    USBPacket packet;
    uint8_t static_buf[64];
    uint8_t *buf;
    UHCIQueue *queue;
    TAILQ_ENTRY(UHCIAsync) next;
    uint32_t td_addr;
    uint8_t done;
};

struct UHCIQueue {
    uint32_t qh_addr;
    uint32_t token;
    UHCIState *uhci;
    USBEndpoint *ep;
    TAILQ_ENTRY(UHCIQueue) next;
    TAILQ_HEAD(, UHCIAsync) asyncs;
    int8_t valid;
};

/*
    port of QEMU uhci_port_read
        driver expects val to be returned rather than just dereferencing, setting res
*/
static int via_uhci_dev_read(void *data, int offset, void *res, int size) {

    uint64_t val;
    UHCIState* state = (UHCIState*) data;

    switch (offset) {
    case 0x00:
        val = state->cmd;
        break;
    case 0x02:
        val = state->status;
        break;
    case 0x04:
        val = state->intr;
        break;
    case 0x06:
        val = state->frnum;
        break;
    case 0x08:
        val = state->fl_base_addr & 0xffff;
        break;
    case 0x0a:
        val = (state->fl_base_addr >> 16) & 0xffff;
        break;
    case 0x0c:
        val = state->sof_timing;
        break;
    case 0x10 ... 0x1f:
        {
            UHCIPort *port;
            int n;
            n = (offset >> 1) & 7;
            if (n >= NB_PORTS)
                goto read_default;
            port = &state->ports[n];
            val = port->ctrl;
        }
        break;
    default:
    read_default:
        val = 0xff7f; // disabled port
        break;
    } 

    if (size == 1) {
        *(char *) res = val;
    } else if (size == 2) {
        *(short *) res = htole16(val);
    } else if (size == 4) {
        *(int *) res = htole32(val);
    } else if (size == 8) {
        *(long *) res = htole64(val);
    }

    return val;
}


/*
    port from QEMU uhci_port_write
*/
static int via_uhci_dev_write(void *data, int offset, void *res, int size) {
    uint64_t val;
    UHCIState* state = (UHCIState*)data;
    
    if (size == 1) {
        val = le64toh(*(char *) res);
    } else if (size == 2) {
        val = le64toh(*(short *) res);
    } else if (size == 4) {
        val = le64toh(*(int *) res);
    } else if (size == 8) {
        val = le64toh(*(long *) res);
    }


    switch(offset) {
    case 0x00:
        if ( (val & UHCI_CMD_RS) && !(state->cmd & UHCI_CMD_RS)) {
            // start frame processing
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            state->expire_time = ts.tv_sec * NANOSECONDS_PER_SECOND + ts.tv_nsec + NANOSECONDS_PER_SECOND/FRAME_TIMER_FREQ;

            /*
                it_value specifies initial expiry time for interval timer
                it_interval specifies the interval for subsequent expirys
            */
            struct itimerspec its;

            its.it_value.tv_sec = (NANOSECONDS_PER_SECOND / FRAME_TIMER_FREQ) / NANOSECONDS_PER_SECOND;
            its.it_value.tv_nsec = (NANOSECONDS_PER_SECOND / FRAME_TIMER_FREQ) % NANOSECONDS_PER_SECOND;
            
            its.it_interval.tv_sec = (NANOSECONDS_PER_SECOND / FRAME_TIMER_FREQ) / NANOSECONDS_PER_SECOND;;
            its.it_interval.tv_nsec = (NANOSECONDS_PER_SECOND / FRAME_TIMER_FREQ) % NANOSECONDS_PER_SECOND;
            
            timer_settime(state->timer_id, 0, &its, NULL);

            state->status &= ~UHCI_STS_HCHALTED;

        } else if (!(val & UHCI_CMD_RS)) {
            state->status |= UHCI_STS_HCHALTED;
        }
        if (val & UHCI_CMD_GRESET) {
            UHCIPort *port;
            int i;

            // send reset on the USB bus
            for (i = 0; i < NB_PORTS; i++) {
                port = &state->ports[i];

                // used in QEMU, not yet implemented
                //usb_device_reset(port->port.dev);
            }
            uhci_reset(state);

            // not sure what to return here, -1 at least seems not to break anything
            return -1;
        }
        if (val & UHCI_CMD_HCRESET) {
            uhci_reset(state);
            
            // not sure what to return here, -1 at least seems not to break anything            
            return -1;
        }
        state->cmd = val;
        if (val & UHCI_CMD_EGSM) {
            if (
                (state->ports[0].ctrl & UHCI_PORT_RD) ||
                (state->ports[1].ctrl & UHCI_PORT_RD) ) {

                uhci_resume(state);
            }
        }
        break;
    case 0x02:
        state->status &= ~val;

        // the chip spec is not coherent
        if (val & UHCI_STS_USBINT) {
            state->status2 = 0;
        }
        uhci_update_irq(state);
        break;
    case 0x04:
        state->intr = val;
        uhci_update_irq(state);
        break;
    case 0x06:
        if (state->status & UHCI_STS_HCHALTED) {
            state->frnum = val & 0x7ff;
        }
        break;
    case 0x08:
        state->fl_base_addr &= 0xffff0000;
        state->fl_base_addr |= val & ~0xfff;
        break;
    case 0x0a:
        state->fl_base_addr &= 0x0000ffff;
        state->fl_base_addr |= (val << 16);
        break;
    case 0x0c:
        state->sof_timing = val & 0xff;
        break;
    case 0x10 ... 0x1f:
        {
        UHCIPort *port;
        USBDevice *dev;

        int n = (offset >> 1) & 7;

        if (n >= NB_PORTS) {
            // the configuration is incorrect
            // assuming EINVAL is the right return code
            return -LKL_EINVAL;
        }

        port = &state->ports[n];
        dev = port->port.dev;

        if (dev && dev->attached) {
            // port reset
            if ( (val & UHCI_PORT_RESET) && !(port->ctrl & UHCI_PORT_RESET)) {
                // QEMU uses this, not yet implemented
                //usb_device_reset(dev);
            }
        }
        port->ctrl &= UHCI_PORT_READ_ONLY;
        if (!(port->ctrl & UHCI_PORT_CCS)) {
            val &= ~UHCI_PORT_EN;
        }
        port->ctrl |= (val & ~UHCI_PORT_READ_ONLY);
        port->ctrl &= ~(val & UHCI_PORT_WRITE_CLEAR);
        }
        break;
    }

    return 0;
}

static void uhci_update_irq(UHCIState *s) {
    lkl_trigger_irq(3);
}

static void uhci_resume(void *state) {
    UHCIState *s = (UHCIState *) state;
    if (!s) {
        return;
    }

    // in QEMU's uhci_resume this is a conditional IRQ trigger
    // s->cmd is not set correctly elsewhere

    //if (s->cmd & UHCI_CMD_EGSM) {
    s->cmd |= UHCI_CMD_FGR;
    s->status |= UHCI_STS_RD;
    uhci_update_irq(s);
    //}
}

static void uhci_async_free(UHCIAsync *async) {
    // QEMU uses this, not yet implemented
    //usb_packet_cleanup(&async->packet);

    if (async->buf != async->static_buf) {
        //g_free(async->buf);
    }
    //g_free(async);
}

static void uhci_async_unlink(UHCIAsync *async) {
    UHCIQueue *queue = async->queue;
    TAILQ_REMOVE(&queue->asyncs, async, next);
}

static void uhci_async_cancel(UHCIAsync *async) {
    uhci_async_unlink(async);
    if (!async->done) {
        // QEMU uses this, not yet implemented
        //usb_cancel_packet(&async->packet);
    }
    uhci_async_free(async);
}

static void uhci_queue_free(UHCIQueue *queue, const char *reason) {
    UHCIState *s = queue->uhci;
    UHCIAsync *async;

    while (!TAILQ_EMPTY(&queue->asyncs)) {
        async = TAILQ_FIRST(&queue->asyncs);
        uhci_async_cancel(async);
    }
    // QEMU uses this, not yet implemented
    //usb_device_ep_stopped(queue->ep->dev, queue->ep);

    TAILQ_REMOVE(&s->queues, queue, next);
    //g_free(queue);
}
static void uhci_async_cancel_device(UHCIState *s, USBDevice *dev) {
    UHCIQueue *queue, *n;

    TAILQ_FOREACH_SAFE(queue, &s->queues, next, n) {
        if (queue->ep->dev == dev) {
            uhci_queue_free(queue, "cancel-device");
        }
    }
}

void uhci_attach(USBPort *port1) {

    UHCIState *s = port1->opaque;
    UHCIPort *port = &s->ports[port1->index];

    // set connect status
    port->ctrl |= UHCI_PORT_CCS | UHCI_PORT_CSC;

    // // update speed
    if (port->port.dev->speed == USB_SPEED_LOW) {
        port->ctrl |= UHCI_PORT_LSDA;
    } else {
        port->ctrl &= ~UHCI_PORT_LSDA;
    }

    uhci_resume(s);
}


void uhci_detach(USBPort *port1) {
    UHCIState *s = port1->opaque;
    UHCIPort *port = &s->ports[port1->index];

    uhci_async_cancel_device(s, port1->dev);

    // set connect status
    if (port->ctrl & UHCI_PORT_CCS) {
        port->ctrl &= ~UHCI_PORT_CCS;
        port->ctrl |= UHCI_PORT_CSC;
    }

    // disable port
    if (port->ctrl & UHCI_PORT_EN) {
        port->ctrl &= ~UHCI_PORT_EN;
        port->ctrl |= UHCI_PORT_ENC;
    }
    uhci_resume(s);
}

static const struct lkl_iomem_ops via_dev_ops = {
  .read = via_uhci_dev_read,
  .write = via_uhci_dev_write,
};

static USBPortOps uhci_port_ops = {
    .attach = uhci_attach,
    .detach = uhci_detach
};

static void uhci_async_validate_begin(UHCIState *s){
    UHCIQueue *queue;
    TAILQ_FOREACH(queue, &s->queues, next) {
        queue->valid--;
    }
}

static void uhci_async_validate_end(UHCIState *s) {
    UHCIQueue *queue, *n;

    TAILQ_FOREACH_SAFE(queue, &s->queues, next, n) {
        if (!queue->valid) {
            uhci_queue_free(queue, "validate-end");
        }
    }
}

static void uhci_async_cancel_all(UHCIState *s) {
    UHCIQueue *queue, *nq;
    TAILQ_FOREACH_SAFE(queue, &s->queues, next, nq) {
        uhci_queue_free(queue, "cancel-all");
    }
}

static void uhci_process_frame(UHCIState *s) {
    // NOP for now, need to implement
    lkl_printf("(NoahD) via_uhci_dev : uhci_process_frame");
}

void uhci_reset(UHCIState* s)  {
    // partially implemented, just enough to support timing and irqs
    s->cmd = 0;
    s->status = UHCI_STS_HCHALTED;
    s->status2 = 0;
    s->intr = 0;
    s->fl_base_addr = 0;
    s->sof_timing = 64;
    uhci_async_cancel_all(s);
    uhci_update_irq(s);
}

/*
    port of QEMU uhci_frame_timer

    QEMU timer callback supports passing data with pointer, 
        the time.h interval timer does not support this
        the device state is a global variable instead
*/
void timer_callback(int signum) {
    uint64_t t_now, t_last_run;
    int i, frames;

    const uint64_t frame_t = NANOSECONDS_PER_SECOND/FRAME_TIMER_FREQ;

    state->completions_only = 0;

    // QEMU uses this, not yet implemented
    //qemu_bh_cancel(s->bh);

    if (!(state->cmd & UHCI_CMD_RS)) {
        // full stop
        struct itimerspec new_value;
        new_value.it_value.tv_sec = 0;
        new_value.it_value.tv_nsec = 0;
        timer_settime(state->timer_id, 0, &new_value, NULL);
        timer_delete(state->timer_id);

        uhci_async_cancel_all(state);
        state->status |= UHCI_STS_HCHALTED;
        return;
    }

    // state->expire_time is absolute time (not relative like it_value from interval timer)
    t_last_run = state->expire_time - frame_t;

    struct timespec ts;
    // get absolute realtime
    clock_gettime(CLOCK_REALTIME, &ts);
    t_now = (ts.tv_sec*NANOSECONDS_PER_SECOND) + ts.tv_nsec;

    frames = (t_now - t_last_run) / frame_t;

    if (frames > state->maxframes) {
        int skipped = frames - state->maxframes;
        state->expire_time += skipped * frame_t;
        state->frnum = (state->frnum + skipped) & 0x7ff;
        frames -= skipped;
    }
    if (frames > MAX_FRAMES_PER_TICK) {
        frames = MAX_FRAMES_PER_TICK;
    }
    for (i = 0; i < frames; i++) {
        state->frame_bytes = 0;
        uhci_async_validate_begin(state);
        uhci_process_frame(state);
        uhci_async_validate_end(state);
        state->frnum = (state->frnum + 1) & 0x7ff;
        state->expire_time += frame_t;
    }

    if (state->pending_int_mask) {
        state->status2 |= state->pending_int_mask;
        state->status |= UHCI_STS_USBINT;
        uhci_update_irq(state);
    }
    state->pending_int_mask = 0;

    /*  
        QEMU updates the timer expiry time here
        since the timer used here is an interval timer, this is not necessary
    */
}

void setup_via_uhci_device() {

    long handle;    
	int mmio_size = 0x1fffff, i = 0;


    state = lkl_host_ops.mem_alloc(sizeof(*state));
    if (!state) {
        fprintf(stderr, "uhci_dev: failed to allocate memory");
        return;
    }

    memset(state, 0, sizeof(*state));

    // setup frame timer
    timer_t timer_id;

    struct itimerspec its;
    its.it_value.tv_sec = (NANOSECONDS_PER_SECOND / FRAME_TIMER_FREQ) / NANOSECONDS_PER_SECOND;
    its.it_value.tv_nsec = (NANOSECONDS_PER_SECOND / FRAME_TIMER_FREQ) % NANOSECONDS_PER_SECOND;
    its.it_interval.tv_nsec = NANOSECONDS_PER_SECOND/FRAME_TIMER_FREQ;
    
    struct sigevent sev;
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGUSR1;
    sev.sigev_value.sival_ptr = &timer_id;

    timer_create(CLOCK_REALTIME, &sev, &timer_id);
    timer_settime(timer_id, 0, &its, NULL);

    // install timer_callback as signal handler for SIGUSR1
    signal(SIGUSR1, timer_callback);

    state->frame_timer = &its;
    state->timer_id = timer_id;

    
    // config ports
    USBDevice* usb_dev = (USBDevice*) lkl_host_ops.mem_alloc(sizeof(USBDevice));
    usb_dev->speed = USB_SPEED_LOW;

    for (int i = 0; i < NB_PORTS; i++){
        state->ports[i].ctrl = 0x0080;
        state->ports[i].port.ops = &uhci_port_ops;
        state->ports[i].port.index = i;
        state->ports[i].port.opaque = state;
        state->ports[i].port.dev = usb_dev;
    }

    // initial values
    state->cmd = 0;
    state->status = UHCI_STS_HCHALTED;
    state->status2 = 0;
    state->intr = 0;
    state->fl_base_addr = 0;
    state->sof_timing = 64;
    state->pending_int_mask = 1;
    state->completions_only = true;
    state->expire_time = (NANOSECONDS_PER_SECOND / FRAME_TIMER_FREQ);
    state->frnum = 0;
    state->frame_bytes = 0;
    TAILQ_INIT(&state->queues);

    void* base_addr = register_iomem(state, mmio_size, &via_dev_ops);	
    int64_t mmio_start = (uint64_t)base_addr;

    struct lkl_fuzz_pci_dev_config pci_conf;
    memset(&pci_conf, 0, sizeof(pci_conf));
    
    pci_conf.conf.vendor_id = 0x1234;
    pci_conf.conf.device_id = 0x1234;
    pci_conf.conf.class_prog   = 0x00;     // 00
    pci_conf.conf.class_device = 0x0c03;   // 0c03
    pci_conf.n_mmio = 1;
    for(i=0; i < pci_conf.n_mmio; i++) {
        pci_conf.mmio_regions[i].remapped = 1;
        pci_conf.mmio_regions[i].flags = 0x40200;
        pci_conf.mmio_regions[i].start = mmio_start;
        pci_conf.mmio_regions[i].end = mmio_start + mmio_size;
        mmio_start = pci_conf.mmio_regions[i].end;
    }

    handle = lkl_sys_fuzz_configure_dev(LKL_FDEV_TYPE_PCI, &pci_conf);    

    // setup port for attach/detach in harness
    // usb_port is global so it can be used in the harness
    usb_port = (USBPort*) lkl_host_ops.mem_alloc(sizeof(USBPort));
    usb_port->dev = usb_dev;
    usb_port->opaque = state;
    usb_port->index = 0;    
    usb_port->ops = &uhci_port_ops;
}

