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

#define VID 0xc0300
#define DID 0x00


#define IORESOURCE_MEM		0x00000200
#define IORESOURCE_IO       0x00000100

//int pci_class = 0;

static int via_uhci_dev_read(void *data, int offset, void *res, int size) {
    // port from qemu uhci_port_read

    uint64_t val;
    struct UHCIState* state = (struct UHCIState*) data;

	lkl_printf("(NoahD) via_uhci_dev : in via_uhci_dev_read %d\n", size);



    // if (size != sizeof(uint32_t)) {
    //     lkl_printf("(NoahD) via_uhci_dev : incorrect size %d end of via_uhci_dev_read\n", size);
    //     return -LKL_EINVAL;
    // }

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


    //*(uint32_t *)res = htole32(val);

    lkl_printf("(NoahD) via_uhci_dev : end of via_uhci_dev_read\n");

    return 0;
}

static int via_uhci_dev_write(void *data, int offset, void *res, int size) {
    // port from qemu uhci_port_write
    uint64_t val;
    struct UHCIState* state = (struct UHCIState*)data;
    int ret = 0;
    
    lkl_printf("(NoahD) via_uhci_dev : in via_uhci_dev_write %d\n", size);

    if (size == 1) {
        val = le64toh(*(char *) res);
    } else if (size == 2) {
        val = le64toh(*(short *) res);
    } else if (size == 4) {
        val = le64toh(*(int *) res);
    } else if (size == 8) {
        val = le64toh(*(long *) res);
    }


    //val = le32toh(*(uint16_t *) res);


    switch(offset) {
    case 0x00:
        if ( (val & UHCI_CMD_RS) && !(state->cmd & UHCI_CMD_RS)) {
            // start frame processing
            struct itimerspec remaining_time;
            timer_gettime(state->timer_id, &remaining_time);
            state->expire_time = remaining_time.it_value.tv_nsec + (NANOSECONDS_PER_SECOND / FRAME_TIMER_FREQ);
            timer_settime(state->timer_id, 0, state->frame_timer, NULL);
            state->status &= ~UHCI_STS_HCHALTED;
        } else if (!(val & UHCI_CMD_RS)) {
            state->status |= UHCI_STS_HCHALTED;
        }
        if (val & UHCI_CMD_GRESET) {
            // UHCIPort *port;
            int i;

            // send reset on the USB bus
            for (i = 0; i < NB_PORTS; i++) {
                // port = &state->ports[i];
                // usb_device_reset(port->port.dev);
            }
            //uhci_reset(DEVICE(state));
            lkl_printf("(NoahD) via_uhci_dev : UHCI_CMD_GRESET end of via_uhci_dev_write\n");

            return -1;
        }
        if (val & UHCI_CMD_HCRESET) {
            //uhci_reset(DEVICE(state));
            lkl_printf("(NoahD) via_uhci_dev : UHCI_CMD_HCRESET end of via_uhci_dev_write\n");

            return -1;
        }
        state->cmd = val;
        if (val & UHCI_CMD_EGSM) {
            if (
                (state->ports[0].ctrl & UHCI_PORT_RD) ||
                (state->ports[1].ctrl & UHCI_PORT_RD) ) {

                // uhci_resume(state);
            }
        }
        break;
    case 0x02:
        state->status &= ~val;
        // the chip spec is not coherent
        if (val & UHCI_STS_USBINT) {
            state->status2 = 0;
        }
        //uhci_update_irq(state);
        break;
    case 0x04:
        state->intr = val;
        //uhci_update_irq(state);
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
        // UHCIPort *port;
        // USBDevice *dev;
        int n = (offset >> 1) & 7;
        if (n >= NB_PORTS) {
            lkl_printf("(NoahD) via_uhci_dev : incorrect num ports %d via_uhci_dev_write\n", n);
            return -LKL_EINVAL; // likely -LKL_EINVAL
        }
        //port = &state->ports[n];
        //dev = port->port.dev;

        // if (dev && dev->attached) {
            // port reset
            // if ( (val & UHCI_PORT_RESET) && !(port->ctrl & UHCI_PORT_RESET))
                //usb_device_reset(dev);

        // port->ctrl &= UHCI_PORT_READ_ONLY
        // if (!(port->ctrl & UHCI_PORT_CCS)) {
            // val &= ~UHCI_PORT_EN;
        // port->ctrl |= (val & ~UHCI_PORT_READ_ONLY);
        // port->ctrl &= ~(val & UHCI_PORT_WRITE_CLEAR);
        }
        break;
    }

    lkl_printf("(NoahD) via_uhci_dev : end of via_uhci_dev_write\n");


    return 0;
}


static const struct lkl_iomem_ops via_dev_ops = {
  .read = via_uhci_dev_read,
  .write = via_uhci_dev_write,
};



void setup_via_uhci_device() {

    long handle;    
	int mmio_size = 0x1fffff, i = 0;

    struct lkl_fuzz_pci_dev_config pci_conf;
    struct UHCIState* state; 

    lkl_printf("(NoahD) via_uhci_dev : in setup_via_uhci_device\n");

    state = lkl_host_ops.mem_alloc(sizeof(*state));
    if (!state) {
        fprintf(stderr, "uhci_dev: failed to allocate memory");
        return;
    }

    memset(state, 0, sizeof(*state));

    struct sigevent sev;
    timer_t timer_id;
    struct itimerspec timer_spec;

    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGALRM;
    sev.sigev_value.sival_ptr = &timer_id;

    timer_create(CLOCK_REALTIME, &sev, &timer_id);
    timer_spec.it_value.tv_nsec = NANOSECONDS_PER_SECOND/FRAME_TIMER_FREQ;

    state->frame_timer = &timer_spec;
    state->timer_id = timer_id;

    void* base_addr = register_iomem(state, mmio_size, &via_dev_ops);	
    int64_t mmio_start = (uint64_t)base_addr;

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
    lkl_printf("(NoahD) via_uhci_dev : setup_via_uhci_device END\n");    
}
