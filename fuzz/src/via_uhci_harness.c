#include <fcntl.h>
#include <dlfcn.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <linux/ethtool.h>
#include <net/if.h>
#include <pthread.h>
#include <lkl.h>
#include <lkl_host.h>
#include "../../lkl/tools/lkl/lib/fuzz/usb.h"
#include "../../lkl/tools/lkl/lib/fuzz/via_uhci_dev.h"

void *this_module = NULL;
char module_path[512];
int loglevel = 8;



int main(void) {
    long handle;
    char k_cmdline[256];

    //handle = lkl_sys_fuzz_configure_dev(LKL_FDEV_TYPE_PCI, &pci_conf);

    // load and initialize pci module
    snprintf(module_path, sizeof(module_path), "/home/admax/via/lkl/drivers/usb/host/uhci-hcd.ko");

    // start lkl
    fprintf(stderr, "Initializing LKL...\n");
    snprintf(k_cmdline, sizeof(k_cmdline), "mem=4096M noirqdebug loglevel=%d maxcpus=1 debug loglevel=8 earlyprintk=serial dyndbg=\"module uhci-pci +p\"", loglevel);

    lkl_fuzz_init_fuzzer();
    lkl_start_kernel(&lkl_host_ops, k_cmdline);
    
    fprintf(stdout, "(NoahD) via_uhci_harness : before setup_via_uhci_device\n");
    // invoke setup function for pci uhci device
    setup_via_uhci_device();
    
    void *module_handle = dlopen(module_path, RTLD_GLOBAL | RTLD_NOW);
    if (!module_handle) {
        fprintf(stderr, "Error loading module dependency %s: %s\n", module_path, dlerror());
	return -1;
    }
    fprintf(stdout, "(NoahD) via_uhci_harness : after dlopen\n");


    this_module = dlsym(module_handle, "__this_module");
    if (!this_module) {
        fprintf(stderr, "Error resolving __this_module for %s: %s\n", module_path, dlerror());
	return -1;
    }
    fprintf(stdout, "(NoahD) via_uhci_harness : after dlsym\n");

    lkl_sys_init_loaded_module(this_module);

    fprintf(stdout, "(NoahD) via_uhci_harness : after init loaded\n");

    UHCIState* state = (UHCIState*) lkl_host_ops.mem_alloc(sizeof(UHCIState));

    if (!state) {
        fprintf(stderr, "uhci_dev: failed to allocate memory");
        return -1;
    }

    memset(state, 0, sizeof(*state));

    USBDevice* usb_dev = (USBDevice*) lkl_host_ops.mem_alloc(sizeof(USBDevice));
    USBPortOps port_ops = {
        .attach = uhci_attach,
        .detach = uhci_detach
    };
    for (int i = 0; i < NB_PORTS; i++){
        state->ports[i].ctrl = 0x0080;
        state->ports[i].port.ops = &port_ops;
        state->ports[i].port.index = i;
        state->ports[i].port.opaque = state;
        state->ports[i].port.dev = usb_dev;
    }

    // config irq
    state->irq = lkl_get_free_irq("virtio");

    // initial values
    state->cmd = 0;
    state->status = UHCI_STS_HCHALTED;
    state->status2 = 0;
    state->intr = 0;
    state->fl_base_addr = 0;
    state->sof_timing = 64;

    usb_dev->speed = USB_SPEED_LOW;

    USBPort* usb_port = (USBPort*) lkl_host_ops.mem_alloc(sizeof(USBPort));
    usb_port->dev = usb_dev;
    usb_port->opaque = state;
    usb_port->index = 0;    

    usb_port->ops = &port_ops;

    fprintf(stdout, "(NoahD) via_uhci_dev : calling uhci_attach\n");
    uhci_attach(usb_port);
    fprintf(stdout, "(NoahD) via_uhci_dev : calling uhci_detach\n");
    uhci_detach(usb_port);    


    fprintf(stdout, "(NoahD) via_uhci_harness : HARNESS END\n");
    return 0;
}
