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

    // lkl_fuzz_set_buf(data_ptr, data_size)lkl

}
