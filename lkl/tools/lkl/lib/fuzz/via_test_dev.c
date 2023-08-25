#include <lkl_host.h>
#include <string.h>
#include <stdio.h>
#include "../iomem.h"

#define IORESOURCE_MEM		0x00000200



static int via_dev_read(void *data, int offset, void *res, int size) {
	lkl_printf("Calling via dev read %d\n", 2);
	if (size == 1) {
		*(char *) res = 'a';		
	} else if (size == 4) {
		*(int *) res = 12345;
	}

    return 0;
}

static int via_dev_write(void *data, int offset, void *res, int size)
{

    return 0;
}


static const struct lkl_iomem_ops via_dev_ops = {
  .read = via_dev_read,
  .write = via_dev_write,
};

void setup_via_test_device() {

	int mmio_size;
	int ret;

    struct lkl_fuzz_pci_dev_config pci_conf;

	//struct lkl_fuzz_platform_dev_config platform_conf;
	
    mmio_size = 1024; // sizeof(lkl_virtio_fuzz_dev->config) = 1024
    void* base_addr = register_iomem(NULL, mmio_size, &via_dev_ops);	

    //memset(&pci_conf, 0, sizeof(pci_conf));
    int64_t mmio_start = 0x10000; // arbitrary value
    memset(&pci_conf, 0, sizeof(pci_conf));
    pci_conf.conf.vendor_id = 0x1912;
    pci_conf.conf.device_id = 0x0014;
        //pci_conf.conf.sub_vendor_id = SVID;
        //pci_conf.conf.sub_id = SDID;
        //pci_conf.conf.revision_id = revision;
        //pci_conf.conf.class_device = pci_class;
        //pci_conf.fuzz_dma = fuzz_dma;
    pci_conf.n_mmio = 1;
	pci_conf.mmio_regions[0].remapped = 1;
	pci_conf.mmio_regions[0].start = (uint64_t)base_addr;
	pci_conf.mmio_regions[0].end = (uint64_t)base_addr + mmio_size;
	pci_conf.mmio_regions[0].flags = IORESOURCE_MEM;    
    ret = lkl_sys_fuzz_configure_dev(LKL_FDEV_TYPE_PCI, &pci_conf);    

    /* platform dev stuff
    	memset(&platform_conf, 0, sizeof(platform_conf));
    	strncpy(platform_conf.name, "via-test-drv", 256);

    	platform_conf.n_mmio = 1;
    	platform_conf.mmio_regions[0].remapped = 1;
    	platform_conf.mmio_regions[0].start = (uint64_t)base_addr;
    	platform_conf.mmio_regions[0].end = (uint64_t)base_addr + mmio_size;
    	platform_conf.mmio_regions[0].flags = IORESOURCE_MEM;	

    	
    	ret = lkl_sys_fuzz_configure_dev(LKL_FDEV_TYPE_PLATFORM, &platform_conf);
    */
}
