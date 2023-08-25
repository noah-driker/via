#ifndef __VIA_UHCI_HARNESS_H__
#define __VIA_UHCI_HARNESS_H__

#include <lkl.h>
#include <lkl_host.h>

extern void *this_module;
extern void *module_handle;
extern char module_name[512];

int do_config();
uint64_t do_lkl_init(void);

#endif
