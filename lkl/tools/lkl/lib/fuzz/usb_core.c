#include "usb_core.h"

void usb_device_reset(USBDevice *dev) {
	if (dev == NULL || !dev->attached) {
		return;
	}
	usb_device_handle_reset(dev);
	dev->remote_wakeup = 0;
	dev->addr = 0;
	dev->state = USB_STATE_DEFAULT;
}