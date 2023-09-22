void usb_device_handle_reset(USBDevice *dev) {
	USBDeviceClass *klass = USB_DEVICE_GET_CLASS(dev);
	if (klass->handle_reset) {
		klass->handle_reset(dev);
	}
}