#include <sys/queue.h>
#include <stdbool.h>
#include "via_uhci_dev.h" 

#define USB_SPEED_LOW 0
#define USB_SPEED_FULL 1
#define USB_SPEED_HIGH 2
#define USB_SPEED_SUPER 3

typedef struct USBPort USBPort;
typedef struct USBDevice USBDevice;
typedef struct USBPacket USBPacket;
typedef struct USBCombinedPacket USBCombinedPacket;
typedef struct USBEndpoint USBEndpoint;

#define USB_MAX_ENDPOINTS  15
#define USB_MAX_INTERFACES 16

struct USBEndpoint {
	uint8_t nr;
	uint8_t pid;
	uint8_t type;
	uint8_t ifnum;
	int max_packet_size;
	int max_streams;
	bool pipeline;
	bool halted;
	USBDevice *dev;
	TAILQ_HEAD(, USBPacket) queue;
};

struct USBDevice {
	//DeviceState qdev;
	USBPort *port;
	char *port_path;
	char *serial;
	void *opaque;
	uint32_t flags;

	char *pcap_filename;
	FILE *pcap;

	// actual connected speed
	int speed;
	// supported speeds
	int speedmask;
	uint8_t addr;
	char product_desc[32];
	int auto_attach;
	bool attached;

	int32_t state;
	uint8_t setup_buf[8];
	uint8_t data_buf[4096];
	int32_t remote_wakeup;
	int32_t setup_state;
	int32_t setup_len;
	int32_t setup_index;

	USBEndpoint ep_ctl;
	USBEndpoint ep_in[USB_MAX_ENDPOINTS];
	USBEndpoint ep_out[USB_MAX_ENDPOINTS];

	//LIST_HEAD(, USBDescString) strings;
	//const USBDesc *usb_desc;
	//const USBDescDevice *device;

	int configuration;
	int ninterfaces;
	int altsetting[USB_MAX_INTERFACES];
	//const USBDescConfig *config;
	//const USBDescIface *ifaces[USB_MAX_INTERFACES];
};

OBJECT_DECLARE_TYPE(USBDevice, USBDeviceClass, USB_DEVICE);

// typedef void (*USBDeviceRealize)(USBDevice *dev, Error **errp);
// typedef void (*USBDeviceUnrealize)(USBDevice *dev);

struct USBDeviceClass {
	//DeviceClass parent_class;

	// USBDeviceRealize realize;
	// USBDeviceUnrealize unrealize;

	USBDevice *(*find_device)(USBDevice *dev, uint8_t addr);
	void (*cancel_packet)(USBDevice *dev, USBPacket *p);

	void (*handle_attach)(USBDevice *dev);

	void (*handle_reset)(USBDevice *dev);

	void (*handle_control)(USBDevice *dev, USBPacket *p, int request, int value, int index, int length, uint8_t *data);

	void (*handle_data)(USBDevice *dev, USBPacket *p);

	void (*set_interface)(USBDevice *dev, int interface, int alt_old, int alt_new);

	void (*flush_ep_queue)(USBDevice *dev, USBEndpoint *ep);

	void (*ep_stopped)(USBDevice *dev, USBEndpoint *ep);

	int (*alloc_streams)(USBDevice *dev, USBEndpoint **eps, int nr_eps, int streams);

	void (*free_streams)(USBDevice *dev, USBEndpoint **eps, int nr_eps);

	const char *product_desc;
	//const USBDesc *usb_desc;
	bool attached_settable;
};

typedef struct USBPortOps {
	void (*attach)(USBPort *port);
	void (*detach)(USBPort *port);

	void (*child_detach)(USBPort *port, USBDevice *child);
	void (*wakeup)(USBPort *port);

	void (*complete)(USBPort *port, USBPacket *p);
} USBPortOps;

struct USBPort {
	USBDevice *dev;
	int speedmask;
	int hubcount;
	char path[16];
	USBPortOps *ops;
	void *opaque;
	int index;
	TAILQ_ENTRY(USBPort) next;
};

typedef enum USBPacketState {
	USB_PACKET_UNDEFINED = 0,
	USB_PACKET_SETUP,
	USB_PACKET_QUEUED,
	USB_PACKET_ASYNC,
	USB_PACKET_COMPLETE,
	USB_PACKET_CANCELLED,
} USBPacketState;

struct USBPacket {
	// data fields for use by the driver
	int pid;
	uint64_t id;
	USBEndpoint *ep;
	unsigned int stream;
	//QEMUIOVector iov;
	uint64_t parameter; // control transfers
	bool short_not_ok;
	bool int_req;
	int status; // USB_RET_* status code
	int actual_length; // number of bytes actually transferred

	// internal use by the USB layer
	USBPacketState state;
	USBCombinedPacket *combined;
	TAILQ_ENTRY(USBPacket) queue;
	TAILQ_ENTRY(USBPacket) combined_entry;
};

struct USBCombinedPacket {
	USBPacket *first;
	TAILQ_HEAD(, USBPacket) packets;
	//QEMUIOVector iov;
};

void usb_device_handle_reset(USBDevice *dev);