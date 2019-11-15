/* This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/** library for USB CDC ACM communication (code)
 *  @file usb_cdcacm.c
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2016-2017
 */

/* standard libraries */
#include <stdint.h> // standard integer types
#include <stdio.h> // standard I/O facilities
#include <stdlib.h> // general utilities

/* STM32 (including CM3) libraries */
#include <libopencm3/stm32/rcc.h> // real-time control clock library
#include <libopencm3/stm32/gpio.h> // general purpose input output library
#include <libopencm3/cm3/nvic.h> // interrupt handler
#include <libopencm3/cm3/scb.h> // reset utilities
#include <libopencmsis/core_cm3.h> // Cortex M3 utilities
#include <libopencm3/usb/usbd.h> // USB library
#include <libopencm3/usb/cdc.h> // USB CDC library
#include <libopencm3/cm3/sync.h> // synchronisation utilities
#include <libopencm3/usb/dfu.h> // DFU definitions

#include "global.h" // global utilities
#include "usb_cdcacm.h" // USB CDC ACM header and definitions

static uint8_t usbd_control_buffer[128] = {0}; /**< buffer to be used for control requests */
static usbd_device *usb_device = NULL; /**< structure holding all the info related to the USB device */

/** USB CDC ACM device descriptor
 *  @note as defined in USB CDC specification section 5
 */
static const struct usb_device_descriptor usb_cdcacm_device_descriptor = {
	.bLength = USB_DT_DEVICE_SIZE, /**< the size of this header in bytes, 18 */
	.bDescriptorType = USB_DT_DEVICE, /**< a value of 1 indicates that this is a device descriptor */
	.bcdUSB = 0x0200, /**< this device supports USB 2.0 */
	.bDeviceClass = USB_CLASS_CDC, /**< use the CDC device class */
	.bDeviceSubClass = 0, /**< unused */
	.bDeviceProtocol = 0, /**< unused */
	.bMaxPacketSize0 = 64, /**< packet size for endpoint zero in bytes */
	.idVendor = 0xc440, /**< Vendor ID (CuVo...) */
	.idProduct = 0x0d00, /**< product ID within the Vendor ID space (...odoo) */
	.bcdDevice = 0x0100, /**< version number for the device */
	.iManufacturer = 1, /**< the index of the string in the string table that represents the name of the manufacturer of this device */
	.iProduct = 2, /**< the index of the string in the string table that represents the name of the product */
	.iSerialNumber = 3, /**< the index of the string in the string table that represents the serial number of this item in string form */
	.bNumConfigurations = 1, /**< the number of possible configurations this device has */
};

/** USB CDC ACM data endpoints
 *  @note as defined in USB CDC specification section 5
 */
static const struct usb_endpoint_descriptor usb_cdcacm_data_endpoints[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE, /**< the size of the endpoint descriptor in bytes */
	.bDescriptorType = USB_DT_ENDPOINT, /**< a value of 5 indicates that this describes an endpoint */
	.bEndpointAddress = 0x02, /**< OUT (from host) direction (0<<7), endpoint 2 */
	.bmAttributes = USB_ENDPOINT_ATTR_BULK, /**< bulk mode */
	.wMaxPacketSize = 64, /**< maximum packet size */
	.bInterval = 1, /**< the frequency, in number of frames, that we're going to be sending data */
},{
	.bLength = USB_DT_ENDPOINT_SIZE, /**< the size of the endpoint descriptor in bytes */
	.bDescriptorType = USB_DT_ENDPOINT, /**< a value of 5 indicates that this describes an endpoint */
	.bEndpointAddress = 0x82, /**< IN (to host) direction (1<<7), endpoint 2 */
	.bmAttributes = USB_ENDPOINT_ATTR_BULK, /**< bulk mode */
	.wMaxPacketSize = 64, /**< maximum packet size */
	.bInterval = 1, /**< the frequency, in number of frames, that we're going to be sending data */
}};

/** USB CDC ACM communication endpoints
 * @note This notification endpoint isn't implemented. According to CDC spec its optional, but its absence causes a NULL pointer dereference in Linux cdc_acm driver
 */
static const struct usb_endpoint_descriptor usb_cdcacm_communication_endpoints[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE, /**< the size of the endpoint descriptor in bytes */
	.bDescriptorType = USB_DT_ENDPOINT, /**< a value of 5 indicates that this describes an endpoint */
	.bEndpointAddress = 0x81, /**< IN (to host) direction (1<<7), endpoint 1 */
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT, /**< interrupt mode */
	.wMaxPacketSize = 16, /**< maximum packet size */
	.bInterval = 255, /**< the frequency, in number of frames, that we're going to be sending data */
}};

/** USB CDC ACM functional descriptor
 *  @return
 *  @note as defined in USB CDC specification section 5.2.3
 */
static const struct {
	struct usb_cdc_header_descriptor header; /**< header */
	struct usb_cdc_call_management_descriptor call_mgmt; /**< call management descriptor */
	struct usb_cdc_acm_descriptor acm; /**< descriptor */
	struct usb_cdc_union_descriptor cdc_union;  /**< descriptor */
} __attribute__((packed)) usb_cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor), /**< descriptor length */
		.bDescriptorType = CS_INTERFACE, /**< descriptor type */
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER, /**< descriptor subtype */
		.bcdCDC = 0x0110, /**< CDC value */
	},
	.call_mgmt = {
		.bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor), /**< descriptor length */
		.bDescriptorType = CS_INTERFACE,  /**< descriptor type */
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT, /**< descriptor subtype */
		.bmCapabilities = 0, /**< capabilities */
		.bDataInterface = 1, /**< data interface */
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor), /**< descriptor length */
		.bDescriptorType = CS_INTERFACE, /**< descriptor type */
		.bDescriptorSubtype = USB_CDC_TYPE_ACM, /**< descriptor subtype */
		.bmCapabilities = 0, /**< capabilities */
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor), /**< descriptor length */
		.bDescriptorType = CS_INTERFACE, /**< descriptor type */
		.bDescriptorSubtype = USB_CDC_TYPE_UNION, /**< descriptor subtype */
		.bControlInterface = 0, /**< control interface */
		.bSubordinateInterface0 = 1, /**< subordinate interface */
	 },
};

/** USB CDC interface descriptor
 *  @note as defined in USB CDC specification section 5.1.3
 */
static const struct usb_interface_descriptor usb_cdcacm_communication_interface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,
	.iInterface = 0,

	.endpoint = usb_cdcacm_communication_endpoints,

	.extra = &usb_cdcacm_functional_descriptors,
	.extralen = sizeof(usb_cdcacm_functional_descriptors),
};

/** USB CDC ACM data class interface descriptor
 *  @note as defined in USB CDC specification section 5.1.3
 */
static const struct usb_interface_descriptor usb_cdcacm_data_interface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = usb_cdcacm_data_endpoints,
};

/** USB DFU functional descriptor
 *  @note as defined in USB Device Firmware Upgrade specification section 4.2.4
 */
static const struct usb_dfu_descriptor usb_dfu_functional = {
	.bLength = sizeof(struct usb_dfu_descriptor), /**< provide own size */
	.bDescriptorType = DFU_FUNCTIONAL, /**< functional descriptor type */
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH, /**< this DFU can download and will detach after download */
	.wDetachTimeout = 200, /**< maximum time (in milliseconds) needed to detach (else resume normal operation) */
	.wTransferSize = sizeof(usbd_control_buffer), /**< set max transfer size */
	.bcdDFUVersion = 0x0110, /**< DFU specification version 1.1 used */
};

/** USB DFU interface descriptor
 *  @note as defined in USB Device Firmware Upgrade specification section 4.2.3
 */
static const struct usb_interface_descriptor usb_dfu_interface = {
	.bLength = USB_DT_INTERFACE_SIZE, /**< size of descriptor in byte */
	.bDescriptorType = USB_DT_INTERFACE, /**< interface descriptor type */
	.bInterfaceNumber = 2, /**< interface number in the list */
	.bAlternateSetting = 0, /**< no alternative settings */
	.bNumEndpoints = 0, /**< only the control pipe at endpoint 0 is used */
	.bInterfaceClass = 0xFE, /**< DFU interface class (not defined in libopencm3 dfu lib) */
	.bInterfaceSubClass = 1, /**< DFU interface subclass (not defined in libopencm3 dfu lib) */
	.bInterfaceProtocol = 1,  /**< runtime protocol (not defined in libopencm3 dfu lib) */
	.iInterface = 4, /**< the index of the string in the string table that represents interface description */
	.extra = &usb_dfu_functional, /**< point to functional descriptor */
	.extralen = sizeof(usb_dfu_functional), /**< size of functional descriptor */
};

/** USB CDC ACM interface descriptor */
static const struct usb_interface usb_cdcacm_interfaces[] = {{
	.num_altsetting = 1,
	.altsetting = &usb_cdcacm_communication_interface,
}, {
	.num_altsetting = 1,
	.altsetting = &usb_cdcacm_data_interface,
}, {
	.num_altsetting = 1,
	.altsetting = &usb_dfu_interface,
}};

/** USB CDC ACM configuration descriptor */
static const struct usb_config_descriptor usb_cdcacm_configuration_descriptor = {
	.bLength = USB_DT_CONFIGURATION_SIZE, /**< the length of this header in bytes */
	.bDescriptorType = USB_DT_CONFIGURATION, /**< a value of 2 indicates that this is a configuration descriptor */
	.wTotalLength = 0, /**< this should hold the total size of the configuration descriptor including all sub interfaces. it is automatically filled in by the USB stack in libopencm3 */
	.bNumInterfaces = LENGTH(usb_cdcacm_interfaces), /**< the number of interfaces in this configuration */
	.bConfigurationValue = 1, /**< the index of this configuration */
	.iConfiguration = 0, /**< a string index describing this configuration (zero means not provided) */
	.bmAttributes = 0x80, /**< bus powered (1<<7) */
	.bMaxPower = 0x32, /**< the maximum amount of current that this device will draw in 2mA units */
	// end of header
	.interface = usb_cdcacm_interfaces, /**< pointer to an array of interfaces */
};

/** USB string table
 *  @note starting with index 1
 */
static const char *usb_strings[] = {
	"CuVoodoo",
	"STM32F1",
	"CDC-ACM",
	"CuVoodoo DFU bootloader (runtime mode)",
};

/* input and output ring buffer, indexes, and available memory */
static uint8_t rx_buffer[CDCACM_BUFFER] = {0}; /**< ring buffer for received data */
static volatile uint8_t rx_i = 0; /**< current position of read received data */
static volatile uint8_t rx_used = 0; /**< how much data has been received and not red */
mutex_t rx_lock = MUTEX_UNLOCKED; /**< lock to update rx_i or rx_used */
static uint8_t tx_buffer[CDCACM_BUFFER] = {0}; /**< ring buffer for data to transmit */
static volatile uint8_t tx_i = 0; /**< current position if transmitted data */
static volatile uint8_t tx_used = 0; /**< how much data needs to be transmitted */
mutex_t tx_lock = MUTEX_UNLOCKED; /**< lock to update tx_i or tx_used */
volatile uint8_t usb_cdcacm_received = 0; // same as rx_used, but since the user can write this variable we don't rely on it
static bool connected = false; /**< is the USB device is connected to a host */

/** disconnect USB by pulling down D+ to for re-enumerate */
static void usb_disconnect(void)
{
#if defined(MAPLE_MINI)
	// disconnect USB D+ using dedicated DISC line/circuit on PB9
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
	gpio_set(GPIOB, GPIO9);
	for (uint32_t i = 0; i < 0x2000; i++) {
		__asm__("nop");
	}
	gpio_clear(GPIOB, GPIO9);
#else
	// pull USB D+ low for a short while
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	for (uint32_t i = 0; i < 0x2000; i++) {
		__asm__("nop");
	}
#endif
}

/** disconnect USB and perform system reset
 *  @param[in] usbd_dev USB device (unused)
 *  @param[in] req USB request (unused)
 */
static void usb_reset(usbd_device *usbd_dev, struct usb_setup_data *req)
{
	(void)usbd_dev; // variable not used
	(void)req; // variable not used
	usb_disconnect(); // USB detach (disconnect to force re-enumeration)
	scb_reset_system(); // reset device
	while (true); // wait for the reset to happen
}

/** DFU detach (disconnect USB and perform core reset)
 *  @param[in] usbd_dev USB device (unused)
 *  @param[in] req USB request (unused)
 */
static void usb_dfu_detach(usbd_device *usbd_dev, struct usb_setup_data *req)
{
	(void)usbd_dev; // variable not used
	(void)req; // variable not used
	RCC_CSR |= RCC_CSR_RMVF; // clear reset flag for the bootloader to detect the core reset
	usb_disconnect(); // USB detach (disconnect to force re-enumeration)
	scb_reset_core(); // reset device (only the core, to the peripheral stay configured)
	while (true); // wait for the reset to happen
}

/** incoming USB CDC ACM control request
 *  @param[in] usbd_dev USB device descriptor
 *  @param[in] req control request information
 *  @param[in] buf control request data
 *  @param[in] len control request data length
 *  @param[in] complete function to run after request completed
 *  @return 0 if succeeded, error else
 *  @note resets device when configured with 5 bits
 */
static int usb_cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	if (usb_dfu_interface.bInterfaceNumber==req->wIndex) { // check if request is for DFU
		switch (req->bRequest) {
			case DFU_DETACH: // USB detach requested
				*complete = usb_dfu_detach; // detach after reply 
				break;
			case DFU_GETSTATUS: // get status
				(*buf)[0] = DFU_STATUS_OK;; // set OK status
				(*buf)[1] = 0; // set null poll timeout
				(*buf)[2] = 0; // set null poll timeout
				(*buf)[3] = 0; // set null poll timeout
				(*buf)[4] = STATE_APP_IDLE; // application is running
				(*buf)[5] = 0; // string not used
				*len = 6; // set length of buffer to return
				break;
			default: // other requests are not supported
				return 0;
		}
	} else {
		switch (req->bRequest) {
			case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
				connected = req->wValue ? true : false; // check if terminal is open
				//bool dtr = (req->wValue & (1 << 0)) ? true : false;
				//bool rts = (req->wValue & (1 << 1)) ? true : false;
				/* this Linux cdc_acm driver requires this to be implemented
				 * even though it's optional in the CDC spec, and we don't
				 * advertise it in the ACM functional descriptor.
				 */
				uint8_t reply[10] = {0};
				struct usb_cdc_notification *notif = (void *)reply;
				/* we echo signals back to host as notification. */
				notif->bmRequestType = 0xA1;
				notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
				notif->wValue = 0;
				notif->wIndex = 0;
				notif->wLength = 2;
				reply[8] = req->wValue & 3;
				reply[9] = 0;
				usbd_ep_write_packet(usbd_dev, 0x81, reply, LENGTH(reply));
				break;
			case USB_CDC_REQ_SET_LINE_CODING:
				// ignore if length is wrong
				if (*len < sizeof(struct usb_cdc_line_coding)) {
					return 0;
				}
				// get the line coding
				struct usb_cdc_line_coding *coding = (struct usb_cdc_line_coding *)*buf;
				/* reset device is the data bits is set to 5
				 * to reset the device from the host you can use stty --file /dev/ttyACM0 raw cs5
				 */
				if (coding->bDataBits==5) {
					*complete = usb_reset; // perform reset after reply
				}
				break;
			default:
				return 0;
		}
	}
	return 1;
}

/** USB CDC ACM data received callback
 *  @param[in] usbd_dev USB device descriptor
 *  @param[in] ep endpoint where data came in
 */
static void usb_cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	char usb_data[64] = {0}; // buffer to read data
	uint16_t usb_length = 0; // length of incoming data
	
	/* receive data */
	usb_length = usbd_ep_read_packet(usbd_dev, 0x02, usb_data, sizeof(usb_data));
	if (usb_length) { // copy received data
		for (uint16_t i=0; i<usb_length && rx_used<LENGTH(rx_buffer); i++) { // only until buffer is full
			rx_buffer[(rx_i+rx_used)%LENGTH(rx_buffer)] = usb_data[i]; // put character in buffer
			rx_used++; // update used buffer
		}
		usb_cdcacm_received = rx_used; // update available data
	}
}

/** USB CDC ACM data transmitted callback
 *  @param[in] usbd_dev USB device descriptor
 *  @param[in] ep endpoint where data came in
 */
static void usb_cdcacm_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;
	(void)usbd_dev;

	if (!usbd_dev || !connected || !tx_used) { // verify if we can send and there is something to send
		return;
	}
	if (mutex_trylock(&tx_lock)) { // try to get lock
		uint8_t usb_length = (tx_used > 64 ? 64 : tx_used); // length of data to be transmitted (respect max packet size)
		usb_length = (usb_length > (LENGTH(tx_buffer)-tx_i) ? LENGTH(tx_buffer)-tx_i : usb_length); // since here we use the source array not as ring buffer, only go up to the end
		while (usb_length != usbd_ep_write_packet(usb_device, 0x82, (void*)(&tx_buffer[tx_i]), usb_length)); // ensure data is written into transmit buffer
		tx_i = (tx_i+usb_length)%LENGTH(tx_buffer); // update location on buffer
		tx_used -= usb_length; // update used size
		mutex_unlock(&tx_lock); // release lock
	} else {
		usbd_ep_write_packet(usb_device, 0x82, NULL, 0); // trigger empty tx for a later callback
	}
	usbd_poll(usb_device); // ensure the data gets sent
}

/** set USB CDC ACM configuration
 *  @param[in] usbd_dev USB device descriptor
 *  @param[in] wValue not used
 */
static void usb_cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);
	usbd_ep_setup(usbd_dev, 0x02, USB_ENDPOINT_ATTR_BULK, 64, usb_cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, usb_cdcacm_data_tx_cb);

	usbd_register_control_callback( usbd_dev, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE, USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, usb_cdcacm_control_request);
}
	
void usb_cdcacm_setup(void)
{
	connected = false; // start with USB not connected

	// initialize USB
	rcc_periph_reset_pulse(RST_USB); // reset USB peripheral
	usb_disconnect(); // disconnect to force re-enumeration
	rcc_periph_clock_enable(RCC_GPIOA); // enable clock for GPIO used for USB
	rcc_periph_clock_enable(RCC_USB); // enable clock for USB domain
	usb_device = usbd_init(&st_usbfs_v1_usb_driver, &usb_cdcacm_device_descriptor, &usb_cdcacm_configuration_descriptor, usb_strings, LENGTH(usb_strings), usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usb_device, usb_cdcacm_set_config);
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ); // enable interrupts (to not have to poll all the time)
    
    // reset buffer states
	rx_i = 0;
	rx_used = 0;
	mutex_unlock(&rx_lock);
	usb_cdcacm_received = 0;
	tx_i = 0;
	tx_used = 0;
	mutex_unlock(&tx_lock);
}

char usb_cdcacm_getchar(void)
{
	while (!rx_used) { // idle until data is available
		__WFI(); // sleep until interrupt (not sure if it's a good idea here)
	}
	char to_return = rx_buffer[rx_i]; // get the next available character
	rx_i = (rx_i+1)%LENGTH(rx_buffer); // update used buffer
	rx_used--; // update used buffer
	usb_cdcacm_received = rx_used; // update available data
	return to_return;
}

void usb_cdcacm_putchar(char c)
{
	if (!usb_device || !connected) {
		return;
	}
	mutex_lock(&tx_lock); // get lock to prevent race condition
	if (tx_used<LENGTH(tx_buffer)) { // buffer not full
		tx_buffer[(tx_i+tx_used)%LENGTH(tx_buffer)] = c; // put character in buffer
		tx_used++; // update used buffer
	} else { // buffer full (might be that no terminal is connected to this serial)
		tx_i = (tx_i+1)%LENGTH(tx_buffer); // shift start
		tx_buffer[(tx_i+tx_used)%LENGTH(tx_buffer)] = c; // overwrite old data
	}
	mutex_unlock(&tx_lock); // release lock
	if (tx_used==1) { // to buffer is not empty anymore
		usbd_ep_write_packet(usb_device, 0x82, NULL, 0); // trigger tx callback
	}
}

/** USB interrupt service routine called when data is received */
void usb_lp_can_rx0_isr(void) {
	usbd_poll(usb_device);
}
