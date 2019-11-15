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
/** library for USB DFU to write on internal flash (code)
 *  @file usb_dfu.c
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2017
 */

/* standard libraries */
#include <stdint.h> // standard integer types
#include <stdlib.h> // general utilities

/* STM32 (including CM3) libraries */
#include <libopencmsis/core_cm3.h> // Cortex M3 utilities
#include <libopencm3/cm3/scb.h> // reset utilities
#include <libopencm3/stm32/rcc.h> // real-time control clock library
#include <libopencm3/stm32/gpio.h> // general purpose input output library
#include <libopencm3/usb/usbd.h> // USB library
#include <libopencm3/usb/dfu.h> // USB DFU library

#include "global.h" // global utilities
#include "usb_dfu.h" // USB DFU header and definitions
#include "flash_internal.h" // flash reading/writing utilities

static uint8_t usbd_control_buffer[1024] = {0}; /**< buffer to be used for control requests (fit to flash page size) */
static usbd_device *usb_device = NULL; /**< structure holding all the info related to the USB device */
static enum dfu_state usb_dfu_state = STATE_DFU_IDLE; /**< current DFU state */
static enum dfu_status usb_dfu_status = DFU_STATUS_OK; /**< current DFU status */

static uint8_t download_data[sizeof(usbd_control_buffer)] = {0}; /**< downloaded data to be programmed in flash */
static uint16_t download_length = 0; /**< length of downloaded data */
static uint32_t flash_pointer = 0; /**< where the downloaded data should be flashed */

/** USB DFU device descriptor
 *  @note as defined in USB Device Firmware Upgrade specification section 4.2.1
 */
static const struct usb_device_descriptor usb_dfu_device = {
	.bLength = USB_DT_DEVICE_SIZE, /**< the size of this header in bytes, 18 */
	.bDescriptorType = USB_DT_DEVICE, /**< a value of 1 indicates that this is a device descriptor */
	.bcdUSB = 0x0200, /**< this device supports USB 2.0 */
	.bDeviceClass = 0, /**< unused */
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

/** USB DFU functional descriptor
 *  @note as defined in USB Device Firmware Upgrade specification section 4.2.4
 */
static const struct usb_dfu_descriptor usb_dfu_functional = {
	.bLength = sizeof(struct usb_dfu_descriptor), /**< provide own size */
	.bDescriptorType = DFU_FUNCTIONAL, /**< functional descriptor type */
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH, /**< this DFU can download and will detach after download (we don't support manifest for simplicity, technically we could) */
	.wDetachTimeout = 200, /**< maximum time in milliseconds to detach (and reboot) */
	.wTransferSize = sizeof(usbd_control_buffer), /**< set max transfer size */
	.bcdDFUVersion = 0x0110, /**< DFU specification version 1.1 used */
};

/** USB DFU interface descriptor
 *  @note as defined in USB Device Firmware Upgrade specification section 4.2.3
 */
static const struct usb_interface_descriptor usb_dfu_interface = {
	.bLength = USB_DT_INTERFACE_SIZE, /**< size of descriptor in byte */
	.bDescriptorType = USB_DT_INTERFACE, /**< interface descriptor type */
	.bInterfaceNumber = 0, /**< this interface is the first (and only) */
	.bAlternateSetting = 0, /**< no alternative settings */
	.bNumEndpoints = 0, /**< only the control pipe at endpoint 0 is used */
	.bInterfaceClass = 0xFE, /**< DFU interface class (not defined in libopencm3 dfu lib) */
	.bInterfaceSubClass = 1, /**< DFU interface subclass (not defined in libopencm3 dfu lib) */
	.bInterfaceProtocol = 2,  /**< DFU interface mode protocol (not defined in libopencm3 dfu lib) */
	.iInterface = 4, /**< the index of the string in the string table that represents interface description */
	.extra = &usb_dfu_functional, /**< point to functional descriptor */
	.extralen = sizeof(usb_dfu_functional), /**< size of functional descriptor */
};

/** USB DFU interface descriptor list */
static const struct usb_interface usb_dfu_interfaces[] = {{
	.num_altsetting = 1, /**< this is the only alternative */
	.altsetting = &usb_dfu_interface, /**< point to only interface descriptor */
}};

/** USB DFU configuration descriptor
 *  @note as defined in USB Device Firmware Upgrade specification section 4.2.2
 */
static const struct usb_config_descriptor usb_dfu_configuration = {
	.bLength = USB_DT_CONFIGURATION_SIZE, /**< the length of this header in bytes */
	.bDescriptorType = USB_DT_CONFIGURATION, /**< a value of 2 indicates that this is a configuration descriptor */
	.wTotalLength = 0, /**< total size of the configuration descriptor including all sub interfaces (automatically filled in by the USB stack in libopencm3) */
	.bNumInterfaces = LENGTH(usb_dfu_interfaces), /**< the number of interfaces in this configuration */
	.bConfigurationValue = 1, /**< the index of this configuration */
	.iConfiguration = 0, /**< a string index describing this configuration (zero means not provided) */
	.bmAttributes = 0x80, /**< bus powered (1<<7) */
	.bMaxPower = 0x32, /**< the maximum amount of current that this device will draw in 2mA units */
	// end of header
	.interface = usb_dfu_interfaces, /**< pointer to an array of interfaces */
};

/** USB string table
 *  @note starts with index 1
 */
static const char *usb_dfu_strings[] = {
	"CuVoodoo",
	"STM32F1",
	"DFU",
	"CuVoodoo DFU bootloader (DFU mode)",
};

/** disconnect USB to force re-enumerate */
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

/** flash downloaded data block
 *  @param[in] usbd_dev USB device (unused)
 *  @param[in] req USB request (unused)
 *  @note this function is called after the corresponding GETSTATUS request
 */
static void usb_dfu_flash(usbd_device *usbd_dev, struct usb_setup_data *req)
{
	(void)usbd_dev; // variable not used
	(void)req; // variable not used
	led_off(); // indicate we are processing
	if (flash_internal_write(flash_pointer, download_data, download_length)) { // write downloaded data
		flash_pointer += download_length; // go to next segment
		usb_dfu_state = STATE_DFU_DNLOAD_IDLE; // go back to idle stat to wait for next segment
	} else { // warn about writing error
		usb_dfu_status = DFU_STATUS_ERR_WRITE;
		usb_dfu_state = STATE_DFU_ERROR;
	}
	led_on(); // indicate we finished processing
}

/** disconnect USB and perform system reset
 *  @param[in] usbd_dev USB device (unused)
 *  @param[in] req USB request (unused)
 *  @note this function is called after the corresponding GETSTATUS request
 */
static void usb_dfu_reset(usbd_device *usbd_dev, struct usb_setup_data *req)
{
	(void)usbd_dev; // variable not used
	(void)req; // variable not used
	usb_disconnect(); // USB detach (disconnect to force re-enumeration)
	scb_reset_system(); // reset device
	while (true); // wait for the reset to happen
}

/** handle incoming USB DFU control request
 *  @param[in] usbd_dev USB device descriptor
 *  @param[in] req control request information
 *  @param[in] buf control request data
 *  @param[in] len control request data length
 *  @param[in] complete not used
 *  @return 0 if succeeded, error else
 *  @note resets device when configured with 5 bits
 */
static int usb_dfu_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)usbd_dev; // device is not used

	// DFU only requires handling class requests
	if ((req->bmRequestType & USB_REQ_TYPE_TYPE)!=USB_REQ_TYPE_CLASS) {
		return 0;
	}

	led_off(); // indicate we are processing request
	int to_return = 1; // value to return
	switch (req->bRequest) {
		case DFU_DETACH: // USB detach requested
			*complete = usb_dfu_reset; // reset after reply
			break;
		case DFU_DNLOAD: // download firmware on flash
			if (STATE_DFU_IDLE!=usb_dfu_state && STATE_DFU_DNLOAD_IDLE!=usb_dfu_state) { // wrong start to request download
				// warn about programming error
				usb_dfu_status = DFU_STATUS_ERR_PROG;
				usb_dfu_state = STATE_DFU_ERROR;
			} else if (STATE_DFU_IDLE==usb_dfu_state && ((NULL==len) || (0 == *len))) { // download request should not start empty
				// warn about programming error
				usb_dfu_status = DFU_STATUS_ERR_PROG;
				usb_dfu_state = STATE_DFU_ERROR;
			} else if (STATE_DFU_DNLOAD_IDLE==usb_dfu_state && ((NULL==len) || (0 == *len))) { // download completed
				// go to manifestation phase
				usb_dfu_state = STATE_DFU_MANIFEST_SYNC;
			} else { // there is data to be flashed
				if (*len%2) {
					// we can only write half words
					usb_dfu_status = DFU_STATUS_ERR_PROG;
					usb_dfu_state = STATE_DFU_ERROR;
				} else if (flash_pointer+*len>=(uint32_t)&__application_end) {
					// application data is too large
					usb_dfu_status = DFU_STATUS_ERR_ADDRESS;
					usb_dfu_state = STATE_DFU_ERROR;
				} else {
					// save downloaded data to be flashed
					for (uint16_t i=0; i<*len && i<sizeof(download_data); i++) {
						download_data[i] = (*buf)[i];
					}
					download_length = *len;
					usb_dfu_state = STATE_DFU_DNLOAD_SYNC; // go to sync state
					*complete = usb_dfu_flash; // start flashing the downloaded data
				}
			}
			break;
		case DFU_UPLOAD: // upload firmware from flash
			to_return = 0; // upload no supported
			break;
		case DFU_GETSTATUS: // get status
			(*buf)[0] = usb_dfu_status; // set status
			(*buf)[1] = 100; // set poll timeout (24 bits, in milliseconds) to small value for periodical poll
			(*buf)[2] = 0; // set poll timeout (24 bits, in milliseconds) to small value for periodical poll
			(*buf)[3] = 0; // set poll timeout (24 bits, in milliseconds) to small value for periodical poll
			(*buf)[4] = usb_dfu_state; // set state
			(*buf)[5] = 0; // string not used
			*len = 6; // set length of buffer to return
			if (STATE_DFU_DNLOAD_SYNC==usb_dfu_state) {
				usb_dfu_state = STATE_DFU_DNBUSY; // switch to busy state
			} else if (STATE_DFU_MANIFEST_SYNC==usb_dfu_state) {
				usb_dfu_state = STATE_DFU_MANIFEST; // go to manifest mode
				led_off(); // indicate the end
				*complete = usb_dfu_reset; // start reset without waiting for request since we advertised we would detach 
			}
			break;
		case DFU_CLRSTATUS: // clear status
			if (STATE_DFU_ERROR==usb_dfu_state || DFU_STATUS_OK!=usb_dfu_status) { // only clear in case there is an error
				usb_dfu_status = DFU_STATUS_OK; // clear error status
				usb_dfu_state = STATE_APP_IDLE; // put back in idle state
			}
			break;
		case DFU_GETSTATE: // get state
			(*buf)[0] = usb_dfu_state; // return state
			*len = 1; // only state needs to be provided
			break;
		case DFU_ABORT: // abort current operation
			usb_dfu_state = STATE_APP_IDLE; // put back in idle state (nothing else to do)
			flash_pointer = (uint32_t)&__application_beginning; // reset download location
			break;
		default:
			to_return = 0;
	}
	led_on(); // indicate we finished processing

	return to_return;
}

void usb_dfu_setup(void)
{
	flash_pointer = (uint32_t)&__application_beginning; // set download destination to beginning of application in flash
	rcc_periph_reset_pulse(RST_USB); // reset USB peripheral
	usb_disconnect(); // disconnect to force re-enumeration
	rcc_periph_clock_enable(RCC_GPIOA); // enable clock for GPIO used for USB
	rcc_periph_clock_enable(RCC_USB); // enable clock for USB domain
	usb_device = usbd_init(&st_usbfs_v1_usb_driver, &usb_dfu_device, &usb_dfu_configuration, usb_dfu_strings, LENGTH(usb_dfu_strings), usbd_control_buffer, sizeof(usbd_control_buffer)); // configure USB device
	usbd_register_control_callback(usb_device, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE, USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, usb_dfu_control_request); // set control request handling DFU operations
}

void usb_dfu_start(void)
{
	// infinitely poll device to handle requests
	while (true) {
		usbd_poll(usb_device);
	}
}
