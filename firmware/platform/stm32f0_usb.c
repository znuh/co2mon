/*
 * This file is based on the cdcacm.c file of the libopencm3 project:
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * 
 * Changes for co2mon Copyright (C) 2020 Benedikt Heinz <hunz@mailbox.org>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/crs.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <stdlib.h>
#include <string.h>
#include "utils.h"

static const struct usb_device_descriptor dev = {
  .bLength = USB_DT_DEVICE_SIZE,
  .bDescriptorType = USB_DT_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = USB_CLASS_CDC,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = 64,
  .idVendor = 0x0483,
  .idProduct = 0x5740,
  .bcdDevice = 0x0200,
  .iManufacturer = 1,
  .iProduct = 2,
  .iSerialNumber = 3,
  .bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	 },
};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors),
}};

static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"znu.nz/co2mon",
	"CO2 monitor",
	"0",
};

int  usb_tx(const void *p, size_t n);
void usb_setup(void);

static int usb_active = 0;
static usbd_device *usb_dev = NULL;

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req)) {
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		char local_buf[10];
		struct usb_cdc_notification *notif = (void *)local_buf;

		/* We echo signals back to host as notification. */
		notif->bmRequestType = 0xA1;
		notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
		notif->wValue = 0;
		notif->wIndex = 0;
		notif->wLength = 2;
		local_buf[8] = req->wValue & 3;
		local_buf[9] = 0;
		// usbd_ep_write_packet(0x83, buf, 10);
		return USBD_REQ_HANDLED;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding))
			return USBD_REQ_NOTSUPP;
		return USBD_REQ_HANDLED;
	}
	return USBD_REQ_NOTSUPP;
}

static const char bl_string[] = "ICANHAZBOOTLOADER";

extern void start_bootloader(void);

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep) {
	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);
	if((len >= (int)(sizeof(bl_string)-1)) && (!memcmp(buf,bl_string,sizeof(bl_string)-1))) {
		usbd_disconnect(usbd_dev, true);
		gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO11 | GPIO12);
		start_bootloader();
	}
	ep=ep;
}

#define USB_TXBUF_SZ          128
static uint32_t usb_tx_put    = 0;
static uint32_t usb_tx_get    = 0;
static uint32_t usb_tx_fill   = 0;
static uint8_t  usb_txbuf[USB_TXBUF_SZ];
static uint32_t usb_tx_active = 0;

static void cdcacm_data_tx_cb(usbd_device *usbd_dev, uint8_t ep) {
	if((!usb_active) || (!usb_tx_fill)) {
		usb_tx_active = 0;
		return;
	}

	usb_tx_active = 1;
	ep&=0x7f;
	if ((*USB_EP_REG(ep) & USB_EP_TX_STAT) != USB_EP_TX_STAT_VALID) { /* check if busy */
		volatile uint16_t *PM = (volatile void *)USB_GET_EP_TX_BUFF(ep);
		uint32_t i, len = MIN(usb_tx_fill, 64);
		uint16_t buf = 0;

		/* write 16bit words */
		for(i=0;i<len;i++) {
			buf>>=8;
			buf|=usb_txbuf[usb_tx_get++]<<8;
			usb_tx_get&=(USB_TXBUF_SZ-1);
			if(i&1)
				*PM++ = buf;
		}

		/* write remaining byte if odd length */
		if(len&1)
			*PM++ = buf>>8;

		USB_SET_EP_TX_COUNT(ep, len);
		USB_SET_EP_TX_STAT(ep, USB_EP_TX_STAT_VALID);

		usb_tx_fill -= len;
	}
	usbd_dev = usbd_dev; /* mute compiler warning */
}

int usb_tx(const void *p, size_t n) {
	const uint8_t *d = p;
	int res;

	nvic_disable_irq(NVIC_USB_IRQ);

	res = MIN(USB_TXBUF_SZ - usb_tx_fill, n);

	for(n=res;n;n--,d++) {
		usb_txbuf[usb_tx_put++] = *d;
		usb_tx_put &= (USB_TXBUF_SZ-1);
	}

	usb_tx_fill += res;

	if((usb_tx_fill) && (!usb_tx_active))
		cdcacm_data_tx_cb(usb_dev, 0x82); /* send 1st chunk */

	nvic_enable_irq(NVIC_USB_IRQ);
	return res;
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue) {
	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_tx_cb);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);
	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
	wValue = wValue;
	usb_active = 1;
}

void usb_isr(void) {
	if(usb_dev)
		usbd_poll(usb_dev);
}

void usb_setup(void) {
	crs_autotrim_usb_enable();
	usb_dev = usbd_init(&st_usbfs_v2_usb_driver, &dev, &config, usb_strings,
						sizeof(usb_strings)/sizeof(char *),
						usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usb_dev, cdcacm_set_config);

	nvic_enable_irq(NVIC_USB_IRQ);
}
