/*
* lib for plutosdr
* Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
* Copyright (C) 2017 by Hoernchen <la@tfc-server.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#ifndef _WIN32
#include <unistd.h>
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#include <libusb.h>
#include "osmoplutosdr.h"

#ifndef LIBUSB_CALL
#define LIBUSB_CALL
#endif

#define PLUTO_USB_ITF_IDX 3
#define PLUTO_USB_READ_EPNUM 0x84

#define DEFAULT_BUF_NUMBER	15
#define DEFAULT_BUF_LENGTH	(512 * 16 * 100)

#define CTRL_TIMEOUT	300
#define BULK_TIMEOUT	0

#define PLUTO_CTL_OUT (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT | LIBUSB_RECIPIENT_INTERFACE)
#define PLUTO_CTL_IN (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN | LIBUSB_RECIPIENT_INTERFACE)

#define PLUTO_CTLTSZ 128
enum plutoctl_commands {
    ENABLE_BUFSTREAM=0,
    GLOBALPHYSTORE,
    RXLO,
    HWGAIN,     // milli-db!
    SAMPFREQ,
    GAINCTLMODE,    // always manual.
};


enum plutosdr_async_status {
    plutosdr_INACTIVE = 0,
    plutosdr_CANCELING,
    plutosdr_RUNNING
};


struct plutosdr_dev {
    struct libusb_context *ctx;
    struct libusb_device_handle *devh;
	uint32_t xfer_buf_num;
    uint32_t xfer_buf_len;
    struct libusb_transfer **xfer;
    unsigned char **xfer_buf;
	plutosdr_read_async_cb_t cb;
	void *cb_ctx;
	enum plutosdr_async_status async_status;
	int async_cancel;

    /* status */
    int dev_lost;
	int driver_active;
    unsigned int xfer_errors;
    int attached;

    //buf for ctrl transfers
    unsigned char ctrlbuf[PLUTO_CTLTSZ];
};


uint32_t plutosdr_get_device_count(void)
{
	int i, r;
	libusb_context *ctx;
	libusb_device **list;
	uint32_t device_count = 0;
	struct libusb_device_descriptor dd;
	ssize_t cnt;

	r = libusb_init(&ctx);
	if (r < 0)
		return 0;

	cnt = libusb_get_device_list(ctx, &list);

	for (i = 0; i < cnt; i++) {
		libusb_get_device_descriptor(list[i], &dd);

		if (0x0456 == dd.idVendor && 0xB673 == dd.idProduct)
			device_count++;
	}

	libusb_free_device_list(list, 1);

	libusb_exit(ctx);

	return device_count;
}


int plutosdr_open(plutosdr_dev_t **out_dev, uint32_t index)
{
    int r;
    int i;
    libusb_device **list;
    plutosdr_dev_t *dev = NULL;
    libusb_device *device = NULL;
    uint32_t device_count = 0;
    struct libusb_device_descriptor dd;
    ssize_t cnt;

    dev = malloc(sizeof(plutosdr_dev_t));
    if (NULL == dev)
        return -ENOMEM;

    memset(dev, 0, sizeof(plutosdr_dev_t));


    r = libusb_init(&dev->ctx);
    if(r < 0){
        free(dev);
        return -1;
    }

    dev->dev_lost = 1;

    cnt = libusb_get_device_list(dev->ctx, &list);

    for (i = 0; i < cnt; i++) {
        device = list[i];

        libusb_get_device_descriptor(list[i], &dd);

        if (0x0456 == dd.idVendor && 0xB673 == dd.idProduct)
            device_count++;
        

        if (index == device_count - 1)
            break;

        device = NULL;
    }

    if (!device) {
        r = -1;
        goto err;
    }

    r = libusb_open(device, &dev->devh);
    if (r < 0) {
        libusb_free_device_list(list, 1);
        fprintf(stderr, "usb_open error %d\n", r);
        if(r == LIBUSB_ERROR_ACCESS)
            fprintf(stderr, "Please fix the device permissions, e.g. "
            "by installing the udev rules file rtl-sdr.rules\n");
        goto err;
    }

    libusb_free_device_list(list, 1);

    if (libusb_kernel_driver_active(dev->devh, PLUTO_USB_ITF_IDX) == 1) {
        dev->driver_active = 1;

#ifdef DETACH_KERNEL_DRIVER
        if (!libusb_detach_kernel_driver(dev->devh, PLUTO_USB_ITF_IDX)) {
            fprintf(stderr, "Detached kernel driver\n");
        } else {
            fprintf(stderr, "Detaching kernel driver failed!");
            goto err;
        }
#else
        fprintf(stderr, "\nKernel driver is active, or device is "
                "claimed by second instance of librtlsdr."
                "\nIn the first case, please either detach"
                " or blacklist the kernel module\n"
                "(dvb_usb_rtl28xxu), or enable automatic"
                " detaching at compile time.\n\n");
#endif
    }

    r = libusb_claim_interface(dev->devh, PLUTO_USB_ITF_IDX);
    if (r < 0) {
        fprintf(stderr, "usb_claim_interface error %d\n", r);
        goto err;
    }

    dev->dev_lost = 0;
    *out_dev = dev;

    return 0;
err:
    if (dev) {
        if (dev->ctx)
            libusb_exit(dev->ctx);

        free(dev);
    }

    return r;
}


int plutosdr_close(plutosdr_dev_t *dev)
{
    if (!dev)
        return -1;

    if(!dev->dev_lost) {
        /* block until all async operations have been completed (if any) */
        while (plutosdr_INACTIVE != dev->async_status) {
#ifdef _WIN32
            Sleep(1);
#else
            usleep(1000);
#endif
        }

//        plutosdr_deinit_baseband(dev);
    }

    libusb_release_interface(dev->devh, PLUTO_USB_ITF_IDX);

#ifdef DETACH_KERNEL_DRIVER
    if (dev->driver_active) {
        if (!libusb_attach_kernel_driver(dev->devh, ITFIDX1))
            fprintf(stderr, "Reattached kernel driver\n");
        else
            fprintf(stderr, "Reattaching kernel driver failed!\n");
    }
#endif

    libusb_close(dev->devh);

    libusb_exit(dev->ctx);

    free(dev);

    return 0;
}

static void LIBUSB_CALL _libusb_callback(struct libusb_transfer *xfer)
{
	plutosdr_dev_t *dev = (plutosdr_dev_t *)xfer->user_data;

	if (LIBUSB_TRANSFER_COMPLETED == xfer->status) {
		if (dev->cb)
			dev->cb(xfer->buffer, xfer->actual_length, dev->cb_ctx);

		libusb_submit_transfer(xfer); /* resubmit transfer */
		dev->xfer_errors = 0;
	} else if (LIBUSB_TRANSFER_CANCELLED != xfer->status) {
#ifndef _WIN32
		if (LIBUSB_TRANSFER_ERROR == xfer->status)
			dev->xfer_errors++;

		if (dev->xfer_errors >= dev->xfer_buf_num ||
		    LIBUSB_TRANSFER_NO_DEVICE == xfer->status) {
#endif
			dev->dev_lost = 1;
			plutosdr_cancel_async(dev);
			fprintf(stderr, "cb transfer status: %d, "
				"canceling...\n", xfer->status);
#ifndef _WIN32
		}
#endif
	}
}

int plutosdr_wait_async(plutosdr_dev_t *dev, plutosdr_read_async_cb_t cb, void *ctx)
{
	return plutosdr_read_async(dev, cb, ctx, 0, 0);
}

static int _plutosdr_alloc_async_buffers(plutosdr_dev_t *dev)
{
    unsigned int i;

    if (!dev)
        return -1;

    if (!dev->xfer) {
        dev->xfer = malloc(dev->xfer_buf_num *
                   sizeof(struct libusb_transfer *));

        for(i = 0; i < dev->xfer_buf_num; ++i)
            dev->xfer[i] = libusb_alloc_transfer(0);
    }

    if (!dev->xfer_buf) {
        dev->xfer_buf = malloc(dev->xfer_buf_num *
                       sizeof(unsigned char *));

		for(i = 0; i < dev->xfer_buf_num; ++i)
            dev->xfer_buf[i] = malloc(dev->xfer_buf_len);
    }

    return 0;
}

static int _plutosdr_free_async_buffers(plutosdr_dev_t *dev)
{
    unsigned int i;

    if (!dev)
        return -1;

    if (dev->xfer) {
        for(i = 0; i < dev->xfer_buf_num; ++i) {
            if (dev->xfer[i]) {
                libusb_free_transfer(dev->xfer[i]);
            }
        }

        free(dev->xfer);
        dev->xfer = NULL;
    }

    if (dev->xfer_buf) {
        for(i = 0; i < dev->xfer_buf_num; ++i) {
            if (dev->xfer_buf[i])
                free(dev->xfer_buf[i]);
        }

        free(dev->xfer_buf);
        dev->xfer_buf = NULL;
    }

    return 0;
}

int plutosdr_read_async(plutosdr_dev_t *dev, plutosdr_read_async_cb_t cb, void *ctx,
			  uint32_t buf_num, uint32_t buf_len)
{
    unsigned int i;
    int r = 0;
    struct timeval tv = { 1, 0 };
    struct timeval zerotv = { 0, 0 };

	enum plutosdr_async_status next_status = plutosdr_INACTIVE;

    if (!dev)
        return -1;

	if (plutosdr_INACTIVE != dev->async_status)
		return -2;

	dev->async_status = plutosdr_RUNNING;
    dev->async_cancel = 0;

	dev->cb = cb;
	dev->cb_ctx = ctx;

	if (buf_num > 0)
		dev->xfer_buf_num = buf_num;
	else
    dev->xfer_buf_num = DEFAULT_BUF_NUMBER;
	if (buf_len > 0 && buf_len % 512 == 0) /* len must be multiple of 512 */
		dev->xfer_buf_len = buf_len;
	else
    dev->xfer_buf_len = DEFAULT_BUF_LENGTH;

    _plutosdr_alloc_async_buffers(dev);

    for(i = 0; i < dev->xfer_buf_num; ++i) {
        libusb_fill_bulk_transfer(dev->xfer[i],
					  dev->devh,
                      PLUTO_USB_READ_EPNUM,
                      dev->xfer_buf[i],
                      dev->xfer_buf_len,
                      _libusb_callback,
                      (void *)dev,
                      BULK_TIMEOUT);

        r = libusb_submit_transfer(dev->xfer[i]);
        if (r < 0) {
            fprintf(stderr, "Failed to submit transfer %i!\n", i);

			dev->async_status = plutosdr_CANCELING;
            break;
        }
    }

	while (plutosdr_INACTIVE != dev->async_status) {
        r = libusb_handle_events_timeout_completed(dev->ctx, &tv,
                               &dev->async_cancel);
        if (r < 0) {
            /*fprintf(stderr, "handle_events returned: %d\n", r);*/
            if (r == LIBUSB_ERROR_INTERRUPTED) /* stray signal */
                continue;
            break;
		}

		if (plutosdr_CANCELING == dev->async_status) {
			next_status = plutosdr_INACTIVE;

			if (!dev->xfer)
				break;

			for(i = 0; i < dev->xfer_buf_num; ++i) {
				if (!dev->xfer[i])
					continue;

				if (LIBUSB_TRANSFER_CANCELLED !=
						dev->xfer[i]->status) {
					r = libusb_cancel_transfer(dev->xfer[i]);
					/* handle events after canceling
					 * to allow transfer status to
					 * propagate */
					libusb_handle_events_timeout_completed(dev->ctx,
									       &zerotv, NULL);
					if (r < 0)
						continue;

					next_status = plutosdr_CANCELING;
				}
			}

			if (dev->dev_lost || plutosdr_INACTIVE == next_status) {
				/* handle any events that still need to
				 * be handled before exiting after we
				 * just cancelled all transfers */
				libusb_handle_events_timeout_completed(dev->ctx,
								       &zerotv, NULL);
				break;
			}
        }
    }

    _plutosdr_free_async_buffers(dev);

	dev->async_status = next_status;

    return r;
}

int plutosdr_cancel_async(plutosdr_dev_t *dev)
{
	if (!dev)
		return -1;

	/* if streaming, try to cancel gracefully */
	if (plutosdr_RUNNING == dev->async_status) {
		dev->async_status = plutosdr_CANCELING;
		dev->async_cancel = 1;
		return 0;
	}

	/* if called while in pending state, change the state forcefully */
#if 0
	if (plutosdr_INACTIVE != dev->async_status) {
		dev->async_status = plutosdr_INACTIVE;
		return 0;
	}
#endif
	return -2;
}


void plutosdr_bufstream_enable(plutosdr_dev_t *dev, uint32_t enable){
    int ret;
    dev->ctrlbuf[0] = enable ? '1' : '0';
    dev->ctrlbuf[1] = 0;
    ret = libusb_control_transfer(dev->devh, PLUTO_CTL_OUT, 0xff, ENABLE_BUFSTREAM, 0x1234, dev->ctrlbuf, PLUTO_CTLTSZ, 0);
    fprintf(stderr,"ctl %d %s req was:%s\n", ret, ret < 0 ? libusb_strerror(ret) : "fine",  dev->ctrlbuf);
}

void plutosdr_set_rfbw(plutosdr_dev_t *dev, uint32_t rfbw_hz){
    int ret;
    //ad9361_phy_store
    snprintf(dev->ctrlbuf, 64, "%s=%d", "in_voltage_rf_bandwidth", rfbw_hz);
    ret = libusb_control_transfer(dev->devh, PLUTO_CTL_OUT, 0xff, GLOBALPHYSTORE, 0x1234, dev->ctrlbuf, PLUTO_CTLTSZ, 0);
    fprintf(stderr,"ctl %d %s req was:%s\n", ret, ret < 0 ? libusb_strerror(ret) : "fine",  dev->ctrlbuf);
}

void plutosdr_set_sample_rate(plutosdr_dev_t *dev, uint32_t sampfreq_hz){
    int ret;
    //samplefreq
    snprintf(dev->ctrlbuf, 64, "%d", sampfreq_hz);
    ret = libusb_control_transfer(dev->devh, PLUTO_CTL_OUT, 0xff, SAMPFREQ, 0x1234, dev->ctrlbuf, PLUTO_CTLTSZ, 0);
    fprintf(stderr,"ctl %d %s req was:%s\n", ret, ret < 0 ? libusb_strerror(ret) : "fine",  dev->ctrlbuf);
}

void plutosdr_set_rxlo(plutosdr_dev_t *dev, uint64_t rfbw_hz){
    int ret;
    //RX_LO
    snprintf(dev->ctrlbuf, 64, "%s=%llu", "frequency", rfbw_hz);
    ret = libusb_control_transfer(dev->devh, PLUTO_CTL_OUT, 0xff, RXLO, 0x1234, dev->ctrlbuf, PLUTO_CTLTSZ, 0);
    fprintf(stderr,"ctl %d %s req was:%s\n", ret, ret < 0 ? libusb_strerror(ret) : "fine",  dev->ctrlbuf);
}

void plutosdr_set_gainctl_manual(plutosdr_dev_t *dev){
    int ret;
    //gain control mode
    snprintf(dev->ctrlbuf, 64, "%s=%s", "gain_control_mode", "manual");
    ret = libusb_control_transfer(dev->devh, PLUTO_CTL_OUT, 0xff, GAINCTLMODE, 0x1234, dev->ctrlbuf, PLUTO_CTLTSZ, 0);
    fprintf(stderr,"ctl %d %s req was:%s\n", ret, ret < 0 ? libusb_strerror(ret) : "fine",  dev->ctrlbuf);
}

void plutosdr_set_gain_mdb(plutosdr_dev_t *dev, int32_t gain_in_millib){
    int ret;
    //hw gain in milli-dB
    snprintf(dev->ctrlbuf, 64, "%d", gain_in_millib);
    ret = libusb_control_transfer(dev->devh, PLUTO_CTL_OUT, 0xff, HWGAIN, 0x1234, dev->ctrlbuf, PLUTO_CTLTSZ, 0);
    fprintf(stderr,"ctl %d %s req was:%s\n", ret, ret < 0 ? libusb_strerror(ret) : "fine",  dev->ctrlbuf);
}
