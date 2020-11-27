/*
 *   Apple "Magic" Wireless Mouse driver
 *
 *   Copyright (c) 2010 Michael Poole <mdpoole@troilus.org>
 *   Copyright (c) 2010 Chase Douglas <chase.douglas@canonical.com>
 *   Copyright (c) 2018 Rohit Pidaparthi <rohitkernel@gmail.com>
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb/input.h>

#include "hid-ids.h"

static unsigned int scroll_speed = 5;
static int param_set_scroll_speed(const char *val,
				  const struct kernel_param *kp) {
	unsigned long speed;
	if (!val || kstrtoul(val, 0, &speed) || speed > 63)
		return -EINVAL;
	scroll_speed = speed;
	return 0;
}
module_param_call(scroll_speed, param_set_scroll_speed, param_get_uint, &scroll_speed, 0644);
MODULE_PARM_DESC(scroll_speed, "Scroll speed, value from 0 (slow) to 63 (fast)");

static bool scroll_acceleration = false;
module_param(scroll_acceleration, bool, 0644);
MODULE_PARM_DESC(scroll_acceleration, "Accelerate sequential scroll events");

#define TRACKPAD_REPORT_ID 0x28
#define TRACKPAD2_USB_REPORT_ID 0x02
#define TRACKPAD2_BT_REPORT_ID 0x31
#define MOUSE_REPORT_ID    0x29
#define MOUSE2_REPORT_ID   0x12
#define MOUSE2_REQUEST_REPORT_ID   0xa1
#define DOUBLE_REPORT_ID   0xf7
/* These definitions are not precise, but they're close enough.  (Bits
 * 0x03 seem to indicate the aspect ratio of the touch, bits 0x70 seem
 * to be some kind of bit mask -- 0x20 may be a near-field reading,
 * and 0x40 is actual contact, and 0x10 may be a start/stop or change
 * indication.)
 */
#define TOUCH_STATE_MASK  0xf0
#define TOUCH_STATE_NONE  0x00
#define TOUCH_STATE_START 0x30
#define TOUCH_STATE_DRAG  0x40

#define SCROLL_ACCEL_DEFAULT 1

/* Touch surface information. Dimension is in hundredths of a mm, min and max
 * are in units. */
#define MOUSE_DIMENSION_X (float)9056
#define MOUSE_MIN_X -1100
#define MOUSE_MAX_X 1258
#define MOUSE_RES_X ((MOUSE_MAX_X - MOUSE_MIN_X) / (MOUSE_DIMENSION_X / 100))
#define MOUSE_DIMENSION_Y (float)5152
#define MOUSE_MIN_Y -1589
#define MOUSE_MAX_Y 2047
#define MOUSE_RES_Y ((MOUSE_MAX_Y - MOUSE_MIN_Y) / (MOUSE_DIMENSION_Y / 100))

#define TRACKPAD_DIMENSION_X (float)13000
#define TRACKPAD_MIN_X -2909
#define TRACKPAD_MAX_X 3167
#define TRACKPAD_RES_X \
	((TRACKPAD_MAX_X - TRACKPAD_MIN_X) / (TRACKPAD_DIMENSION_X / 100))
#define TRACKPAD_DIMENSION_Y (float)11000
#define TRACKPAD_MIN_Y -2456
#define TRACKPAD_MAX_Y 2565
#define TRACKPAD_RES_Y \
	((TRACKPAD_MAX_Y - TRACKPAD_MIN_Y) / (TRACKPAD_DIMENSION_Y / 100))

#define TRACKPAD2_DIMENSION_X (float)16000
#define TRACKPAD2_MIN_X -3678
#define TRACKPAD2_MAX_X 3934
#define TRACKPAD2_RES_X \
	((TRACKPAD2_MAX_X - TRACKPAD2_MIN_X) / (TRACKPAD2_DIMENSION_X / 100))
#define TRACKPAD2_DIMENSION_Y (float)11490
#define TRACKPAD2_MIN_Y -2478
#define TRACKPAD2_MAX_Y 2587
#define TRACKPAD2_RES_Y \
	((TRACKPAD2_MAX_Y - TRACKPAD2_MIN_Y) / (TRACKPAD2_DIMENSION_Y / 100))

#define MAX_TOUCHES		16

/**
 * struct magicmouse_sc - Tracks Magic Mouse-specific data.
 * @input: Input device through which we report events.
 * @quirks: Currently unused.
 * @ntouches: Number of touches in most recent touch report.
 * @scroll_accel: Number of consecutive scroll motions.
 * @scroll_jiffies: Time of last scroll motion.
 * @touches: Most recent data for a touch, indexed by tracking ID.
 * @tracking_ids: Mapping of current touch input data to @touches.
 */
struct magicmouse_sc {
	struct input_dev *input;
	unsigned long quirks;

	int ntouches;
	int scroll_accel;
	unsigned long scroll_jiffies;

	struct {
		short x;
		short y;
		short scroll_x;
		short scroll_y;
		u8 size;
	} touches[MAX_TOUCHES];
	int tracking_ids[MAX_TOUCHES];
};

static void magicmouse_emit_touch(struct magicmouse_sc *msc, int raw_id,
		u8 *tdata, int npoints)
{
	struct input_dev *input = msc->input;
	int id, x, y, state;

	if (input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE ||
		input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE2) {
		/* tdata is 8 bytes per finger detected.
		 * tdata[0] (lsb of x) and least sig 4bits of tdata[1] (msb of x)
		 *          are x position of touch on touch surface.
		 * tdata[1] most sig 4bits (lsb of y) and and tdata[2] (msb of y)
		 *          are y position of touch on touch surface.
		 * tdata[1] bits look like [y y y y x x x x]
		 * tdata[3] touch major axis of ellipse of finger detected
		 * tdata[4] touch minor axis of ellipse of finger detected
		 * tdata[5] contains 6bits of size info (lsb) and the two msb of tdata[5]
		 *          are the lsb of id: [id id size size size size size size]
		 * tdata[6] 2 lsb bits of tdata[6] are the msb of id and 6msb of tdata[6]
		 *          are the orientation of the touch. [o o o o o o id id]
		 * tdata[7] 4 msb are state. 4lsb are unknown.
		 *
		 * [ x x x x x x x x ]
		 * [ y y y y x x x x ]
		 * [ y y y y y y y y ]
		 * [touch major      ]
		 * [touch minor      ]
		 * [id id s s s s s s]
		 * [o o o o o o id id]
		 * [s s s s | unknown]
		 */
		id = (tdata[6] << 2 | tdata[5] >> 6) & 0xf;
		x = (tdata[1] << 28 | tdata[0] << 20) >> 20;
		y = -((tdata[2] << 24 | tdata[1] << 16) >> 20);
		state = tdata[7] & TOUCH_STATE_MASK;

		/* If requested, emulate a scroll wheel by detecting small
		* vertical touch motions.
		*/
		{
			unsigned long now = jiffies;
			int step_x = msc->touches[id].scroll_x - x;
			int step_y = msc->touches[id].scroll_y - y;

			/* Calculate and apply the scroll motion. */
			switch (state) {
			case TOUCH_STATE_START:
				msc->touches[id].scroll_x = x;
				msc->touches[id].scroll_y = y;

				/* Reset acceleration after half a second. */
				if (scroll_acceleration && time_before(now,
							msc->scroll_jiffies + HZ / 2))
					msc->scroll_accel = max_t(int,
							msc->scroll_accel - 1, 1);
				else
					msc->scroll_accel = SCROLL_ACCEL_DEFAULT;

				break;
			case TOUCH_STATE_DRAG:
				step_x /= (64 - (int)scroll_speed) * msc->scroll_accel;
				if (step_x != 0) {
					msc->touches[id].scroll_x -= step_x *
						(64 - scroll_speed) * msc->scroll_accel;
					msc->scroll_jiffies = now;
					input_report_rel(input, REL_HWHEEL, -step_x);
				}

				step_y /= (64 - (int)scroll_speed) * msc->scroll_accel;
				if (step_y != 0) {
					msc->touches[id].scroll_y -= step_y *
						(64 - scroll_speed) * msc->scroll_accel;
					msc->scroll_jiffies = now;
					input_report_rel(input, REL_WHEEL, step_y);
				}
				break;
			}
		}
	}
}

static int magicmouse_raw_event(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size)
{
	struct magicmouse_sc *msc = hid_get_drvdata(hdev);
	struct input_dev *input = msc->input;
	int x = 0, y = 0, ii, clicks = 0, npoints;

	switch (data[0]) {
	case MOUSE_REPORT_ID:
		/* Expect six bytes of prefix, and N*8 bytes of touch data. */
		if (size < 6 || ((size - 6) % 8) != 0)
			return 0;
		npoints = (size - 6) / 8;
		if (npoints > 15) {
			hid_warn(hdev, "invalid size value (%d) for MOUSE_REPORT_ID\n",
					size);
			return 0;
		}
		msc->ntouches = 0;
		for (ii = 0; ii < npoints; ii++)
			magicmouse_emit_touch(msc, ii, data + ii * 8 + 6, npoints);

		/* When emulating three-button mode, it is important
		 * to have the current touch information before
		 * generating a click event.
		 */
		x = (int)(((data[3] & 0x0c) << 28) | (data[1] << 22)) >> 22;
		y = (int)(((data[3] & 0x30) << 26) | (data[2] << 22)) >> 22;
		clicks = data[3];

		/* The following bits provide a device specific timestamp. They
		 * are unused here.
		 *
		 * ts = data[3] >> 6 | data[4] << 2 | data[5] << 10;
		 */
		break;
	case MOUSE2_REPORT_ID:
		/* The data layout for magic mouse 2 is:
		 * 14 bytes of prefix
		 * data[0] is the device report ID
		 * data[1] is the mouse click events. Value of 1 is left, 2 is right.
		 * data[2] (contains lsb) and data[3] (msb) are the x movement
		 *         of the mouse 16bit representation.
		 * data[4] (contains msb) and data[5] (msb) are the y movement
		 *         of the mouse 16bit representation.
		 * data[6] data[13] are unknown so far. Need to decode this still
		 *
		 * data[14] onwards represent touch data on top of the mouse surface
		 *          touchpad. There are 8 bytes per finger. e.g:
		 * data[14]-data[21] will be the first finger detected.
		 * data[22]-data[29] will be finger 2 etc.
		 * these sets of 8 bytes are passed in as tdata to
		 * magicmouse_emit_touch()
		 *
		 * npoints is the number of fingers detected.
		 * size is minimum 14 but could be any multpiple of 14+ii*8 based on
		 * how many fingers are detected. e.g for 1 finger, size=22 for
		 * 2 fingers, size=30 and so on.
		 */
		if (size > 14 && ((size - 14) % 8) != 0)
            return 0;
        npoints = (size - 14) / 8;
        if (npoints > 15) {
            hid_warn(hdev, "invalid size value (%d) for MOUSE_REPORT_ID\n",
                     size);
            return 0;
        }
        msc->ntouches = 0;
		// print the values of the first 14 bytes of data and number of points and size.
		// printk("The contents of npoints are: %i\n", npoints);
		// printk("Size is: %i\n", size);
		// int jj = 0;
		// for (jj=0; jj < 15; jj++){
		// 	int d = data[jj];
		// 	printk("data %i is: %i\n", jj, d);
		// }
        for (ii = 0; ii < npoints; ii++)
            magicmouse_emit_touch(msc, ii, data + ii * 8 + 14, npoints);

        /* When emulating three-button mode, it is important
         * to have the current touch information before
         * generating a click event.
         */
        x = (int)((data[3] << 24) | (data[2] << 16)) >> 16;
        y = (int)((data[5] << 24) | (data[4] << 16)) >> 16;
        clicks = data[1];
        break;
	case DOUBLE_REPORT_ID:
		/* Sometimes the trackpad sends two touch reports in one
		 * packet.
		 */
		magicmouse_raw_event(hdev, report, data + 2, data[1]);
		magicmouse_raw_event(hdev, report, data + 2 + data[1],
			size - 2 - data[1]);
		break;
	default:
		return 0;
	}

	input_sync(input);
	return 1;
}

static int magicmouse_setup_input(struct input_dev *input, struct hid_device *hdev)
{
	__set_bit(EV_KEY, input->evbit);

	if (input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE ||
		input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE2) {
		__set_bit(EV_REL, input->evbit);
		__set_bit(REL_WHEEL, input->relbit);
		__set_bit(REL_HWHEEL, input->relbit);
	}

	input_set_events_per_packet(input, 60);

	return 0;
}

static int magicmouse_input_mapping(struct hid_device *hdev,
		struct hid_input *hi, struct hid_field *field,
		struct hid_usage *usage, unsigned long **bit, int *max)
{
	struct magicmouse_sc *msc = hid_get_drvdata(hdev);

	if (!msc->input)
		msc->input = hi->input;

	/* Magic Trackpad does not give relative data after switching to MT */
	if ((hi->input->id.product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD ||
	     hi->input->id.product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD2) &&
	    field->flags & HID_MAIN_ITEM_RELATIVE)
		return -1;

	return 0;
}

static int magicmouse_input_configured(struct hid_device *hdev,
		struct hid_input *hi)

{
	struct magicmouse_sc *msc = hid_get_drvdata(hdev);
	int ret;

	ret = magicmouse_setup_input(msc->input, hdev);
	if (ret) {
		hid_err(hdev, "magicmouse setup input failed (%d)\n", ret);
		/* clean msc->input to notify probe() of the failure */
		msc->input = NULL;
		return ret;
	}

	return 0;
}


static int magicmouse_probe(struct hid_device *hdev,
	const struct hid_device_id *id)
{
	__u8 feature_mt_mouse2_bt[] = { 0xF1, 0x02, 0x01 };
	__u8 feature_mt[] = { 0xD7, 0x01 };
	__u8 *feature;
	struct magicmouse_sc *msc;
	struct hid_report *report;
	int ret;
	int feature_size;
	struct usb_interface *intf;

	if (id->vendor == USB_VENDOR_ID_APPLE &&
	    id->product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD2) {
		intf = to_usb_interface(hdev->dev.parent);
		if (intf->cur_altsetting->desc.bInterfaceNumber != 1)
			return 0;
	}

	msc = devm_kzalloc(&hdev->dev, sizeof(*msc), GFP_KERNEL);
	if (msc == NULL) {
		hid_err(hdev, "can't alloc magicmouse descriptor\n");
		return -ENOMEM;
	}

	msc->scroll_accel = SCROLL_ACCEL_DEFAULT;

	msc->quirks = id->driver_data;
	hid_set_drvdata(hdev, msc);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "magicmouse hid parse failed\n");
		return ret;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		hid_err(hdev, "magicmouse hw start failed\n");
		return ret;
	}

	if (!msc->input) {
		hid_err(hdev, "magicmouse input not registered\n");
		ret = -ENOMEM;
		goto err_stop_hw;
	}

	if (id->product == USB_DEVICE_ID_APPLE_MAGICMOUSE) {
		report = hid_register_report(hdev, HID_INPUT_REPORT,
			MOUSE_REPORT_ID, 0);
	} else if (id->product == USB_DEVICE_ID_APPLE_MAGICMOUSE2) {
		report = hid_register_report(hdev, HID_INPUT_REPORT,
			MOUSE2_REQUEST_REPORT_ID, 0);
	}

	if (!report) {
		hid_err(hdev, "unable to register touch report\n");
		ret = -ENOMEM;
		goto err_stop_hw;
	}
	report->size = 6;


	/*
	 * Some devices repond with 'invalid report id' when feature
	 * report switching it into multitouch mode is sent to it.
	 *
	 * This results in -EIO from the _raw low-level transport callback,
	 * but there seems to be no other way of switching the mode.
	 * Thus the super-ugly hacky success check below.
	 */
	if (id->product == USB_DEVICE_ID_APPLE_MAGICMOUSE ) {
		feature_size = sizeof(feature_mt);
		feature = kmemdup(feature_mt, feature_size, GFP_KERNEL);
	}
	else if (id->product == USB_DEVICE_ID_APPLE_MAGICMOUSE2){
		feature_size = sizeof(feature_mt_mouse2_bt);
		feature = kmemdup(feature_mt_mouse2_bt, feature_size, GFP_KERNEL);
	}
	if (!feature) {
		ret = -ENOMEM;
		goto err_stop_hw;
	}
	ret = hid_hw_raw_request(hdev, feature[0], feature, feature_size,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	kfree(feature);
	if (ret != -EIO && ret != feature_size) {
		hid_err(hdev, "unable to request touch data (%d)\n", ret);
		goto err_stop_hw;
	}

	return 0;
err_stop_hw:
	hid_hw_stop(hdev);
	return ret;
}

static const struct hid_device_id magic_mice[] = {
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_APPLE,
		USB_DEVICE_ID_APPLE_MAGICMOUSE), .driver_data = 0 },
	{ HID_BLUETOOTH_DEVICE(BT_VENDOR_ID_APPLE,
		USB_DEVICE_ID_APPLE_MAGICMOUSE2), .driver_data = 0 },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_APPLE,
		USB_DEVICE_ID_APPLE_MAGICTRACKPAD), .driver_data = 0 },
	{ HID_BLUETOOTH_DEVICE(BT_VENDOR_ID_APPLE,
		USB_DEVICE_ID_APPLE_MAGICTRACKPAD2), .driver_data = 0 },
	{ HID_USB_DEVICE(USB_VENDOR_ID_APPLE,
		USB_DEVICE_ID_APPLE_MAGICTRACKPAD2), .driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(hid, magic_mice);

static struct hid_driver magicmouse_driver = {
	.name = "magicmouse",
	.id_table = magic_mice,
	.probe = magicmouse_probe,
	.raw_event = magicmouse_raw_event,
	.input_mapping = magicmouse_input_mapping,
	.input_configured = magicmouse_input_configured,
};
module_hid_driver(magicmouse_driver);

MODULE_LICENSE("GPL");
