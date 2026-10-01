// SPDX-License-Identifier: GPL-2.0+ OR MIT
/*
 * Copyright The Asahi Linux Contributors
 */

#include <dm.h>
#include <dm/simple_bus.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <mailbox.h>
#include <keyboard.h>
#include <stdio_dev.h>
#include <asm/arch/rtkit.h>
#include <asm/io.h>
#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/input.h>
#include "apple_kbd.h"

struct apple_mtp_kbd_priv {
	struct apple_kbd_priv kbd;
	struct udevice *helper;
	void *local;
	void *rmt;

	u8 *init_data;
	u32 init_size;
	u32 fifo_size;
	int kbd_iface;		/* keyboard interface, -1 until announced */
	int backlog;		/* bytes queued at the first poll, -1 before it */
};

#define DATA_TX8		0x4
#define DATA_TX_FREE		0x14
#define DATA_RX8		0x1c
#define DATA_RX_COUNT		0x2c

struct dchid_hdr {
	u8 hdr_len;
	u8 channel;
	__le16 length;
	u8 seq;
	u8 iface;
	__le16 pad;
} __packed;

#define DCHID_CHANNEL_REPORT	0x12

struct dchid_subhdr {
	u8 flags;
	u8 unk;
	__le16 length;
	__le32 retcode;
} __packed;

#define FLAGS_GROUP		GENMASK(7, 6)
#define GROUP_INPUT		0

#define EVENT_INIT		0xf0

struct dchid_init_hdr {
	u8 type;
	u8 unk1;
	u8 unk2;
	u8 iface;
	char name[16];
	u8 more_packets;
	u8 unkpad;
} __packed;

static int dockchannel_read(struct udevice *dev, void *buf, size_t size)
{
	struct apple_mtp_kbd_priv *priv = dev_get_priv(dev);
	int ret = 0;
	u8 b;
	u8 *p = buf;

	while (size--) {
		ulong start;
		start = get_timer(0);
		while (get_timer(start) < 100) {
			if (readl(priv->local + DATA_RX_COUNT) != 0)
				break;
		}

		if (readl(priv->local + DATA_RX_COUNT) == 0) {
			return -ETIME;
		}

		b = readl(priv->local + DATA_RX8) >> 8;
		if (buf)
			*p++ = b;

		ret++;
	}

	return ret;
}

/*
 * Check a packet as Linux dockchannel-hid does: the header, body and
 * checksum words add up to 0xffffffff, the sub header is an input report
 * and its length fits in the body. Return the payload length or -1.
 */
static int dchid_input_len(struct dchid_hdr *hdr, u8 *body)
{
	struct dchid_subhdr *sub = (void *)body;
	u32 sum = get_unaligned_le32(hdr) + get_unaligned_le32((u8 *)hdr + 4);
	size_t i;

	if (hdr->length < sizeof(*sub))
		return -1;

	for (i = 0; i + 4 <= hdr->length + 4u; i += 4)
		sum += get_unaligned_le32(body + i);

	if (sum != 0xffffffff ||
	    FIELD_GET(FLAGS_GROUP, sub->flags) != GROUP_INPUT ||
	    le16_to_cpu(sub->length) > hdr->length - sizeof(*sub))
		return -1;

	return le16_to_cpu(sub->length);
}

/*
 * The comm interface announces each interface with EVENT_INIT reports
 * that name it. Remember which one is the keyboard once its last init
 * packet is in, as Linux does.
 */
static void apple_mtp_kbd_comm(struct apple_mtp_kbd_priv *priv,
			       struct dchid_hdr *hdr, u8 *body)
{
	struct dchid_init_hdr *init = (void *)(body + sizeof(struct dchid_subhdr));

	if (hdr->channel != DCHID_CHANNEL_REPORT ||
	    dchid_input_len(hdr, body) < (int)sizeof(*init) ||
	    init->type != EVENT_INIT)
		return;

	if (!strncmp(init->name, "keyboard", sizeof(init->name)) &&
	    !init->more_packets)
		priv->kbd_iface = init->iface;
}

/*
 * A keyboard report that was queued before the first poll gives the key
 * state at start, not a key press at a prompt. Take it as the initial
 * state: keys that are down in it make no key event, and their release
 * makes none either. So a stuck key never gets to stdin.
 */
static void apple_mtp_kbd_held(struct apple_mtp_kbd_priv *priv, u8 *data)
{
	struct apple_kbd_report *r = (void *)data;

	if (r->reportid != KBD_REPORTID)
		return;

	priv->kbd.old = *r;
	if (r->modifiers || memchr_inv(r->keycode, 0, sizeof(r->keycode)))
		printf("mtp: keys down at start, ignored: %02x %02x %02x %02x %02x %02x %02x\n",
		       r->modifiers, r->keycode[0], r->keycode[1], r->keycode[2],
		       r->keycode[3], r->keycode[4], r->keycode[5]);
}

static int apple_mtp_kbd_check(struct input_config *input)
{
	struct udevice *dev = input->dev;
	struct apple_mtp_kbd_priv *priv = dev_get_priv(dev);
	struct dchid_hdr hdr;
	bool startup;
	int ret;

	/* Poll for syslogs if RTKit is up */
	if (priv->helper)
		apple_rtkit_helper_poll(priv->helper, 0);

	u32 pending = readl(priv->local + DATA_RX_COUNT);
	if (priv->backlog < 0)
		priv->backlog = pending;
	if (pending < 8)
		return 0;

	ret = dockchannel_read(dev, &hdr, sizeof(hdr));
	if (ret < 0) {
		dev_err(dev, "failed to read packet header\n");
		return ret;
	}

	if (hdr.hdr_len != sizeof(hdr)) {
		/* Not at a packet start: drop what is queued, then resync */
		dev_err(dev, "bad header length %d\n", hdr.hdr_len);
		dockchannel_read(dev, NULL, pending - sizeof(hdr));
		priv->backlog = 0;
		return 0;
	}

	/* Packets that were queued at the first poll were sent at start */
	startup = priv->backlog > 0;
	priv->backlog -= min_t(int, priv->backlog, sizeof(hdr) + hdr.length + 4);

	/* Save comm init messages for the next stage */
	if (hdr.iface == 0) {
		int space = priv->fifo_size - priv->init_size;
		int need = hdr.length + sizeof(hdr) + 4;

		if (space < need) {
			dev_err(dev, "out of buf space (%d > %d)\n",
				need, space);
			ret = dockchannel_read(dev, NULL, hdr.length + 4);
			if (ret < 0)
				return ret;
		} else {
			u8 *p = &priv->init_data[priv->init_size];

			memcpy(p, &hdr, sizeof(hdr));
			ret = dockchannel_read(dev, p + sizeof(hdr), hdr.length + 4);
			if (ret < 0)
				return ret;
			priv->init_size += need;
			apple_mtp_kbd_comm(priv, &hdr, p + sizeof(hdr));
		}
	} else if (hdr.channel == DCHID_CHANNEL_REPORT &&
		   hdr.iface == priv->kbd_iface && hdr.length == 0x14) {
		u8 buf[0x18];
		int len;

		ret = dockchannel_read(dev, buf, 0x18);
		if (ret < 0)
			return ret;

		if (!priv->helper)
			return 1; /* Ignore if shutting down */

		len = dchid_input_len(&hdr, buf);
		if (len < (int)sizeof(struct apple_kbd_report))
			return 0;

		if (startup) {
			apple_mtp_kbd_held(priv, buf + sizeof(struct dchid_subhdr));
			return 0;
		}

		return apple_kbd_handle_report(input, &priv->kbd,
					       buf + sizeof(struct dchid_subhdr),
					       len);
	} else if (hdr.channel == DCHID_CHANNEL_REPORT) {
		/* Other interfaces, other reports, or keyboard not announced */
		ret = dockchannel_read(dev, NULL, hdr.length + 4);
		if (ret < 0)
			return ret;
	} else {
		printk("mtp: unknown message ch=%02x l=%04x if=%02x\n",
		       hdr.channel, hdr.length, hdr.iface);
		ret = dockchannel_read(dev, NULL, hdr.length + 4);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int get_rtkit_helper(struct udevice *dev)
{
	struct apple_mtp_kbd_priv *priv = dev_get_priv(dev);
	int ret;
	u32 phandle;
	ofnode of_mtp;

	ret = dev_read_u32(dev, "apple,helper-cpu", &phandle);
	if (ret < 0)
		return ret;

	of_mtp = ofnode_get_by_phandle(phandle);
	ret = uclass_get_device_by_ofnode(UCLASS_MISC, of_mtp, &priv->helper);
	if (ret < 0)
		return ret;

	return 0;
}

static int apple_mtp_kbd_probe(struct udevice *dev)
{
	struct apple_mtp_kbd_priv *priv = dev_get_priv(dev);
	struct keyboard_priv *uc_priv = dev_get_uclass_priv(dev);
	struct stdio_dev *sdev = &uc_priv->sdev;
	struct input_config *input = &uc_priv->input;
	int ret;
	fdt_addr_t reg;

	printf("mtp_kbd_probe\n");

	reg = dev_read_addr_name(dev, "data");
	if (reg == FDT_ADDR_T_NONE) {
		dev_err(dev, "no reg property for local FIFO data registers\n");
		return -EINVAL;
	}
	priv->local = (void *)reg;

	reg = dev_read_addr_name(dev, "rmt-data");
	if (reg == FDT_ADDR_T_NONE) {
		dev_err(dev, "no reg property for remote FIFO data registers\n");
		return -EINVAL;
	}
	priv->rmt = (void *)reg;

	ret = dev_read_u32(dev, "apple,fifo-size", &priv->fifo_size);
	if (ret < 0 || !priv->fifo_size) {
		dev_err(dev, "no apple,fifo-size property\n");
		return ret;
	}

	ret = get_rtkit_helper(dev);
	if (ret < 0) {
		dev_err(dev, "Failed to get helper device (%d)\n", ret);
		return ret;
	}

	priv->init_data = malloc(priv->fifo_size);
	if (!priv->init_data)
		return -ENOMEM;

	priv->kbd_iface = -1;
	priv->backlog = -1;

	input->dev = dev;
	input->read_keys = apple_mtp_kbd_check;
	input_add_tables(input, false);
	strcpy(sdev->name, "mtpkbd");

	return input_stdio_register(sdev);
}

static int apple_mtp_kbd_remove(struct udevice *dev)
{
	struct apple_mtp_kbd_priv *priv = dev_get_priv(dev);
	struct keyboard_priv *uc_priv = dev_get_uclass_priv(dev);
	struct input_config *input = &uc_priv->input;
	int i;

	if (priv->helper) {
		device_remove(priv->helper, DM_REMOVE_NORMAL);
		priv->helper = NULL;
	}

	/* Drain the FIFO */
	while (readl(priv->local + DATA_RX_COUNT)) {
		if (apple_mtp_kbd_check(input) < 0) {
			dev_err(dev, "Failed to drain FIFO\n");
			break;
		}
	}

	/* Stuff init messages back into FIFO for the next stage to find */
	for (i = 0; i < priv->init_size; i++)
		writel(priv->init_data[i], priv->rmt + DATA_TX8);

	return 0;
}

static const struct keyboard_ops apple_mtp_kbd_ops = {
};

static const struct udevice_id apple_mtp_kbd_of_match[] = {
	{ .compatible = "apple,dockchannel-hid" },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(apple_mtp_kbd) = {
	.name = "apple_mtp_kbd",
	.id = UCLASS_KEYBOARD,
	.of_match = apple_mtp_kbd_of_match,
	.probe = apple_mtp_kbd_probe,
	.remove = apple_mtp_kbd_remove,
	.priv_auto = sizeof(struct apple_mtp_kbd_priv),
	.ops = &apple_mtp_kbd_ops,
	.flags = DM_FLAG_OS_PREPARE,
};

/* Treat dockchannel as a simple-bus, since we don't use the IRQ stuff */

static const struct udevice_id dockchannel_bus_ids[] = {
	{ .compatible = "apple,dockchannel" },
	{ }
};

U_BOOT_DRIVER(dockchannel) = {
	.name	= "dockchannel",
	.id	= UCLASS_SIMPLE_BUS,
	.of_match = of_match_ptr(dockchannel_bus_ids),
};
