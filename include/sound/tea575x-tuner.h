#ifndef __SOUND_TEA575X_TUNER_H
#define __SOUND_TEA575X_TUNER_H

/*
 *   ALSA driver for TEA5757/5759 Philips AM/FM tuner chips
 *
 *	Copyright (c) 2004 Jaroslav Kysela <perex@perex.cz>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include "compat.h"
#include <linux/videodev.h>
#include <media/v4l2-dev.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,17)
typedef struct snd_tea575x tea575x_t;
#else
struct snd_tea575x;
#endif

struct snd_tea575x_ops {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,17)
	void (*write)(tea575x_t *tea, unsigned int val);
	unsigned int (*read)(tea575x_t *tea);
#else
	void (*write)(struct snd_tea575x *tea, unsigned int val);
	unsigned int (*read)(struct snd_tea575x *tea);
	void (*mute)(struct snd_tea575x *tea, unsigned int mute);
#endif
};

struct snd_tea575x {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,17)
	snd_card_t *card;
#else
	struct snd_card *card;
#endif
	struct video_device vd;		/* video device */
	struct v4l2_file_operations fops;
	int dev_nr;			/* requested device number + 1 */
	int vd_registered;		/* video device is registered */
	int tea5759;			/* 5759 chip is present */
	unsigned int freq_fixup;	/* crystal onboard */
	unsigned int val;		/* hw value */
	unsigned long freq;		/* frequency */
	unsigned long in_use;		/* set if the device is in use */
	struct snd_tea575x_ops *ops;
	void *private_data;
};

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,17)
void snd_tea575x_init(tea575x_t *tea);
void snd_tea575x_exit(tea575x_t *tea);
#else
void snd_tea575x_init(struct snd_tea575x *tea);
void snd_tea575x_exit(struct snd_tea575x *tea);
#endif

#endif /* __SOUND_TEA575X_TUNER_H */
