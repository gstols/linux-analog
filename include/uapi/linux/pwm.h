/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */

#ifndef _UAPI_PWM_H_
#define _UAPI_PWM_H_

#include <linux/ioctl.h>
#include <linux/types.h>

struct pwmchip_waveform {
	unsigned int hwpwm;
	unsigned int __pad; /* padding, must be zero */
	__u64 period_length;
	__u64 duty_length;
	__u64 duty_offset;
};

#define PWM_IOCTL_GET_NUM_PWMS	_IO(0x75, 0)
#define PWM_IOCTL_REQUEST	_IOW(0x75, 1, unsigned int)
#define PWM_IOCTL_FREE		_IOW(0x75, 2, unsigned int)
#define PWM_IOCTL_ROUNDWF	_IOWR(0x75, 3, struct pwmchip_waveform)
#define PWM_IOCTL_GETWF		_IOWR(0x75, 4, struct pwmchip_waveform)
#define PWM_IOCTL_SETROUNDEDWF	_IOW(0x75, 5, struct pwmchip_waveform)
#define PWM_IOCTL_SETEXACTWF	_IOW(0x75, 6, struct pwmchip_waveform)

#endif /* _UAPI_PWM_H_ */
