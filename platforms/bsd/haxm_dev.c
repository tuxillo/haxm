/*
 * Copyright (c) 2011-2018 The DragonFly Project.  All rights reserved.
 * Copyright (c) 2018 Kryptos Logic
 *
 * This code is derived from software contributed to The DragonFly Project
 * by Antonio Huete Jimenez <tuxillo@quantumachine.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of The DragonFly Project nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific, prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/device.h>
#include <sys/errno.h>
#include <sys/module.h>

#include "../../core/include/hax_core_interface.h"

#define HAX_VM_DEVFS_FMT	"hax_vm/vm%02d"
#define HAX_VCPU_DEVFS_FMT	"hax_vm%02d/vcpu%02d"
#define HAX_DEVICE_NAME 	"HAX"

MALLOC_DEFINE(M_HAXM, "haxm", "haxm driver structures");

static d_ioctl_t	hax_ioctl;

static d_open_t		hax_vm_open;
static d_close_t	hax_vm_close;
static d_ioctl_t	hax_vm_ioctl;

static d_open_t		hax_vcpu_open;
static d_close_t	hax_vcpu_close;
static d_ioctl_t	hax_vcpu_ioctl;

static cdev_t hax_dev;

struct hax_vm_softc {
	struct	vm_t *sc_vm;	/* VM implementation struct */
	char *sc_devname;	/* device name */
	int sc_id;		/* vm id */
	cdev_t sc_dev;		/* device associated */
};

struct hax_vcpu_softc {
	struct hax_vm_softc *sc_vm;	/* VM to which vcpu belongs to */
	struct vcpu_t *sc_vcpu;	/* VCPU implementation info */
	char *sc_devname;		/* device name */
	cdev_t sc_dev;			/* device associated */
	int sc_id;			/* vcpu id */
};

static struct dev_ops hax_ops = {
	{ "haxm", 0, D_MEM },
	.d_ioctl =	hax_ioctl
};

static struct dev_ops hax_vm_ops = {
	{ "haxm", 0, D_MEM },
	.d_open =	hax_vm_open,
	.d_close =	hax_vm_close,
	.d_ioctl =	hax_vm_ioctl
};

static struct dev_ops hax_vcpu_ops = {
	{ "haxm", 0, D_MEM },
	.d_open =	hax_vcpu_open,
	.d_close =	hax_vcpu_close,
	.d_ioctl =	hax_vcpu_ioctl
};

/*
 * HAXVM API functions
 */
int hax_vcpu_create_host(struct vcpu_t *cvcpu, void *vm_host, int vm_id,
                         int vcpu_id)
{
	return EOPNOTSUPP;
}

int hax_vcpu_destroy_host(struct vcpu_t *cvcpu, void *vcpu_host)
{
	return EOPNOTSUPP;
}

int hax_vm_create_host(struct vm_t *cvm, int vm_id)
{
	return EOPNOTSUPP;
}

/* When coming here, all vcpus should have been destroyed already. */
int hax_vm_destroy_host(struct vm_t *cvm, void *vm_host)
{
	return EOPNOTSUPP;
}

/* XXX: What about BSD?
 * No corresponding function in Linux side, it can be cleaned later.
 */
int hax_destroy_host_interface(void)
{
    return 0;
}

/*
 * Driver file operations
 */
static int
hax_ioctl(struct dev_ioctl_args *ap)
{
	int ret = 0;

	switch (ap->a_cmd) {
	case HAX_IOCTL_VERSION: {
		struct hax_module_version version = {};
		version.cur_version = HAX_CUR_VERSION;
		version.compat_version = HAX_COMPAT_VERSION;
		if (copyout(&version, ap->a_data, sizeof(version)))
			return EFAULT;
		break;
	}
	case HAX_IOCTL_CAPABILITY: {
		struct hax_capabilityinfo capab = {};
		hax_get_capability(&capab, sizeof(capab), NULL);
		if (copyout(&capab, ap->a_data, sizeof(capab)))
			return EFAULT;
		break;
	}
	case HAX_IOCTL_SET_MEMLIMIT: {
		struct hax_set_memlimit memlimit = {};
		if (copyin(ap->a_data, &memlimit, sizeof(memlimit)))
			return EFAULT;
		ret = hax_set_memlimit(&memlimit, sizeof(memlimit), NULL);
		break;
	}
	case HAX_IOCTL_CREATE_VM: {
		int vm_id;
		struct vm_t *vm;

		vm = hax_create_vm(&vm_id);
		if (!vm) {
			hax_log_level(HAX_LOGE, "Failed to create the HAX VM\n");
			ret = ENOMEM;
			break;
		}

		if (copyout(&vm_id, ap->a_data, sizeof(vm_id)))
			return EFAULT;
		break;
	}
	default:
        break;
	}
    return ret;
}

/*
 * VM file operations
 */
static int
hax_vm_open(struct dev_open_args *ap)
{
	return EOPNOTSUPP;
}

static int
hax_vm_close(struct dev_close_args *ap)
{
	return EOPNOTSUPP;
}

static int
hax_vm_ioctl(struct dev_ioctl_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct vm_t *vm;
	struct hax_vm_softc *sc;
	int ret;

	sc = dev->si_drv1;
	assert(sc != NULL);

	vm = hax_get_vm(sc->sc_id, 1);
	if (!vm)
		return ENODEV;

	switch (ap->a_cmd) {
	case HAX_VM_IOCTL_VCPU_CREATE:
	case HAX_VM_IOCTL_VCPU_CREATE_ORIG: {
		uint32_t vcpu_id, vm_id;
		struct vcpu_t *vcpu;

		vm_id = sc->sc_id;
		if (copyin(ap->a_data, &vcpu_id, sizeof(vcpu_id))) {
			ret = EFAULT;
			break;
		}
		vcpu = vcpu_create(vm, sc, vcpu_id);
		if (!vcpu) {
			hax_error("Failed to create vcpu %x on vm %x\n", vcpu_id, vm_id);
			ret = EINVAL;
		}
		break;
	}
	case HAX_VM_IOCTL_ALLOC_RAM: {
		struct hax_alloc_ram_info info;
		if (copyin(ap->a_data, &info, sizeof(info))) {
			ret = EFAULT;
			break;
		}
		hax_info("IOCTL_ALLOC_RAM: vm_id=%d, va=0x%llx, size=0x%x, pad=0x%x\n",
		    sc->sc_id, info.va, info.size, info.pad);
		ret = hax_vm_add_ramblock(vm, info.va, info.size);
		break;
	}
	case HAX_VM_IOCTL_ADD_RAMBLOCK: {
		struct hax_ramblock_info info;
		if (copyin(ap->a_data, &info, sizeof(info))) {
			ret = EFAULT;
			break;
		}
		if (info.reserved) {
			hax_error("IOCTL_ADD_RAMBLOCK: vm_id=%d, reserved=0x%llx\n",
			    sc->sc_id, info.reserved);
			ret = EINVAL;
			break;
		}
		hax_info("IOCTL_ADD_RAMBLOCK: vm_id=%d, start_va=0x%llx, size=0x%llx\n",
		    sc->sc_id, info.start_va, info.size);
		ret = hax_vm_add_ramblock(vm, info.start_va, info.size);
		break;
	}
	case HAX_VM_IOCTL_SET_RAM: {
		struct hax_set_ram_info info;
		if (copyin(ap->a_data, &info, sizeof(info))) {
			ret = EFAULT;
			break;
		}
		ret = hax_vm_set_ram(vm, &info);
		break;
	}
#ifdef CONFIG_HAX_EPT2
	case HAX_VM_IOCTL_SET_RAM2: {
		struct hax_set_ram_info2 info;
		if (copyin(ap->a_data, &info, sizeof(info))) {
			ret = EFAULT;
			break;
		}
		if (info.reserved1 || info.reserved2) {
			hax_error("IOCTL_SET_RAM2: vm_id=%d, reserved1=0x%x reserved2=0x%llx\n",
			    sc->sc_id, info.reserved1, info.reserved2);
			ret = EINVAL;
			break;
		}
		ret = hax_vm_set_ram2(vm, &info);
		break;
	}
	case HAX_VM_IOCTL_PROTECT_RAM: {
		struct hax_protect_ram_info info;
		if (copyin(ap->a_data, &info, sizeof(info))) {
			ret = EFAULT;
			break;
		}
		if (info.reserved) {
			hax_error("IOCTL_PROTECT_RAM: vm_id=%d, reserved=0x%x\n",
			    sc->sc_id, info.reserved);
			ret = EINVAL;
			break;
		}
		ret = hax_vm_protect_ram(vm, &info);
		break;
	}
#endif
	case HAX_VM_IOCTL_NOTIFY_QEMU_VERSION: {
		struct hax_qemu_version info;
		if (copyin(ap->a_data, &info, sizeof(info))) {
			ret = EFAULT;
			break;
		}

		// TODO: Print information about the process that sent the ioctl.
		ret = hax_vm_set_qemuversion(vm, &info);
		break;
	}
	default:
		// TODO: Print information about the process that sent the ioctl.
		hax_error("Unknown VM IOCTL 0x%lx\n", ap->a_cmd);
		break;
	}
	hax_put_vm(vm);

	return 0;
}

/*
 * VCPU file operations
 */
static int
hax_vcpu_open(struct dev_open_args *ap)
{
	return EOPNOTSUPP;
}

static int
hax_vcpu_close(struct dev_close_args *ap)
{
	return EOPNOTSUPP;
}

static int
hax_vcpu_ioctl(struct dev_ioctl_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct vcpu_t *vcpu;
	struct hax_vcpu_softc *sc;
	int ret = 0;

	sc = dev->si_drv1;
	assert(sc != NULL);

	vcpu = hax_get_vcpu(sc->sc_vm->sc_id, sc->sc_id, dev);
	if (!vcpu)
		return ENODEV;

	switch(ap->a_cmd) {
	case HAX_VCPU_IOCTL_RUN:
		ret = vcpu_execute(vcpu);
		break;
	case HAX_VCPU_IOCTL_SETUP_TUNNEL: {
		struct hax_tunnel_info info;
		ret = hax_vcpu_setup_hax_tunnel(vcpu, &info);
		if (copyin(ap->a_data, &info, sizeof(info))) {
			ret = EFAULT;
			break;
		}
		break;
	}
	case HAX_VCPU_IOCTL_SET_MSRS: {
		struct hax_msr_data msrs;
		struct vmx_msr *msr;
		int i, fail;

		if (copyin(ap->a_data, &msrs, sizeof(msrs))) {
			ret = EFAULT;
			break;
		}

		msr = msrs.entries;
		/* nr_msr needs to be verified */
		if (msrs.nr_msr >= 0x20) {
			hax_error("MSRS invalid!\n");
			ret = EFAULT;
			break;
		}
		for (i = 0; i < msrs.nr_msr; i++, msr++) {
			fail = vcpu_set_msr(vcpu, msr->entry, msr->value);
			if (fail) {
				break;
			}
		}
		msrs.done = i;
		break;
	}
	case HAX_VCPU_IOCTL_GET_MSRS: {
		struct hax_msr_data msrs;
		struct vmx_msr *msr;
		int i, fail;

		if (copyin(ap->a_data, &msrs, sizeof(msrs))) {
			ret = EFAULT;
			break;
		}

		msr = msrs.entries;
		if(msrs.nr_msr >= 0x20) {
			hax_error("MSRS invalid!\n");
			ret = EFAULT;
			break;
		}
		for (i = 0; i < msrs.nr_msr; i++, msr++) {
			fail = vcpu_get_msr(vcpu, msr->entry, &msr->value);
			if (fail) {
				break;
			}
		}
		msrs.done = i;
		if (copyout(&msrs, ap->a_data, sizeof(msrs))) {
			ret = EFAULT;
			break;
		}
		break;
	}
	case HAX_VCPU_IOCTL_SET_FPU: {
		struct fx_layout fl;
		if (copyin(ap->a_data, &fl, sizeof(fl))) {
			ret = EFAULT;
			break;
		}
		ret = vcpu_put_fpu(vcpu, &fl);
		break;
	}
	case HAX_VCPU_IOCTL_GET_FPU: {
		struct fx_layout fl;
		ret = vcpu_get_fpu(vcpu, &fl);
		if (copyout(&fl, ap->a_data, sizeof(fl))) {
			ret = EFAULT;
			break;
		}
		break;
	}
	case HAX_VCPU_SET_REGS: {
		struct vcpu_state_t vc_state;

		if (copyin(ap->a_data, &vc_state, sizeof(vc_state))) {
			ret = EFAULT;
			break;
		}
		ret = vcpu_set_regs(vcpu, &vc_state);
		break;
	}
	case HAX_VCPU_GET_REGS: {
		struct vcpu_state_t vc_state;

		ret = vcpu_get_regs(vcpu, &vc_state);
		if (copyout(&vc_state, ap->a_data, sizeof(vc_state))) {
			ret = EFAULT;
			break;
		}
		break;
	}
	case HAX_VCPU_IOCTL_INTERRUPT: {
		uint8_t vector;

		if (copyin(ap->a_data, &vector, sizeof(vector))) {
			ret = EFAULT;
			break;
		}
		vcpu_interrupt(vcpu, vector);
		break;
	}
	case HAX_IOCTL_VCPU_DEBUG: {
		struct hax_debug_t hax_debug;
		if (copyin(ap->a_data, &hax_debug, sizeof(hax_debug))) {
			ret = EFAULT;
			break;
		}
		vcpu_debug(vcpu, &hax_debug);
		break;
	}
	default:
		// TODO: Print information about the process that sent the ioctl.
		hax_error("Unknown VCPU IOCTL 0x%lx\n", ap->a_cmd);
		ret = ENOSYS;
		break;
	}
	hax_put_vcpu(vcpu);

	return ret;
}

static int
haxm_modevent(module_t mod, int type, void *data)
{
	switch (type) {
	case MOD_LOAD:
		/*
		 * Linux checks if the CPU to be added is online,
		 * but there is no CPU hotplugging in DragonFly.
		 */
		for (int i = 0; i < ncpus; i++)
			cpu_online_map |= (1ULL << i);

		if (hax_module_init() < 0) {
			hax_error("Failed to initialized HAXM module\n");
			return EAGAIN;
		}

		hax_dev = make_dev(&hax_ops, 0, UID_ROOT, GID_WHEEL, 0600,
		    HAX_DEVICE_NAME);
		hax_info("Created HAXM device with minor=%d\n",
		    hax_dev->si_uminor);
		break;
	case MOD_UNLOAD:
		if (hax_module_exit() < 0) {
			hax_error("Failed to finalize HAXM module\n");
		}

		destroy_dev(hax_dev);
		hax_info("Removed HAXM device\n");
		break;
	case MOD_SHUTDOWN:
		break;
	default:
		break;
	}
	return 0;
}

DEV_MODULE(haxm, haxm_modevent, 0);
MODULE_VERSION(haxm, 1);
