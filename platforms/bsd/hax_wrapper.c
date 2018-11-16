/*
 * Copyright (c) 2018 Kryptos Logic
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <machine/stdarg.h>

#include "../../include/hax.h"
#include "../../core/include/hax_core_interface.h"
#include "../../core/include/ia32.h"

int default_hax_log_level = 3;
int max_cpus;
hax_cpumap_t cpu_online_map;

/* Match what linux displays */
#define ID_ERR	"d3"
#define ID_WARN	"d4"
#define ID_INFO	"d6"
#define ID_DBG	"d7"

int hax_log_level(int level, const char *fmt,  ...)
{
    __va_list ap;

    if (level < default_hax_log_level)
        return 0;

    __va_start(ap, fmt);
    kprintf("%shaxm: ", ID_ERR);
    kvprintf(ap, fmt);
    kprintf("\n");
    __va_end(ap);

    return 0;
}

uint32_t hax_cpuid(void)
{
}

typedef struct smp_call_parameter {
    void (*func)(void *);
    void *param;
    hax_cpumap_t *cpus;
} smp_call_parameter;

static void smp_cfunction(void *p)
{
    struct smp_call_parameter *info = p;
    hax_cpumap_t *cpus;
    uint32_t cpuid;

    cpus = info->cpus;
    cpuid = hax_cpuid();
    if (*cpus & (0x1 << cpuid))
        info->func(info->param);
}

int hax_smp_call_function(hax_cpumap_t *cpus, void (*scfunc)(void *),
                          void *param)
{
}

/* XXX */
int proc_event_pending(struct vcpu_t *vcpu)
{
    return vcpu_event_pending(vcpu);
}

void hax_disable_preemption(preempt_flag *eflags)
{
}

void hax_enable_preemption(preempt_flag *eflags)
{
}

void hax_enable_irq(void)
{
    asm_enable_irq();
}

void hax_disable_irq(void)
{
    asm_disable_irq();
}

void hax_error(char *fmt, ...)
{
    __va_list ap;

    if (HAX_LOGE < default_hax_log_level)
        return;

    __va_start(ap, fmt);
    kprintf("%shaxm_error: ", ID_ERR);
    kvprintf(fmt, ap);
    kprintf("\n");
    __va_end(ap);
}

void hax_warning(char *fmt, ...)
{
    __va_list ap;

    if (HAX_LOGW < default_hax_log_level)
        return;

    __va_start(ap, fmt);
    kprintf("%shaxm_warning: ", ID_WARN);
    kvprintf(fmt, ap);
    kprintf("\n");
    __va_end(ap);
}

void hax_info(char *fmt, ...)
{
    __va_list ap;

    if (HAX_LOGI < default_hax_log_level)
        return;

    __va_start(ap, fmt);
    kprintf("%shaxm_info: ", ID_INFO);
    kvprintf(fmt, ap);
    kprintf("\n");
    __va_end(ap);
}

void hax_debug(char *fmt, ...)
{
    __va_list ap;

    if (HAX_LOGD < default_hax_log_level)
        return;

    __va_start(ap, fmt);
    kprintf("%shaxm_debug: ", ID_DBG);
    kvprintf(fmt, ap);
    kprintf("\n");
    __va_end(ap);
}

void hax_panic_vcpu(struct vcpu_t *v, char *fmt, ...)
{
    __va_list ap;

    __va_start(ap, fmt);
    kprintf("%shaxm_panic: ", ID_ERR);
    kvprintf(fmt, ap);
    kprintf("\n");
    __va_end(ap);
    vcpu_set_panic(v);
}

/* Misc */
void hax_smp_mb(void)
{
}

/* Compare-Exchange */
bool hax_cmpxchg32(uint32_t old_val, uint32_t new_val, volatile uint32_t *addr)
{
}

bool hax_cmpxchg64(uint64_t old_val, uint64_t new_val, volatile uint64_t *addr)
{
}

/* Atomics */
hax_atomic_t hax_atomic_add(volatile hax_atomic_t *atom, uint32_t value)
{
}

hax_atomic_t hax_atomic_inc(volatile hax_atomic_t *atom)
{
}

hax_atomic_t hax_atomic_dec(volatile hax_atomic_t *atom)
{
}

int hax_test_and_set_bit(int bit, uint64_t *memory)
{
}

int hax_test_and_clear_bit(int bit, uint64_t *memory)
{
}

/* Spinlock */
struct hax_spinlock {
    struct spinlock lock;
};

hax_spinlock *hax_spinlock_alloc_init(void)
{
}

void hax_spinlock_free(hax_spinlock *lock)
{
}

void hax_spin_lock(hax_spinlock *lock)
{
}

void hax_spin_unlock(hax_spinlock *lock)
{
}

/* Mutex */
hax_mutex hax_mutex_alloc_init(void)
{
}

void hax_mutex_lock(hax_mutex lock)
{
}

void hax_mutex_unlock(hax_mutex lock)
{
}

void hax_mutex_free(hax_mutex lock)
{
}
