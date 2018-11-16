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

#include <vm/vm_page.h>

#include "../../include/hax.h"

typedef struct hax_vcpu_mem_hinfo_t {
    int flags;
    int nr_pages;
    vm_page_t *pages;
} hax_vcpu_mem_hinfo_t;

int hax_clear_vcpumem(struct hax_vcpu_mem *mem)
{
}

int hax_setup_vcpumem(struct hax_vcpu_mem *mem, uint64_t uva, uint32_t size,
                      int flags)
{
}

uint64_t hax_get_memory_threshold(void)
{
#ifdef CONFIG_HAX_EPT2
    // Since there is no memory cap, just return a sufficiently large value
    return 1ULL << 48;  // PHYSADDR_MAX + 1
#else  // !CONFIG_HAX_EPT2
    return 0;
#endif  // CONFIG_HAX_EPT2
}
