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

#include "../../include/hax.h"

void * hax_vmalloc(uint32_t size, uint32_t flags)
{
	return NULL;	/* NOT IMPLEMENTED */
}

void hax_vfree_flags(void *va, uint32_t size, uint32_t flags)
{
	return;	/* NOT IMPLEMENTED */
}

void hax_vfree(void *va, uint32_t size)
{
}

void hax_vfree_aligned(void *va, uint32_t size, uint32_t alignment,
                       uint32_t flags)
{
}

void * hax_vmap(hax_pa_t pa, uint32_t size)
{
}

void hax_vunmap(void *addr, uint32_t size)
{
}

hax_pa_t hax_pa(void *va)
{
}

struct hax_page * hax_alloc_pages(int order, uint32_t flags, bool vmap)
{
}

void hax_free_pages(struct hax_page *pages)
{
}

void * hax_map_page(struct hax_page *page)
{
    if (!page)
        return NULL;

    return page->kva;
}

void hax_unmap_page(struct hax_page *page)
{
    return;
}

hax_pfn_t hax_page2pfn(struct hax_page *page)
{
    if (!page)
        return 0;

    return page->pa >> PAGE_SHIFT;
}

void hax_clear_page(struct hax_page *page)
{
    memset((void *)page->kva, 0, PAGE_SIZE);
}

void hax_set_page(struct hax_page *page)
{
    memset((void *)page->kva, 0xFF, PAGE_SIZE);
}

/* Initialize memory allocation related structures */
int hax_malloc_init(void)
{
    return 0;
}

void hax_malloc_exit(void)
{
}
