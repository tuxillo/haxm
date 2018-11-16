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

#include "../../include/hax_host_mem.h"
#include "../../core/include/paging.h"

int hax_pin_user_pages(uint64_t start_uva, uint64_t size, hax_memdesc_user *memdesc)
{
	return EOPNOTSUPP;	/* NOT IMPLEMENTED */
}

int hax_unpin_user_pages(hax_memdesc_user *memdesc)
{
	return EOPNOTSUPP;	/* NOT IMPLEMENTED */
}

uint64_t hax_get_pfn_user(hax_memdesc_user *memdesc, uint64_t uva_offset)
{
	return EOPNOTSUPP;	/* NOT IMPLEMENTED */
}

void * hax_map_user_pages(hax_memdesc_user *memdesc, uint64_t uva_offset,
                          uint64_t size, hax_kmap_user *kmap)
{
	return NULL;	/* NOT IMPLEMENTED */
}

int hax_unmap_user_pages(hax_kmap_user *kmap)
{
	return EOPNOTSUPP;	/* NOT IMPLEMENTED */
}

int hax_alloc_page_frame(uint8_t flags, hax_memdesc_phys *memdesc)
{
	return EOPNOTSUPP;	/* NOT IMPLEMENTED */
}

int hax_free_page_frame(hax_memdesc_phys *memdesc)
{
	return EOPNOTSUPP;	/* NOT IMPLEMENTED */
}

uint64_t hax_get_pfn_phys(hax_memdesc_phys *memdesc)
{
	return EOPNOTSUPP;	/* NOT IMPLEMENTED */
}

void * hax_get_kva_phys(hax_memdesc_phys *memdesc)
{
	return NULL;	/* NOT IMPLEMENTED */
}

void * hax_map_page_frame(uint64_t pfn, hax_kmap_phys *kmap)
{
	return NULL;	/* NOT IMPLEMENTED */
}

int hax_unmap_page_frame(hax_kmap_phys *kmap)
{
	return EOPNOTSUPP;	/* NOT IMPLEMENTED */
}
