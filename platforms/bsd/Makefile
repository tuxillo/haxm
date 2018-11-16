# DragonFly makefile

KMOD= haxm

# core sources
.PATH: ${.CURDIR}/../../core

# This is just for getting the .o file names, they are handled in the nasm hack
SRCS+= emulate_ops.asm ia32_ops.asm vmx_ops.asm

SRCS+= nasm_hack.h
SRCS+= chunk.c cpu.c cpuid.c dump.c emulate.c ept.c ept2.c
SRCS+= ept_tree.c gpa_space.c hax.c ia32.c intr_exc.c memory.c
SRCS+= memslot.c name.c page_walker.c ramblock.c vcpu.c vm.c vmx.c
SRCS+= vtlb.c

# haxm-bsd
SRCS+= haxm_dev.c hax_mem_alloc.c hax_host_mem.c hax_wrapper.c
SRCS+= hax_mm.c hax_event.c

# Horrible hack, don't blame me :)
nasm_hack.h:
	@nasm -f elf64 ../../core/emulate_ops.asm -o emulate_ops.o
	@nasm -f elf64 ../../core/ia32_ops.asm -o ia32_ops.o
	@nasm -f elf64 ../../core/vmx_ops.asm -o vmx_ops.o

#KCFLAGS+= -Wno-cast-qual -Wno-unused-function -Wno-discarded-qualifiers

# dragonfly specific sources

.include <bsd.kmod.mk>
