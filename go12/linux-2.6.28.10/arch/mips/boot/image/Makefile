#
# MAKEFILE FOR MIPS LINUX BOOTLOADER
#


ifndef Version
 Version = "$(USER)"
endif

$(warning Version=$(Version))

KERNEL_IMAGE = $(VMLINUX)
KERNEL_START = $(shell $(NM) $(KERNEL_IMAGE) | grep " _text" | cut -f1 -d\ )
KERNEL_ENTRY = $(shell $(NM) $(KERNEL_IMAGE) | grep kernel_entry | cut -f1 -d\ )

$(warning KERNEL_IMAGE=$(KERNEL_IMAGE))
$(warning KERNEL_START=$(KERNEL_START))
$(warning KERNEL_ENTRY=$(KERNEL_ENTRY))
$(warning obj=$(obj))

LDSCRIPT= -L$(obj) -T$(TOPDIR)/$(obj)/romscript.lds

HEAD_DEFINES := -D_kernel_start=0x$(KERNEL_START) \
		-D_kernel_entry=0x$(KERNEL_ENTRY) \
		-D VERSION="\"$(Version)\"" \
		-D TIMESTAMP=$(shell date +%s)

$(obj)/head.o: $(obj)/head.S $(KERNEL_IMAGE) $(obj)/kImage.bin $(obj)/kImage.crc32
	@echo '  Making $@ '
	@echo '  $(CC) -fno-pic $(HEAD_DEFINES) -I$(obj) -I$(TOPDIR)/include -c -o $@ $< '
	$(CC) -fno-pic $(HEAD_DEFINES) -I$(obj) -I$(TOPDIR)/include -c -o $@ $<

$(obj)/kImage.o: $(obj)/kImage.bin.gz
	@echo ' Making $@ '
	@echo '  $(LD) -r -o $@ -b binary $< '
	$(LD) -r -o $@ -b binary $<

OBJECTS = head.o kImage.o

Image : $(obj)/Image
       
$(obj)/Image: $(obj)/rom
	@echo '  Making $@ '
	@echo '  $(OBJCOPY) -O binary -S $^ $@ '
	$(OBJCOPY) -O binary -S $^ $@

# Rule to make the bootloader
$(obj)/rom: $(addprefix $(obj)/,$(OBJECTS))
	@echo '  Making $@ '
	@echo '  $(LD) $(LDFLAGS) $(LDSCRIPT) -o $@ $^ '
	$(LD) $(LDFLAGS) $(LDSCRIPT) -o $@ $^

$(obj)/kImage.bin.gz: $(obj)/kImage.bin
	@echo '  Making $@ '
	gzip -9 --stdout $< > $@

$(obj)/kImage.crc32.raw : $(obj)/kImage.bin
	crc32 $< >$@

$(obj)/kImage.crc32 : $(obj)/kImage.crc32.raw
	sed -e 's/.*/\t.word &/' $< >$@

$(obj)/kImage.bin: $(KERNEL_IMAGE)
	@echo '  Making $@ '
	@echo '  $(OBJCOPY) -O binary -S $^ $@ '
	$(OBJCOPY) -O binary -S $^ $@

clean:
	rm -f $(obj)/rom $(obj)/kImage.bin $(obj)/kImage.o $(obj)/Image $(obj)/head.o
