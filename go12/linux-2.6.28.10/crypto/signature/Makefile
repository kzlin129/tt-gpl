#
# Makefile for the signature checker
#

HOSTLOADLIBES_pubkey2c = -lcrypto

hostprogs-y := pubkey2c

$(obj)/pubkey2c: $(src)/pubkey2c.c

quiet_cmd_pubkey2c = PUBKEY  $@
      cmd_pubkey2c = $(filter-out FORCE,$^) > $@

quiet_cmd_gendsa = GENDSA  $@
      cmd_gendsa = $(srctree)/scripts/gen_key_pair rootfs production $(srctree)/../production_keys

ifneq ($(RELEASE),)
TT_ROOTFS_KEY = $(srctree)/../production_keys/tt_rootfs_prod_pubkey.dsa
TT_LOOPFS_KEY = $(srctree)/keys/tt_loopfs_prod_pubkey.dsa

$(TT_ROOTFS_KEY): FORCE
	$(call if_changed,gendsa)
	
TT_FS_KEYS = $(TT_ROOTFS_KEY) $(TT_LOOPFS_KEY)
else
TT_FS_KEYS = $(srctree)/keys/tt_rootfs_dev_pubkey.dsa	\
	$(srctree)/keys/tt_loopfs_prod_pubkey.dsa	\
	$(srctree)/keys/tt_loopfs_dev_pubkey.dsa
endif

$(src)/dsa_pubkey.h: $(obj)/pubkey2c $(TT_FS_KEYS) FORCE
	$(call cmd,pubkey2c)

obj-y := dsasig.o

$(obj)/dsasig.o: $(src)/dsa_pubkey.h

clean-files := pubkey2c dsa_pubkey.h

PHONY += FORCE

