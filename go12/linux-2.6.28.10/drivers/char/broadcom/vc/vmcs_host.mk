# Makefile for building vmcs_host for running on vc02
# Target is vmcs_host.a

LIB_NAME    := vmcs_host
LIB_VPATH   := vc02_local
LIB_IPATH   := .. vc02_local
LIB_SRC     := vc_dispman2.c vcfilesys.c vcgencmd.c \
               vchostreq.c vciface.c vcmsgfifo.c vcstate.c vchostmem.c vcringtone.c vctouch.c \
               vcutil.c vcfile.c vcfs.c vchost.c vchr.c vcih.c vcos.c vchost_audio.c
LIB_DEF     :=               
LIB_LIBS    := latch/latch
LIB_CFLAGS  :=
LIB_AFLAGS  :=
LIB_TYPE    := TEST
