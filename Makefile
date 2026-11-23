# Reflex build
#
#   QNX 8.0 target (run `source ~/qnx800/qnxsdp-env.sh` first):
#       make QNX=1 ARCH=aarch64      Raspberry Pi 4 / 5
#       make QNX=1 ARCH=x86_64       QEMU
#   Linux reference build (A/B baseline, host tests):
#       make
#       make test
#
#   Host helpers (Ubuntu 24.04):
#       make setup                   one-time host install (scripts/setup-host-ubuntu24.sh)
#       make doctor [PI=user@host]   check the host (and the target) are ready
#       make fw / make flash         build / DFU-flash the I/O node firmware (BOARD=BLACKPILL_F401CC, EMULATE=1 for the plant emulator)
#       make deploy PI=user@host     build for QNX and copy to the target
#
#   Optional QNX pieces, each can be switched off if your image lacks the library:
#       HAM=0      do not build rxham   (needs libham)
#       TRACE=0    no kernel trace events from the guardian

QNX   ?= 0
ARCH  ?= aarch64
HAM   ?= 1
TRACE ?= 1

CFLAGS ?= -O2 -g
CFLAGS += -std=gnu11 -Wall -Wextra -Isrc/core -Isrc/ipc -Isrc/guardian -Isrc/tools
ifeq ($(WERROR),1)
  CFLAGS += -Werror
endif

ifeq ($(QNX),1)
  ifeq ($(ARCH),x86_64)
    CC := ntox86_64-gcc
  else
    CC := ntoaarch64-gcc
  endif
  OUT     := build/qnx-$(ARCH)
  IPC     := src/ipc/rx_ipc_qnx.c
  CFLAGS  += -D_QNX_SOURCE
  LIBS    := -lm
  NETLIBS := -lsocket
  ifeq ($(TRACE),0)
    CFLAGS += -DRX_NO_TRACE
  endif
  EXTRA   := $(OUT)/rxfs
  ifeq ($(HAM),1)
    EXTRA += $(OUT)/rxham
  endif
else
  OUT     := build/linux
  IPC     := src/ipc/rx_ipc_posix.c
  CFLAGS  += -D_GNU_SOURCE
  LIBS    := -lm -lpthread -lrt
  NETLIBS :=
  EXTRA   :=
endif

CORE := src/core/rx_core.c src/core/rx_shm.c src/core/rx_learner.c src/core/rx_link.c src/core/rx_conf.c src/core/rx_wire.c
UDP  := src/ipc/rx_ipc_udp.c
HDRS := $(wildcard src/core/*.h src/ipc/*.h src/guardian/*.h src/tools/*.h)

BINS := $(OUT)/reflex-guardian $(OUT)/reflex-pilot $(OUT)/rxctl $(OUT)/rxdash $(OUT)/rxcal $(OUT)/rx_simtest $(OUT)/rx_linktest $(EXTRA)

all: $(BINS)
	@echo "built: $(BINS)"

$(OUT):
	mkdir -p $(OUT)

$(OUT)/reflex-guardian: src/guardian/main.c src/guardian/rx_io.c $(CORE) $(IPC) $(UDP) $(HDRS) | $(OUT)
	$(CC) $(CFLAGS) -o $@ src/guardian/main.c src/guardian/rx_io.c $(CORE) $(IPC) $(UDP) $(LIBS) $(NETLIBS)

$(OUT)/reflex-pilot: src/pilot/main.c $(CORE) $(IPC) $(UDP) $(HDRS) | $(OUT)
	$(CC) $(CFLAGS) -o $@ src/pilot/main.c $(CORE) $(IPC) $(UDP) $(LIBS) $(NETLIBS)

$(OUT)/rxctl: src/tools/rxctl.c $(CORE) $(HDRS) | $(OUT)
	$(CC) $(CFLAGS) -o $@ src/tools/rxctl.c $(CORE) $(LIBS)

$(OUT)/rxdash: src/tools/rxdash.c src/tools/dash_html.h $(CORE) $(HDRS) | $(OUT)
	$(CC) $(CFLAGS) -o $@ src/tools/rxdash.c $(CORE) $(LIBS) $(NETLIBS)

$(OUT)/rx_simtest: src/tools/rx_simtest.c src/core/rx_core.c src/core/rx_learner.c $(HDRS) | $(OUT)
	$(CC) $(CFLAGS) -o $@ src/tools/rx_simtest.c src/core/rx_core.c src/core/rx_learner.c $(LIBS)

$(OUT)/rx_linktest: src/tools/rx_linktest.c src/core/rx_link.c src/core/rx_conf.c src/core/rx_wire.c $(HDRS) | $(OUT)
	$(CC) $(CFLAGS) -o $@ src/tools/rx_linktest.c src/core/rx_link.c src/core/rx_conf.c src/core/rx_wire.c $(LIBS)

$(OUT)/rxcal: src/tools/rxcal.c $(CORE) $(HDRS) | $(OUT)
	$(CC) $(CFLAGS) -o $@ src/tools/rxcal.c $(CORE) $(LIBS)

$(OUT)/rxfs: src/qnx/rxfs.c $(CORE) $(HDRS) | $(OUT)
	$(CC) $(CFLAGS) -o $@ src/qnx/rxfs.c $(CORE) $(LIBS)

$(OUT)/rxham: src/qnx/rxham.c | $(OUT)
	$(CC) $(CFLAGS) -o $@ src/qnx/rxham.c -Wl,-Bstatic -lham -Wl,-Bdynamic   # libham.so is not on the Pi 4 quick-start image

# The pilot as a native, statically linked Linux program: for a Linux guest under QNX Hypervisor, or any
# Linux machine on the network (pilot_transport = udp, docs/HYPERVISOR.md). Same sources, same fault hooks
# and learner as the pilot the guardian spawns; it talks UDP (reflex-pilot --udp <guardian a.b.c.d:port>).
#   make linux-pilot                          LINUX_ARCH=aarch64 (default): aarch64-linux-gnu-gcc
#   make linux-pilot LINUX_ARCH=x86_64        host gcc (laptop tests, make test-udp)
#   make linux-pilot LINUX_CC=<sdk>/aarch64-poky-linux-gcc   any other aarch64 Linux cross compiler
LINUX_ARCH ?= aarch64
ifeq ($(LINUX_ARCH),x86_64)
  LINUX_CC ?= gcc
else
  LINUX_CC ?= aarch64-linux-gnu-gcc
endif
LP_OUT := build/linux-pilot-$(LINUX_ARCH)
LP_SRC := src/pilot/main.c src/core/rx_core.c src/core/rx_learner.c src/core/rx_shm.c src/core/rx_link.c src/core/rx_conf.c \
          src/core/rx_wire.c src/ipc/rx_ipc_posix.c $(UDP)

linux-pilot: $(LP_OUT)/reflex-pilot

$(LP_OUT)/reflex-pilot: $(LP_SRC) $(HDRS)
	@command -v $(LINUX_CC) >/dev/null || { echo "linux-pilot: '$(LINUX_CC)' not found. Install an aarch64 Linux cross compiler"; \
	  echo "  (Ubuntu: sudo apt install gcc-aarch64-linux-gnu) or point LINUX_CC at one, e.g. a Yocto SDK's."; exit 1; }
	mkdir -p $(LP_OUT)
	$(LINUX_CC) -O2 -g -std=gnu11 -Wall -Wextra $(if $(filter 1,$(WERROR)),-Werror) -D_GNU_SOURCE -Isrc/core -Isrc/ipc \
	  -static -o $@ $(LP_SRC) -lm -lpthread -lrt
	@echo "built: $@"

# regenerate the embedded dashboard after editing web/dashboard.html
src/tools/dash_html.h: web/dashboard.html tools/embed.py
	python3 tools/embed.py web/dashboard.html $@

# retrain the neural pilots (needs numpy): simulator weights and serial-backend (hardware) weights
policy:
	python3 tools/train_policy.py
	python3 tools/train_policy.py --hardware

test: $(OUT)/rx_simtest $(OUT)/rx_linktest
	$(OUT)/rx_linktest
	$(OUT)/rx_simtest

# end-to-end serial path against tools/fake_io_node.py (Linux build, ~5 minutes)
test-serial:
	./scripts/test-serial.sh

test-cal:
	./scripts/test-cal.sh

clean:
	rm -rf build

setup:
	./scripts/setup-host-ubuntu24.sh

doctor:
	./scripts/doctor.sh

BOARD   ?= BLACKPILL_F401CC
EMULATE ?= 0
fw:
	BOARD=$(BOARD) EMULATE=$(EMULATE) ./scripts/fw.sh build

flash:
	BOARD=$(BOARD) EMULATE=$(EMULATE) ./scripts/fw.sh flash

deploy:
	./scripts/deploy.sh

.PHONY: all test test-serial test-cal clean policy setup doctor fw flash deploy linux-pilot
