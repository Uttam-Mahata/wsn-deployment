CONTIKI_PROJECT = base-station mobile-robot sensor-node

all: $(CONTIKI_PROJECT)

# Path to Contiki-NG
# Allow overriding with an environment variable, otherwise default to a common path or a relative one.
CONTIKI ?= /opt/contiki-ng
# If /opt/contiki-ng is not available, one might try a relative path assuming contiki-ng is cloned peer to this project's parent
# CONTIKI ?= ../../contiki-ng

# Add math library for calculations
TARGET_LIBFILES += -lm

# Add project source files

# Include the Contiki-NG build system
include $(CONTIKI)/Makefile.include
