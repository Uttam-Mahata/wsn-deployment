CONTIKI_PROJECT = base-station mobile-robot sensor-node

# Include modules for WSN deployment
all: $(CONTIKI_PROJECT)

# Include the Contiki-NG root Makefile
CONTIKI = /home/uttam/contiki-ng

# Enable UDP and IPv6
MODULES += os/net/app-layer/coap

# Add debug flags for better logging
CFLAGS += -DWITH_COAP=1
CFLAGS += -DUIP_CONF_IPV6_CHECKS=1
CFLAGS += -DUIP_CONF_TCP=0

# WSN Deployment specific flags
CFLAGS += -DWSN_DEPLOYMENT_PARALLEL_PROCESSING=1

# Add math library for calculations
TARGET_LIBFILES += -lm

# Add project source files

# Include the Contiki-NG build system
include $(CONTIKI)/Makefile.include
