CONTIKI_PROJECT = base-station sensor-node mobile-robot
all: $(CONTIKI_PROJECT)

# Define deployment strategy flags
CFLAGS += -DAPP_I_DEPLOYMENT        # Using APP_I strategy
CFLAGS += -DINITIAL_RANDOM_DEPLOY   # Initial random deployment of sensors

# For math functions like sqrt
TARGET_LIBFILES += -lm

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
