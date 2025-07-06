# WSN Deployment Project Makefile
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g -O2
LDFLAGS = -lm

# Project executables
TARGETS = base-station mobile-robot sensor-node local-phase

# Source files
BASE_STATION_SRC = base-station.c
MOBILE_ROBOT_SRC = mobile-robot.c
SENSOR_NODE_SRC = sensor-node.c
LOCAL_PHASE_SRC = local-phase.c

# Common dependencies
COMMON_DEPS = common.h

# Default target
all: $(TARGETS)

# Base station executable
base-station: $(BASE_STATION_SRC) $(COMMON_DEPS)
	$(CC) $(CFLAGS) -o $@ $(BASE_STATION_SRC) $(LDFLAGS)

# Mobile robot executable
mobile-robot: $(MOBILE_ROBOT_SRC) $(COMMON_DEPS)
	$(CC) $(CFLAGS) -o $@ $(MOBILE_ROBOT_SRC) $(LDFLAGS)

# Sensor node executable
sensor-node: $(SENSOR_NODE_SRC) $(COMMON_DEPS)
	$(CC) $(CFLAGS) -o $@ $(SENSOR_NODE_SRC) $(LDFLAGS)

# Local phase integration executable
local-phase: $(LOCAL_PHASE_SRC) $(COMMON_DEPS)
	$(CC) $(CFLAGS) -o $@ $(LOCAL_PHASE_SRC) mobile-robot.c sensor-node.c $(LDFLAGS)

# Run targets
run-base-station: base-station
	./base-station

run-mobile-robot: mobile-robot
	./mobile-robot

run-sensor-node: sensor-node
	./sensor-node

run-local-phase: local-phase
	./local-phase

# Contiki-NG specific targets (for simulation)
CONTIKI_PROJECT = base-station mobile-robot sensor-node
CONTIKI = /home/uttam/contiki-ng

# Enable Contiki-NG build when needed
contiki: $(CONTIKI_PROJECT)
	@echo "Building Contiki-NG versions..."

# Clean target
clean:
	rm -f $(TARGETS)
	rm -f *.o
	rm -rf build/

# Contiki-NG Integration
CONTIKI_TARGETS = wsn-base-station wsn-mobile-robot wsn-sensor-node wsn-integrated
CONTIKI_DIR = ../../../contiki-ng

# Build Contiki-NG versions
contiki-build: $(CONTIKI_TARGETS)
	@echo "Building Contiki-NG versions..."
	@if [ -d "$(CONTIKI_DIR)" ]; then \
		cp base-station-contiki.c wsn-base-station.c; \
		cp mobile-robot-contiki.c wsn-mobile-robot.c; \
		cp sensor-node-contiki.c wsn-sensor-node.c; \
		cp wsn-deployment-integrated.c wsn-integrated.c; \
		make -f Makefile.contiki TARGET=cooja; \
	else \
		echo "Contiki-NG not found at $(CONTIKI_DIR)"; \
		echo "Please set CONTIKI_DIR to your Contiki-NG installation"; \
	fi

# Run Cooja simulation
run-simulation: contiki-build
	@echo "Starting Cooja simulation..."
	@if [ -d "$(CONTIKI_DIR)" ]; then \
		java -jar $(CONTIKI_DIR)/tools/cooja/dist/cooja.jar wsn-deployment-simulation-contiki.csc; \
	else \
		echo "Contiki-NG not found. Please install Contiki-NG first."; \
	fi

# Build integrated local phase demonstration
build-integrated: wsn-deployment-integrated.c common.c common.h
	$(CC) $(CFLAGS) -o wsn-integrated wsn-deployment-integrated.c common.c $(LDFLAGS)

run-integrated: build-integrated
	./wsn-integrated

# Documentation generation
docs:
	@echo "Generating documentation from main.tex..."
	@echo "This implementation includes:"
	@echo "1. Base Station (Global Phase) - Algorithm 1"
	@echo "2. Mobile Robot (Local Phase) - Algorithms 2, 3, 4"
	@echo "3. Sensor Nodes (Response to robot broadcasts)"
	@echo "4. Complete Contiki-NG integration"
	@echo "5. Cooja simulation setup"

# Include the Contiki-NG build system for simulation builds
ifneq ($(CONTIKI_DIR),)
ifneq ($(wildcard $(CONTIKI_DIR)),)
include $(CONTIKI_DIR)/Makefile.include
endif
endif

.PHONY: all clean run-base-station run-mobile-robot run-sensor-node run-local-phase contiki contiki-build run-simulation build-integrated run-integrated docs
