# WSN Deployment System - Contiki-NG Integration

## Overview

This project implements the WSN (Wireless Sensor Network) Deployment System based on the specifications in `main.tex`. The implementation includes both standalone C versions and Contiki-NG integrated versions for IoT simulation.

## Implementation Details

### Based on main.tex APP_I Specifications

The implementation follows the deterministic grid-based approach (APP_I) with:

1. **Global Phase (Algorithm 1)** - Base Station coordination
2. **Local Phase (Algorithm 2)** - Mobile Robot execution
3. **Topology Discovery Phase (Algorithm 3)** - Sensor discovery
4. **Dispersion Phase (Algorithm 4)** - Four-case sensor deployment

### System Components

#### 1. Base Station (`base-station-contiki.c`)
- Implements global phase coordination
- Manages LA_DB (Location Area Database) and Robot_DB
- Handles robot deployment and completion messages
- Calculates area coverage percentage (Per_AC)

#### 2. Mobile Robot (`mobile-robot-contiki.c`)
- Executes local phase in assigned location areas
- Implements topology discovery and dispersion phases
- Handles the four dispersion cases:
  - **Case 1**: Robot has sensors, grid has sensors
  - **Case 2**: Robot has sensors, grid empty
  - **Case 3**: Robot has no sensors, grid has sensors
  - **Case 4**: Robot has no sensors, grid empty

#### 3. Sensor Node (`sensor-node-contiki.c`)
- Responds to robot broadcast messages
- Supports relocation and status updates
- Simulates sensing operations and battery management

#### 4. Integrated Application (`wsn-deployment-integrated.c`)
- Demonstrates complete local phase execution
- Simulates all algorithms from main.tex
- Provides performance monitoring and statistics

## System Parameters (from main.tex)

- **Target Area Size**: 1000x1000 units
- **Robot Perception Range**: 100 units
- **Sensor Perception Range**: 50 units
- **Robot Capacity**: 15 sensors maximum
- **Initial Stock (Stock_RS)**: 10 sensors per robot
- **Number of Robots**: 2 (Robot_1, Robot_2)

## Database Structures (as per main.tex)

### LA_DB (Location Area Database)
```
LA_id | X-Y coordinate of center | NO_Grid
```
Size: NO_LA * (log₂(NO_LA) + 16 + log₂(NO_G)) bits

### Robot_DB (Robot Database)
```
Robot_id | A_LA_id
```
Size: 2 * (1 + log₂(NO_LA)) bits

### Grid_DB (Grid Database)
```
Grid_id | X-Y coordinate of center | Grid_Status
```
Size: NO_G * (log₂(NO_G) + 17) bits

### Sensor_DB (Sensor Database)
```
Sensor_id | X-Y coordinate | Sensor_Status
```
Size: Ns * (log₂(Ns) + 17) bits

## Message Formats (from main.tex)

1. **Robot_pM**: (Robot_p, Cov_G) - Size: 1 + log₂(NO_G) bits
2. **Mp**: (Robot_p) - Size: 1 bit
3. **Sensor_M**: (Sensor_id, X-Y coordinate, Sensor_Status) - Size: log₂(Ns) + 17 bits

## Building and Running

### Prerequisites
- GCC compiler
- Contiki-NG (for simulation)
- Java (for Cooja simulator)

### Standalone Versions
```bash
make all                    # Build all standalone components
make run-base-station      # Run base station
make run-mobile-robot      # Run mobile robot
make run-sensor-node       # Run sensor node
make run-local-phase       # Run integrated local phase
```

### Contiki-NG Integration
```bash
make contiki-build         # Build Contiki-NG versions
make run-simulation        # Start Cooja simulation
make run-integrated        # Run integrated demonstration
```

### Documentation
```bash
make docs                  # Show implementation details
```

## File Structure

```
src/
├── common.h                           # Shared data structures and definitions
├── common.c                           # Common utility functions
├── base-station.c                     # Standalone base station
├── mobile-robot.c                     # Standalone mobile robot
├── sensor-node.c                      # Standalone sensor node
├── local-phase.c                      # Integrated local phase demo
├── base-station-contiki.c             # Contiki-NG base station
├── mobile-robot-contiki.c             # Contiki-NG mobile robot
├── sensor-node-contiki.c              # Contiki-NG sensor node
├── wsn-deployment-integrated.c        # Complete system demonstration
├── project-conf.h                     # Contiki-NG project configuration
├── Makefile                           # Build system
├── Makefile.contiki                   # Contiki-NG specific makefile
└── wsn-deployment-simulation-contiki.csc  # Cooja simulation file
```

## Simulation Setup

The Cooja simulation includes:
- 1 Base Station node (center of area)
- 2 Mobile Robot nodes (Robot_1 at LA_1, Robot_2 at LA_NO_LA)
- 10 Sensor nodes (randomly distributed)

### Network Configuration
- UDP communication on ports 8765/5678
- IPv6 networking with RPL routing
- CSMA MAC layer with null RDC

## Algorithm Implementation

### Algorithm 1: Global Phase
1. Initialize target area and partition into LAs
2. Initialize databases (Robot_DB, LA_DB, Grid_DB, Sensor_DB)
3. Deploy robots in initial LAs
4. Receive completion messages and reassign robots
5. Compute final area coverage percentage

### Algorithm 2: Local Phase
1. Divide assigned LA into NO_G grids
2. Insert records into Grid_DB
3. Execute topology discovery phase
4. Initialize NO_P = NO_G
5. Execute dispersion phase
6. Count covered grids and send completion message

### Algorithm 3: Topology Discovery Phase
1. Move robot to center of assigned LA
2. Broadcast Mp message to discover sensors
3. Receive Sensor_M replies from sensors in range
4. Update local Sensor_DB with discovered sensors

### Algorithm 4: Dispersion Phase
1. While NO_P > 0:
   - Process current grid according to four cases
   - Decrement NO_P
   - Move to next uncovered grid
2. Handle sensor placement and collection based on stock availability

## Performance Metrics

The system tracks:
- Area coverage percentage (Per_AC)
- Number of covered grids per LA
- Robot stock levels and movements
- Database memory usage
- Message transmission counts and sizes

## Key Features

1. **Exact Implementation**: Follows main.tex specifications precisely
2. **Four Dispersion Cases**: Complete implementation of all sensor deployment scenarios
3. **Database Management**: Accurate size calculations and record management
4. **Message Protocol**: Implements all message formats from the specification
5. **Contiki-NG Integration**: Full IoT simulation capability
6. **Performance Analysis**: Detailed metrics and monitoring

## Usage Examples

### Running Complete Local Phase Simulation
```bash
make run-integrated
```
This demonstrates the complete local phase execution with all algorithms.

### Running Cooja Network Simulation
```bash
make run-simulation
```
This starts the Cooja simulator with the pre-configured network topology.

### Testing Individual Components
```bash
make run-base-station    # Test base station functionality
make run-mobile-robot    # Test mobile robot local phase
make run-sensor-node     # Test sensor node responses
```

## Configuration

System parameters can be modified in:
- `common.h` - Basic constants and structures
- `project-conf.h` - Contiki-NG specific configuration
- Simulation file - Network topology and node placement

## Troubleshooting

1. **Contiki-NG not found**: Set CONTIKI_DIR in Makefile to your installation
2. **Java not found**: Install Java for Cooja simulator
3. **Compilation errors**: Check GCC and required libraries
4. **Simulation issues**: Verify Cooja configuration and node setup

## References

- main.tex - Original specification document
- Contiki-NG documentation
- Cooja simulator manual
- WSN deployment research papers

This implementation provides a complete, simulation-ready WSN deployment system that faithfully implements the algorithms and specifications from the main.tex document.
