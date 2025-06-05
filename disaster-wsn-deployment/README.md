# Disaster WSN Deployment Simulation

This project implements a disaster Wireless Sensor Network (WSN) deployment system based on the APP_I approach described in the research paper. The implementation simulates a deterministic grid-based deployment strategy for optimal sensor coverage in disaster-hit areas.

## Overview

The system consists of three main components:

1. **Base Station (BS)**: Executes the global phase, manages location areas, and coordinates robot deployment
2. **Mobile Robots**: Execute the local phase with topology discovery and sensor dispersion
3. **Sensor Nodes**: Respond to deployment commands and simulate sensing activities

## Key Features

- **Grid-based Coverage**: Target area is divided into location areas (LAs) and grids for systematic coverage
- **Two-Phase Operation**: Global phase (BS coordination) and Local phase (robot execution)
- **Database Management**: Implementation of LA_DB, Robot_DB, Grid_DB, and Sensor_DB as described in the paper
- **Energy Simulation**: Battery-aware operation with different consumption modes
- **Message-based Communication**: UDP-based inter-node communication following the protocol specification

## Architecture

### APP_I Implementation

The implementation follows the deterministic approach with:

- **Target Area**: 200x200 units divided into location areas based on robot perception range (50 units)
- **Robot Perception Range**: 50 units
- **Sensor Perception Range**: 20 units  
- **Grid Size**: Calculated as robot_range/sensor_range = 2.5x2.5 grids per LA
- **Robot Stock**: Initially 10 sensors per robot (capacity: 15 sensors)

### System Workflow

1. **Initialization**: BS calculates location areas and initializes databases
2. **Robot Deployment**: BS deploys robots to initial LAs
3. **Topology Discovery**: Robots broadcast discovery messages to find sensors
4. **Dispersion Phase**: Robots redistribute sensors according to 4 cases:
   - Case 1: Robot has sensors + Grid has sensors
   - Case 2: Robot has sensors + Grid empty
   - Case 3: Robot empty + Grid has sensors  
   - Case 4: Both empty (grid remains uncovered)
5. **Reporting**: Robots report coverage statistics to BS
6. **Iteration**: Process continues until all LAs are processed

## Files Description

### Core Implementation

- **`base-station.c`**: Base station implementation with global phase logic
- **`mobile-robot.c`**: Mobile robot with local phase execution
- **`sensor-node.c`**: Sensor node with energy simulation and response logic
- **`project-conf.h`**: Project configuration and parameters
- **`Makefile`**: Build configuration for all components

### Simulation Setup

- **`disaster-wsn-cooja.csc`**: Cooja simulator configuration with:
  - 1 Base station (center position)
  - 2 Mobile robots (corner positions)  
  - 13 Sensor nodes (randomly distributed)
  - Network visualization and logging plugins

## Building and Running

### Prerequisites

- Contiki-NG development environment
- Cooja simulator
- GCC toolchain

### Building

```bash
cd examples/disaster-wsn-deployment
make TARGET=cooja
```

This will compile all three node types:
- `base-station.cooja`
- `mobile-robot.cooja`  
- `sensor-node.cooja`

### Running in Cooja

1. Start Cooja simulator:
   ```bash
   cd [CONTIKI_ROOT]/tools/cooja
   ant run
   ```

2. Open the simulation:
   - File → Open simulation → Select `disaster-wsn-cooja.csc`

3. Start the simulation:
   - Click "Start" button
   - Observe the log output for deployment progress

### Expected Output

The simulation will show:

1. **Base Station**: 
   - Location area calculation
   - Robot deployment commands
   - Coverage reports from robots
   - Final coverage percentage

2. **Mobile Robots**:
   - Deployment acknowledgments
   - Topology discovery phase
   - Grid processing and sensor dispersion
   - Coverage reports to BS

3. **Sensor Nodes**:
   - Initial random positioning
   - Response to topology discovery
   - Position updates from robot commands
   - Energy consumption simulation

## Key Parameters

You can modify these parameters in `project-conf.h`:

```c
#define MAX_SENSORS_PER_ROBOT 15    // Robot capacity
#define INITIAL_STOCK_RS 10         // Initial sensors per robot
#define MAX_GRIDS_PER_LA 100        // Maximum grids per location area
#define MAX_LOCATION_AREAS 50       // Maximum location areas
```

## Message Types

The system uses several UDP message types:

- `MSG_ROBOT_REPORT`: Robot coverage reports to BS
- `MSG_ROBOT_DEPLOY`: BS deployment commands to robots
- `MSG_TOPOLOGY_DISCOVERY`: Robot discovery broadcasts
- `MSG_SENSOR_RESPONSE`: Sensor responses to discovery
- `MSG_SENSOR_DEPLOY`: Robot deployment commands to sensors
- `MSG_SENSOR_COLLECT`: Robot collection commands

## Energy Model

The implementation includes energy consumption simulation:

### Sensor Nodes
- **Active Mode**: Higher consumption during sensing and communication
- **Idle Mode**: Lower consumption when inactive
- **Battery Depletion**: Sensors deactivate when battery is exhausted

### Mobile Robots
- **Baseline Energy**: Operational overhead
- **Radio Energy**: Communication costs
- **Mobility Energy**: Movement simulation (coefficient-based)

## Performance Metrics

The system tracks and reports:

- **Area Coverage Percentage**: (Covered grids / Total grids) × 100
- **Deployment Time**: Time to complete sensor deployment
- **Energy Consumption**: Per-node and total system energy usage
- **Grid Coverage Status**: Detailed per-LA coverage information

## Research Implementation Notes

This implementation realizes the theoretical APP_I approach from the research paper:

### Database Structures Implemented
- **LA_DB**: Location area database with coverage tracking
- **Robot_DB**: Robot assignment database  
- **Grid_DB**: Grid status database per robot
- **Sensor_DB**: Sensor information database per robot

### Algorithm Implementation
- **Algorithm 1**: Global phase coordination
- **Algorithm 2**: Local phase execution
- **Algorithm 3**: Topology discovery protocol
- **Algorithm 4**: Dispersion phase with 4 cases

### Message Format Compliance
All message formats follow the paper specification with appropriate bit-level sizing and structure.

## Future Extensions

This implementation provides a foundation for:

- **APP_II**: Non-deterministic heuristic approach
- **Advanced Energy Models**: More detailed energy consumption
- **Real Robot Integration**: Interface with actual mobile robots
- **Dynamic Scenarios**: Sensor failures and environmental changes
- **Performance Optimization**: Algorithm improvements and variants

## Troubleshooting

### Common Issues

1. **Compilation Errors**: Ensure Contiki-NG environment is properly set up
2. **Simulation Issues**: Check node IDs and message routing
3. **Coverage Problems**: Verify perception ranges and grid calculations
4. **Communication Failures**: Check UDP port configuration and addressing

### Debug Tips

- Enable detailed logging by setting `LOG_LEVEL_DEBUG` in source files
- Use Cooja's Timeline plugin to visualize message exchanges
- Monitor the Log Listener for detailed execution traces
- Check the Network visualization for connectivity issues

## References

Based on the research paper describing deterministic grid-based deployment approaches for disaster WSN scenarios with mobile robot assistance for optimal area coverage and energy efficiency. 