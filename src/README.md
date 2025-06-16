# WSN Deployment Implementation for Disaster Areas

This project implements the APP_I algorithm described in the research paper for Wireless Sensor Network (WSN) deployment in disaster-hit areas. The implementation focuses on maximizing area coverage while minimizing energy consumption through optimized sensor deployment.

## Components

The system consists of three main node types:

1. **Base Station** (`base-station.c`):
   - Implements the global phase of APP_I
   - Divides target area into Location Areas (LAs)
   - Manages robot assignments to LAs
   - Tracks deployment statistics and energy consumption

2. **Mobile Robots** (`mobile-robot.c`):
   - Implement the local phase of APP_I
   - Divide assigned LAs into grids
   - Execute topology discovery to find sensors
   - Perform dispersion of sensors using the 4 cases algorithm
   - Track robot energy consumption

3. **Sensor Nodes** (`sensor-node.c`):
   - Respond to robot discovery messages
   - Maintain active/idle modes
   - Track sensor energy consumption
   - Simulate sensing operations

## Energy Model

The implementation follows the energy model from the research paper:
- **Baseline Energy**: `E_baseline = (t_i - t_s) * P_baseline`
- **Sensing Energy**: `E_sensing = μ * r_i^2` 
- **Processing Energy**: `E_processing = P_processing * t_processing`
- **Radio Energy**: `E_radio = E_transmit + E_receive`
- **Mobility Energy**: `E_mobility = τ * D_p`

Total energy is calculated as:
`Energy_Tot = E_active + E_idle + E_robot + E_base_station`

## How to Build & Run

The project is built for Contiki-NG, specifically for the Cooja simulator.

### Prerequisites
- Contiki-NG installed at the path specified in the Makefile

### Building
```bash
cd src/
make
```

### Running in Cooja
1. Start Cooja simulator
2. Create a new simulation
3. Add motes using the compiled files:
   - Add a base station (base-station.cooja)
   - Add 2 robots (mobile-robot.cooja)
   - Add several sensors (sensor-node.cooja)
4. Start the simulation

Or load the provided simulation:
```bash
cooja -quickstart=wsn-deployment-simulation.csc
```

## Algorithm Flow

1. Base station divides area into LAs and assigns robots
2. Robots execute local phase in assigned LAs:
   - Topology discovery to find sensors
   - Grid creation based on NO_G formula
   - Dispersion phase with the 4 cases:
     - Case 1: Robot has sensors AND grid has sensors
     - Case 2: Robot has sensors BUT grid has no sensors
     - Case 3: Robot has no sensors BUT grid has sensors
     - Case 4: Robot has no sensors AND grid has no sensors
3. Robots report covered grids back to base station
4. Base station reassigns robots to new LAs until all are covered

## Configuration Parameters

The main parameters can be adjusted in `wsn-deployment.h`:
- `TARGET_AREA_SIZE`: Size of the deployment area
- `ROBOT_PERCEPTION_RANGE`: Robot's perception range
- `SENSOR_PERCEPTION_RANGE`: Sensor node's perception range
- `ROBOT_CAPACITY`: Maximum sensors a robot can carry
- `INITIAL_STOCK`: Initial sensors given to robot

## Energy Tracking

The system tracks energy consumption for all components:
- Base station processing and radio energy
- Robot baseline, radio, and mobility energy
- Sensor nodes baseline, sensing, processing, and radio energy

Detailed energy statistics are printed periodically during simulation. 