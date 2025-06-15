#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/* Network Configuration */
#define UDP_SERVER_PORT 5678
#define UDP_CLIENT_PORT 8765

/* Base Station Configuration */
#define MAX_LOCATION_AREAS 20
#define MAX_ROBOTS 2
#define MAX_SENSORS_PER_AREA 50
#define ROBOT_STOCK_CAPACITY 15
#define ROBOT_INITIAL_STOCK 10

/* Energy Model Parameters */
/* Base Station Energy Parameters */
#define P_PROCESSING_BASE 0.025      // Processing power in Watts
#define P_TRANSMIT_BASE 0.050        // Transmit power in Watts  
#define P_RECEIVE_BASE 0.040         // Receive power in Watts
#define T_PROCESSING_BASE 0.001      // Processing time per operation in seconds

/* Robot Energy Parameters */
#define P_BASELINE_ROBOT 0.030       // Baseline power consumption in Watts
#define P_TRANSMIT_ROBOT 0.040       // Robot transmit power in Watts
#define P_RECEIVE_ROBOT 0.035        // Robot receive power in Watts
#define TAU_MOBILITY 0.0005          // Energy coefficient for robot movement

/* Sensor Energy Parameters */
#define P_BASELINE_SENSOR 0.020      // Sensor baseline power in Watts
#define P_TRANSMIT_SENSOR 0.030      // Sensor transmit power in Watts
#define P_RECEIVE_SENSOR 0.025       // Sensor receive power in Watts
#define P_PROCESSING_SENSOR 0.015    // Sensor processing power in Watts
#define MU_SENSING 0.0005           // Energy coefficient for sensing field

/* Target Area Configuration */
#define TARGET_AREA_WIDTH 1000       // Target area width in meters
#define TARGET_AREA_HEIGHT 1000      // Target area height in meters
#define ROBOT_PERCEPTION_RANGE 100   // Robot perception range in meters
#define SENSOR_PERCEPTION_RANGE 50   // Sensor perception range in meters

/* Communication Configuration */
#define MESSAGE_SEND_INTERVAL (30 * CLOCK_SECOND)
#define ENERGY_REPORT_INTERVAL (60 * CLOCK_SECOND)

/* Logging */
#define LOG_LEVEL_APP LOG_LEVEL_INFO

#endif /* PROJECT_CONF_H_ */
