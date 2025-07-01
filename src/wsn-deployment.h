#ifndef WSN_DEPLOYMENT_H
#define WSN_DEPLOYMENT_H

#include "contiki.h"
#include "net/ipv6/simple-udp.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Data structure definitions based on the algorithm */

/* Location Area Database Entry */
typedef struct {
    uint8_t la_id;
    int16_t center_x;
    int16_t center_y;
    uint8_t no_grid;
} la_db_entry_t;

/* Robot Database Entry */
typedef struct {
    uint8_t robot_id;
    uint8_t assigned_la_id;
} robot_db_entry_t;

/* Grid Database Entry */
typedef struct {
    uint8_t grid_id;
    int16_t center_x;
    int16_t center_y;
    uint8_t grid_status; /* 0 = uncovered, 1 = covered */
} grid_db_entry_t;

/* Sensor Database Entry */
typedef struct {
    uint8_t sensor_id;
    int16_t x_coord;
    int16_t y_coord;
    uint8_t sensor_status; /* 0 = idle, 1 = active */
} sensor_db_entry_t;

/* Position structure */
typedef struct {
    int16_t x;
    int16_t y;
} position_t;

/* Message structures */

/* Robot broadcast message (Mp) */
typedef struct {
    uint8_t msg_type;
    uint8_t robot_id;
} robot_broadcast_msg_t;

/* Sensor reply message (Sensor_M) */
typedef struct {
    uint8_t msg_type;
    uint8_t sensor_id;
    int16_t x_coord;
    int16_t y_coord;
    uint8_t sensor_status;
} sensor_reply_msg_t;

/* Robot report message (Robot_pM) */
typedef struct {
    uint8_t msg_type;
    uint8_t robot_id;
    uint8_t covered_grids;
} robot_report_msg_t;

/* LA Assignment message */
typedef struct {
    uint8_t msg_type;
    uint8_t robot_id;
    uint8_t la_id;
    int16_t center_x;
    int16_t center_y;
} la_assignment_msg_t;

/* Message type constants */
#define WSN_MSG_TYPE_ROBOT_BROADCAST 1
#define WSN_MSG_TYPE_SENSOR_REPLY 2
#define WSN_MSG_TYPE_ROBOT_REPORT 3
#define WSN_MSG_TYPE_LA_ASSIGNMENT 4
#define WSN_MSG_TYPE_ACK 5

/* Configuration constants from your algorithm */
#define TARGET_AREA_SIZE 1000        /* Target area size in meters */
#define ROBOT_PERCEPTION_RANGE 200   /* Robot perception range */
#define SENSOR_PERCEPTION_RANGE 50   /* Sensor perception range */
#define ROBOT_CAPACITY 15            /* Maximum sensors robot can carry */
#define INITIAL_STOCK 10             /* Initial sensors given to robot */
#define MAX_ROBOTS 2                 /* Number of mobile robots */

/* Calculate derived constants - explicitly using floor as per paper */
/* NO_LA = floor(Size of target area / Perception range of robot) */
#define NO_LA ((int)(TARGET_AREA_SIZE / ROBOT_PERCEPTION_RANGE))
/* NO_G = floor(Perception range of robot / Perception range of sensor) */
#define NO_G ((int)(ROBOT_PERCEPTION_RANGE / SENSOR_PERCEPTION_RANGE))

/* Network ports */
#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

/* Function prototypes */
void wsn_deployment_init(void);
double calculate_distance(position_t pos1, position_t pos2);
void update_grid_status(grid_db_entry_t *grid_db, uint8_t grid_id, uint8_t status);

/* Energy calculation functions */
double calculate_baseline_energy(double time_duration, double power_baseline);
double calculate_sensing_energy(double sensing_range);
double calculate_processing_energy(double power_processing, double time_processing);
double calculate_radio_energy(double power_transmit, double time_transmit, 
                             double power_receive, double time_receive);
double calculate_mobility_energy(double distance);

#endif /* WSN_DEPLOYMENT_H */
