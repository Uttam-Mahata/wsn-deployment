#ifndef COMMON_H
#define COMMON_H

// Contiki-NG includes
#include "contiki.h"
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "sys/log.h"
#include "random.h"

// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

// System constants
#define MAX_ROBOTS 2
#define ROBOT_CAPACITY 15
#define STOCK_RS 10
#define COORD_SIZE 16  // bits for X-Y coordinates

// Contiki-NG specific definitions
#define LOG_MODULE "WSN-Deploy"
#define LOG_LEVEL LOG_LEVEL_INFO

// Network configuration
#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

// Message types for network communication
typedef enum {
    MSG_ROBOT_BROADCAST = 1,
    MSG_SENSOR_REPLY = 2,
    MSG_ROBOT_COMPLETION = 3,
    MSG_BASE_COMMAND = 4,
    MSG_DEPLOYMENT_DATA = 5
} message_type_t;

// Network message structure
typedef struct {
    message_type_t type;
    uint16_t length;
    uint8_t data[256];
} network_message_t;

// Robot IDs
typedef enum {
    ROBOT_1 = 0,
    ROBOT_2 = 1
} robot_id_t;

// Grid and sensor status
typedef enum {
    UNCOVERED = 0,
    COVERED = 1
} grid_status_t;

typedef enum {
    IDLE = 0,
    ACTIVE = 1
} sensor_status_t;

// Coordinate structure
typedef struct {
    float x;
    float y;
} coordinate_t;

// Location Area Database record
typedef struct {
    uint32_t la_id;
    coordinate_t center_coord;
    uint32_t no_grid;
} la_db_record_t;

// Robot Database record
typedef struct {
    robot_id_t robot_id;
    uint32_t assigned_la_id;
} robot_db_record_t;

// Grid Database record
typedef struct {
    uint32_t grid_id;
    coordinate_t center_coord;
    grid_status_t grid_status;
} grid_db_record_t;

// Sensor Database record
typedef struct {
    uint32_t sensor_id;
    coordinate_t coord;
    sensor_status_t sensor_status;
} sensor_db_record_t;

// Message structures
typedef struct {
    robot_id_t robot_id;
    uint32_t no_grid;
} robot_message_t;

// Message structures for local phase
typedef struct {
    robot_id_t robot_id;
} robot_broadcast_message_t;

typedef struct {
    uint32_t sensor_id;
    coordinate_t coord;
    sensor_status_t sensor_status;
} sensor_reply_message_t;

// Mobile Robot structure
typedef struct {
    robot_id_t robot_id;
    uint32_t assigned_la_id;
    coordinate_t current_position;
    coordinate_t la_center;
    uint32_t stock_rs;  // Number of sensors in stock (max 15)
    uint32_t no_p;      // Number of permissible moves
    uint32_t no_g;      // Number of grids in assigned LA
    grid_db_record_t *grid_db;    // Local Grid database
    sensor_db_record_t *sensor_db; // Local Sensor database
    uint32_t sensor_db_count;     // Number of sensors in local DB
} mobile_robot_t;

// Global system parameters
extern float target_area_size;
extern float robot_perception_range;
extern float sensor_perception_range;
extern uint32_t total_sensors;

// Database size calculation functions
uint32_t calculate_la_db_size(uint32_t no_la, uint32_t no_g);
uint32_t calculate_robot_db_size(uint32_t no_la);
uint32_t calculate_grid_db_size(uint32_t no_g);
uint32_t calculate_sensor_db_size(uint32_t ns);

// Local phase function declarations
void initialize_mobile_robot(mobile_robot_t *robot, robot_id_t id, uint32_t la_id, coordinate_t la_center);
void execute_local_phase(mobile_robot_t *robot);
void topology_discovery_phase(mobile_robot_t *robot);
void dispersion_phase(mobile_robot_t *robot);
void broadcast_robot_message(mobile_robot_t *robot);
sensor_reply_message_t* receive_sensor_replies(mobile_robot_t *robot, uint32_t *reply_count);
void divide_la_into_grids(mobile_robot_t *robot);
uint32_t find_nearest_uncovered_grid(mobile_robot_t *robot, uint32_t current_grid);
void handle_dispersion_case_1(mobile_robot_t *robot, uint32_t grid_id);
void handle_dispersion_case_2(mobile_robot_t *robot, uint32_t grid_id);
void handle_dispersion_case_3(mobile_robot_t *robot, uint32_t grid_id);
void handle_dispersion_case_4(mobile_robot_t *robot, uint32_t grid_id);
robot_message_t create_completion_message(mobile_robot_t *robot);
float calculate_distance(coordinate_t pos1, coordinate_t pos2);
void cleanup_mobile_robot(mobile_robot_t *robot);

// Sensor node function declarations
void initialize_sensor_node(uint32_t sensor_id, coordinate_t position);
sensor_reply_message_t handle_robot_broadcast(robot_broadcast_message_t *broadcast);
void update_sensor_status(uint32_t sensor_id, sensor_status_t new_status);
void relocate_sensor(uint32_t sensor_id, coordinate_t new_position);

#endif // COMMON_H