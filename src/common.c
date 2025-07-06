/*
 * WSN Deployment Common Functions - Contiki-NG Implementation
 */

#include "common.h"

// Global system parameters (can be overridden by project-conf.h)
#ifndef WSN_TARGET_AREA_SIZE
float target_area_size = 1000.0;
#else
float target_area_size = WSN_TARGET_AREA_SIZE;
#endif

#ifndef WSN_ROBOT_PERCEPTION_RANGE
float robot_perception_range = 100.0;
#else
float robot_perception_range = WSN_ROBOT_PERCEPTION_RANGE;
#endif

#ifndef WSN_SENSOR_PERCEPTION_RANGE
float sensor_perception_range = 50.0;
#else
float sensor_perception_range = WSN_SENSOR_PERCEPTION_RANGE;
#endif

#ifndef WSN_MAX_SENSORS
uint32_t total_sensors = 100;
#else
uint32_t total_sensors = WSN_MAX_SENSORS;
#endif

// Database size calculation functions
uint32_t calculate_la_db_size(uint32_t no_la, uint32_t no_g) {
    uint32_t la_id_bits = (uint32_t)ceil(log2f((float)no_la));
    uint32_t no_grid_bits = (uint32_t)ceil(log2f((float)no_g));
    return no_la * (la_id_bits + COORD_SIZE + no_grid_bits);
}

uint32_t calculate_robot_db_size(uint32_t no_la) {
    uint32_t la_id_bits = (uint32_t)ceil(log2f((float)no_la));
    return MAX_ROBOTS * (1 + la_id_bits);
}

uint32_t calculate_grid_db_size(uint32_t no_g) {
    uint32_t grid_id_bits = (uint32_t)ceil(log2f((float)no_g));
    return no_g * (grid_id_bits + 17);  // 16 bits coord + 1 bit status
}

uint32_t calculate_sensor_db_size(uint32_t ns) {
    uint32_t sensor_id_bits = (uint32_t)ceil(log2f((float)ns));
    return ns * (sensor_id_bits + 17);  // 16 bits coord + 1 bit status
}

// Calculate Euclidean distance between two coordinates
float calculate_distance(coordinate_t pos1, coordinate_t pos2) {
    float dx = pos1.x - pos2.x;
    float dy = pos1.y - pos2.y;
    return sqrtf(dx * dx + dy * dy);
}

// Network utility functions
void create_network_message(network_message_t *msg, message_type_t type, 
                           const void *data, uint16_t data_len) {
    msg->type = type;
    msg->length = data_len;
    if (data && data_len <= sizeof(msg->data)) {
        memcpy(msg->data, data, data_len);
    }
}

// Coordinate utility functions
coordinate_t create_coordinate(float x, float y) {
    coordinate_t coord = {x, y};
    return coord;
}

int coordinates_equal(coordinate_t c1, coordinate_t c2) {
    const float epsilon = 0.001f;
    return (fabsf(c1.x - c2.x) < epsilon) && (fabsf(c1.y - c2.y) < epsilon);
}

// Grid utility functions
uint32_t coordinate_to_grid_id(coordinate_t coord, coordinate_t la_center, 
                              uint32_t grids_per_row, float grid_size) {
    float relative_x = coord.x - (la_center.x - robot_perception_range / 2);
    float relative_y = coord.y - (la_center.y - robot_perception_range / 2);
    
    uint32_t grid_x = (uint32_t)(relative_x / grid_size);
    uint32_t grid_y = (uint32_t)(relative_y / grid_size);
    
    if (grid_x >= grids_per_row) grid_x = grids_per_row - 1;
    if (grid_y >= grids_per_row) grid_y = grids_per_row - 1;
    
    return grid_y * grids_per_row + grid_x;
}

coordinate_t grid_id_to_coordinate(uint32_t grid_id, coordinate_t la_center,
                                  uint32_t grids_per_row, float grid_size) {
    uint32_t grid_x = grid_id % grids_per_row;
    uint32_t grid_y = grid_id / grids_per_row;
    
    coordinate_t coord;
    coord.x = (la_center.x - robot_perception_range / 2) + (grid_x + 0.5f) * grid_size;
    coord.y = (la_center.y - robot_perception_range / 2) + (grid_y + 0.5f) * grid_size;
    
    return coord;
}

// Random position generation
coordinate_t generate_random_position_in_area(float area_size) {
    coordinate_t pos;
    pos.x = ((float)(random_rand() % 10000) / 10000.0f) * area_size;
    pos.y = ((float)(random_rand() % 10000) / 10000.0f) * area_size;
    return pos;
}

coordinate_t generate_random_position_in_la(coordinate_t la_center) {
    coordinate_t pos;
    float half_range = robot_perception_range / 2.0f;
    
    pos.x = la_center.x + (((float)(random_rand() % 10000) / 10000.0f) - 0.5f) * robot_perception_range;
    pos.y = la_center.y + (((float)(random_rand() % 10000) / 10000.0f) - 0.5f) * robot_perception_range;
    
    // Ensure position is within bounds
    if (pos.x < 0) pos.x = 0;
    if (pos.y < 0) pos.y = 0;
    if (pos.x > target_area_size) pos.x = target_area_size;
    if (pos.y > target_area_size) pos.y = target_area_size;
    
    return pos;
}

// Coverage calculation functions
float calculate_grid_coverage_percentage(grid_db_record_t *grid_db, uint32_t num_grids) {
    uint32_t covered_grids = 0;
    
    for (uint32_t i = 0; i < num_grids; i++) {
        if (grid_db[i].grid_status == COVERED) {
            covered_grids++;
        }
    }
    
    return (float)covered_grids / num_grids * 100.0f;
}

uint32_t count_covered_grids(grid_db_record_t *grid_db, uint32_t num_grids) {
    uint32_t covered_grids = 0;
    
    for (uint32_t i = 0; i < num_grids; i++) {
        if (grid_db[i].grid_status == COVERED) {
            covered_grids++;
        }
    }
    
    return covered_grids;
}

uint32_t count_active_sensors(sensor_db_record_t *sensor_db, uint32_t num_sensors) {
    uint32_t active_sensors = 0;
    
    for (uint32_t i = 0; i < num_sensors; i++) {
        if (sensor_db[i].sensor_status == ACTIVE) {
            active_sensors++;
        }
    }
    
    return active_sensors;
}

// Message size calculation functions (from LaTeX specifications)
uint32_t calculate_robot_pm_message_size(uint32_t no_g) {
    return 1 + (uint32_t)ceil(log2f((float)no_g));  // 1 bit robot_id + log2(NO_G) bits for coverage
}

uint32_t calculate_mp_message_size() {
    return 1;  // 1 bit for robot_id
}

uint32_t calculate_sensor_m_message_size(uint32_t total_sensors) {
    return (uint32_t)ceil(log2f((float)total_sensors)) + 17;  // sensor_id + 16 bits coord + 1 bit status
}

// Debugging and logging utilities
void print_coordinate(coordinate_t coord, const char *label) {
    printf("%s: (%.2f, %.2f)\n", label ? label : "Coordinate", coord.x, coord.y);
}

void print_grid_status(grid_db_record_t *grid_db, uint32_t num_grids) {
    printf("Grid Status Summary:\n");
    printf("Grid_ID\tCenter_X\tCenter_Y\tStatus\n");
    
    for (uint32_t i = 0; i < num_grids; i++) {
        printf("%u\t%.2f\t\t%.2f\t\t%s\n",
               grid_db[i].grid_id,
               grid_db[i].center_coord.x,
               grid_db[i].center_coord.y,
               grid_db[i].grid_status == COVERED ? "COVERED" : "UNCOVERED");
    }
}

void print_sensor_status(sensor_db_record_t *sensor_db, uint32_t num_sensors) {
    printf("Sensor Status Summary:\n");
    printf("Sensor_ID\tPosition_X\tPosition_Y\tStatus\n");
    
    for (uint32_t i = 0; i < num_sensors; i++) {
        printf("%u\t\t%.2f\t\t%.2f\t\t%s\n",
               sensor_db[i].sensor_id,
               sensor_db[i].coord.x,
               sensor_db[i].coord.y,
               sensor_db[i].sensor_status == ACTIVE ? "ACTIVE" : "IDLE");
    }
}

// Memory management utilities for Contiki-NG
void* wsn_malloc(size_t size) {
    // In Contiki-NG, we typically use static allocation or memb
    // For simulation purposes, we can use malloc if available
    #ifdef CONTIKI
    // Use Contiki's memory management
    return malloc(size);
    #else
    return malloc(size);
    #endif
}

void wsn_free(void* ptr) {
    #ifdef CONTIKI
    free(ptr);
    #else
    free(ptr);
    #endif
}

// System status functions
void print_system_parameters() {
    printf("=== WSN Deployment System Parameters ===\n");
    printf("Target area size: %.2f x %.2f\n", target_area_size, target_area_size);
    printf("Robot perception range: %.2f\n", robot_perception_range);
    printf("Sensor perception range: %.2f\n", sensor_perception_range);
    printf("Total sensors: %u\n", total_sensors);
    printf("Max robots: %u\n", MAX_ROBOTS);
    printf("Robot capacity: %u sensors\n", ROBOT_CAPACITY);
    printf("Initial stock per robot: %u sensors\n", STOCK_RS);
    
    // Calculate derived parameters
    uint32_t no_la = (uint32_t)floor(target_area_size / robot_perception_range);
    uint32_t no_g = (uint32_t)floor(robot_perception_range / sensor_perception_range);
    
    printf("Number of Location Areas (NO_LA): %u\n", no_la);
    printf("Number of Grids per LA (NO_G): %u\n", no_g);
    
    // Calculate message sizes
    printf("\n=== Message Sizes (bits) ===\n");
    printf("Robot_pM message: %u bits\n", calculate_robot_pm_message_size(no_g));
    printf("Mp broadcast message: %u bits\n", calculate_mp_message_size());
    printf("Sensor_M reply message: %u bits\n", calculate_sensor_m_message_size(total_sensors));
    
    // Calculate database sizes
    printf("\n=== Database Sizes (bits) ===\n");
    printf("LA_DB size: %u bits\n", calculate_la_db_size(no_la, no_g));
    printf("Robot_DB size: %u bits\n", calculate_robot_db_size(no_la));
    printf("Grid_DB size: %u bits\n", calculate_grid_db_size(no_g));
    printf("Sensor_DB size: %u bits\n", calculate_sensor_db_size(total_sensors));
}
