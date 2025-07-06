#include "common.h"
#include <string.h>

// Global system parameters
float target_area_size = 1000.0;  // Example size
float robot_perception_range = 100.0;
float sensor_perception_range = 50.0;
uint32_t total_sensors = 100;

// Base Station Database
typedef struct {
    la_db_record_t *la_db;
    robot_db_record_t *robot_db;
    uint32_t no_la;
    uint32_t no_g;
    uint32_t size_la_db;
    uint32_t size_robot_db;
} base_station_t;

// Global base station instance
static base_station_t bs;

// Database size calculation functions
uint32_t calculate_la_db_size(uint32_t no_la, uint32_t no_g) {
    uint32_t la_id_bits = (uint32_t)ceil(log2(no_la));
    uint32_t no_grid_bits = (uint32_t)ceil(log2(no_g));
    return no_la * (la_id_bits + COORD_SIZE + no_grid_bits);
}

uint32_t calculate_robot_db_size(uint32_t no_la) {
    uint32_t la_id_bits = (uint32_t)ceil(log2(no_la));
    return MAX_ROBOTS * (1 + la_id_bits);
}

uint32_t calculate_grid_db_size(uint32_t no_g) {
    uint32_t grid_id_bits = (uint32_t)ceil(log2(no_g));
    return no_g * (grid_id_bits + 17);  // 16 bits coord + 1 bit status
}

uint32_t calculate_sensor_db_size(uint32_t ns) {
    uint32_t sensor_id_bits = (uint32_t)ceil(log2(ns));
    return ns * (sensor_id_bits + 17);  // 16 bits coord + 1 bit status
}

// Initialize Base Station
void initialize_base_station() {
    printf("Initializing Base Station...\n");
    
    // Calculate number of location areas
    bs.no_la = (uint32_t)floor(target_area_size / robot_perception_range);
    bs.no_g = (uint32_t)floor(robot_perception_range / sensor_perception_range);
    
    printf("Number of Location Areas (NO_LA): %u\n", bs.no_la);
    printf("Number of Grids per LA (NO_G): %u\n", bs.no_g);
    
    // Allocate memory for databases
    bs.la_db = (la_db_record_t*)calloc(bs.no_la, sizeof(la_db_record_t));
    bs.robot_db = (robot_db_record_t*)calloc(MAX_ROBOTS, sizeof(robot_db_record_t));
    
    if (!bs.la_db || !bs.robot_db) {
        printf("Error: Memory allocation failed\n");
        exit(1);
    }
    
    // Calculate database sizes
    bs.size_la_db = calculate_la_db_size(bs.no_la, bs.no_g);
    bs.size_robot_db = calculate_robot_db_size(bs.no_la);
    
    printf("LA_DB size: %u bits\n", bs.size_la_db);
    printf("Robot_DB size: %u bits\n", bs.size_robot_db);
}

// Initialize Location Area Database
void initialize_la_db() {
    printf("Initializing Location Area Database...\n");
    
    float la_size = robot_perception_range;
    float start_x = la_size / 2;
    float start_y = la_size / 2;
    
    for (uint32_t i = 0; i < bs.no_la; i++) {
        bs.la_db[i].la_id = i + 1;
        
        // Calculate center coordinates for each LA
        uint32_t row = i / (uint32_t)sqrt(bs.no_la);
        uint32_t col = i % (uint32_t)sqrt(bs.no_la);
        
        bs.la_db[i].center_coord.x = start_x + col * la_size;
        bs.la_db[i].center_coord.y = start_y + row * la_size;
        bs.la_db[i].no_grid = 0;  // Initially 0 as per specification
        
        printf("LA_%u: Center(%.2f, %.2f), NO_Grid: %u\n", 
               bs.la_db[i].la_id, 
               bs.la_db[i].center_coord.x, 
               bs.la_db[i].center_coord.y, 
               bs.la_db[i].no_grid);
    }
}

// Deploy robots in location areas
void deploy_robots() {
    printf("Deploying robots in location areas...\n");
    
    // Deploy Robot_1 in LA_1 and Robot_2 in LA_NO_LA as per specification
    bs.robot_db[0].robot_id = ROBOT_1;
    bs.robot_db[0].assigned_la_id = 1;  // LA_1
    
    bs.robot_db[1].robot_id = ROBOT_2;
    bs.robot_db[1].assigned_la_id = bs.no_la;  // LA_NO_LA
    
    printf("Robot_1 deployed in LA_%u\n", bs.robot_db[0].assigned_la_id);
    printf("Robot_2 deployed in LA_%u\n", bs.robot_db[1].assigned_la_id);
    
    // Each robot is assigned STOCK_RS sensors
    printf("Each robot assigned %d sensors from stock\n", STOCK_RS);
}

// Process robot message and find next deployment location
uint32_t find_next_deployment_la() {
    uint32_t min_grid_count = UINT32_MAX;
    uint32_t selected_la_id = 0;
    
    // Find LA with minimum covered grids
    for (uint32_t i = 0; i < bs.no_la; i++) {
        if (bs.la_db[i].no_grid < min_grid_count) {
            min_grid_count = bs.la_db[i].no_grid;
            selected_la_id = bs.la_db[i].la_id;
        }
    }
    
    return selected_la_id;
}

// Handle robot message (global phase)
void handle_robot_message(robot_message_t *msg) {
    printf("Received message from Robot_%d: NO_Grid = %u\n", 
           msg->robot_id + 1, msg->no_grid);
    
    // Update LA_DB with received grid count
    for (uint32_t i = 0; i < bs.no_la; i++) {
        if (bs.la_db[i].la_id == bs.robot_db[msg->robot_id].assigned_la_id) {
            bs.la_db[i].no_grid = msg->no_grid;
            printf("Updated LA_%u with NO_Grid = %u\n", 
                   bs.la_db[i].la_id, msg->no_grid);
            break;
        }
    }
    
    // Find next deployment location
    uint32_t next_la_id = find_next_deployment_la();
    
    if (next_la_id > 0) {
        // Deploy robot in the identified LA
        bs.robot_db[msg->robot_id].assigned_la_id = next_la_id;
        printf("Deploying Robot_%d in LA_%u\n", msg->robot_id + 1, next_la_id);
    } else {
        printf("No suitable LA found for Robot_%d deployment\n", msg->robot_id + 1);
        stop_robot_deployment();
    }
}

// Stop robot deployment and compute area coverage
void stop_robot_deployment() {
    printf("Stopping robot deployment...\n");
    
    // Calculate total covered grids
    uint32_t total_covered_grids = 0;
    for (uint32_t i = 0; i < bs.no_la; i++) {
        total_covered_grids += bs.la_db[i].no_grid;
    }
    
    // Calculate percentage area coverage
    uint32_t total_grids = bs.no_g * bs.no_la;
    float per_ac = ((float)total_covered_grids / total_grids) * 100.0;
    
    printf("Total covered grids: %u\n", total_covered_grids);
    printf("Total grids: %u\n", total_grids);
    printf("Percentage Area Coverage (Per_AC): %.2f%%\n", per_ac);
}

// Display database status
void display_database_status() {
    printf("\n=== Base Station Database Status ===\n");
    
    printf("\nLocation Area Database (LA_DB):\n");
    printf("LA_ID\tCenter_X\tCenter_Y\tNO_Grid\n");
    for (uint32_t i = 0; i < bs.no_la; i++) {
        printf("%u\t%.2f\t\t%.2f\t\t%u\n", 
               bs.la_db[i].la_id,
               bs.la_db[i].center_coord.x,
               bs.la_db[i].center_coord.y,
               bs.la_db[i].no_grid);
    }
    
    printf("\nRobot Database (Robot_DB):\n");
    printf("Robot_ID\tAssigned_LA_ID\n");
    for (int i = 0; i < MAX_ROBOTS; i++) {
        printf("R%d\t\t%u\n", 
               bs.robot_db[i].robot_id + 1,
               bs.robot_db[i].assigned_la_id);
    }
}

// Main base station function
int main() {
    printf("=== WSN Deployment Base Station ===\n");
    
    // Initialize base station
    initialize_base_station();
    
    // Initialize Location Area Database
    initialize_la_db();
    
    // Deploy robots
    deploy_robots();
    
    // Display initial status
    display_database_status();
    
    // Simulate robot messages (for demonstration)
    printf("\n=== Simulating Robot Messages ===\n");
    
    robot_message_t msg1 = {ROBOT_1, 15};  // Robot_1 reports 15 covered grids
    handle_robot_message(&msg1);
    
    robot_message_t msg2 = {ROBOT_2, 12};  // Robot_2 reports 12 covered grids
    handle_robot_message(&msg2);
    
    // Display final status
    printf("\n=== Final Status ===\n");
    display_database_status();
    
    // Clean up
    free(bs.la_db);
    free(bs.robot_db);
    
    return 0;
}