#include "common.h"
#include <string.h>

// Global array to simulate deployed sensors in the environment
static sensor_db_record_t deployed_sensors[1000];
static uint32_t deployed_sensor_count = 0;

// Initialize mobile robot
void initialize_mobile_robot(mobile_robot_t *robot, robot_id_t id, uint32_t la_id, coordinate_t la_center) {
    robot->robot_id = id;
    robot->assigned_la_id = la_id;
    robot->la_center = la_center;
    robot->current_position = la_center;
    robot->stock_rs = STOCK_RS;  // Initially 10 sensors as per specification
    robot->no_g = (uint32_t)floor(robot_perception_range / sensor_perception_range);
    robot->no_p = robot->no_g;  // Initially NO_P = NO_G
    
    // Allocate memory for local databases
    robot->grid_db = (grid_db_record_t*)calloc(robot->no_g, sizeof(grid_db_record_t));
    robot->sensor_db = (sensor_db_record_t*)calloc(robot->no_g * 4, sizeof(sensor_db_record_t)); // Estimate
    robot->sensor_db_count = 0;
    
    printf("Robot_%d initialized in LA_%u at center (%.2f, %.2f)\n", 
           robot->robot_id + 1, robot->assigned_la_id, 
           robot->la_center.x, robot->la_center.y);
    printf("Robot_%d stock: %u sensors, NO_G: %u, NO_P: %u\n", 
           robot->robot_id + 1, robot->stock_rs, robot->no_g, robot->no_p);
}

// Execute complete local phase
void execute_local_phase(mobile_robot_t *robot) {
    printf("\n=== Robot_%d Starting Local Phase in LA_%u ===\n", 
           robot->robot_id + 1, robot->assigned_la_id);
    
    // Algorithm 2: Local phase steps
    
    // Step 1: Divide LA_q into NO_G grids
    divide_la_into_grids(robot);
    
    // Step 2: Insert records into Grid_DB (done in divide_la_into_grids)
    
    // Step 3: Topology Discovery Phase
    topology_discovery_phase(robot);
    
    // Step 4: Initialize NO_P = NO_G (already done in initialization)
    robot->no_p = robot->no_g;
    
    // Step 5: Dispersion Phase
    dispersion_phase(robot);
    
    // Step 6-8: Count covered grids and send message to BS
    uint32_t covered_grids = 0;
    for (uint32_t i = 0; i < robot->no_g; i++) {
        if (robot->grid_db[i].grid_status == COVERED) {
            covered_grids++;
        }
    }
    
    printf("Robot_%d completed local phase. Covered grids: %u/%u\n", 
           robot->robot_id + 1, covered_grids, robot->no_g);
    
    // Create and send completion message to BS
    robot_message_t completion_msg = create_completion_message(robot);
    printf("Robot_%d sending completion message to BS: (%u, %u)\n", 
           robot->robot_id + 1, completion_msg.robot_id, completion_msg.no_grid);
}

// Divide assigned LA into grids
void divide_la_into_grids(mobile_robot_t *robot) {
    printf("Robot_%d dividing LA_%u into %u grids\n", 
           robot->robot_id + 1, robot->assigned_la_id, robot->no_g);
    
    float grid_size = sensor_perception_range;
    float start_x = robot->la_center.x - (robot_perception_range / 2) + (grid_size / 2);
    float start_y = robot->la_center.y - (robot_perception_range / 2) + (grid_size / 2);
    
    uint32_t grids_per_row = (uint32_t)sqrt(robot->no_g);
    
    for (uint32_t i = 0; i < robot->no_g; i++) {
        robot->grid_db[i].grid_id = i + 1;
        
        uint32_t row = i / grids_per_row;
        uint32_t col = i % grids_per_row;
        
        robot->grid_db[i].center_coord.x = start_x + col * grid_size;
        robot->grid_db[i].center_coord.y = start_y + row * grid_size;
        robot->grid_db[i].grid_status = UNCOVERED;  // Initially uncovered
        
        printf("Grid_%u: Center(%.2f, %.2f), Status: %s\n", 
               robot->grid_db[i].grid_id,
               robot->grid_db[i].center_coord.x,
               robot->grid_db[i].center_coord.y,
               robot->grid_db[i].grid_status == COVERED ? "COVERED" : "UNCOVERED");
    }
}

// Algorithm 3: Topology Discovery Phase
void topology_discovery_phase(mobile_robot_t *robot) {
    printf("\n--- Robot_%d Topology Discovery Phase ---\n", robot->robot_id + 1);
    
    // Step 1: Move Robot_p to centre of LA_q
    robot->current_position = robot->la_center;
    printf("Robot_%d moved to center of LA_%u at (%.2f, %.2f)\n", 
           robot->robot_id + 1, robot->assigned_la_id, 
           robot->current_position.x, robot->current_position.y);
    
    // Step 2: Broadcast message (Mp) to discover sensors
    broadcast_robot_message(robot);
    
    // Step 3: Receive responses from sensors within perception range
    uint32_t reply_count = 0;
    sensor_reply_message_t *replies = receive_sensor_replies(robot, &reply_count);
    
    // Step 4: Update Sensor_DB with Sensor_M information
    robot->sensor_db_count = 0;
    for (uint32_t i = 0; i < reply_count; i++) {
        robot->sensor_db[robot->sensor_db_count] = (sensor_db_record_t){
            .sensor_id = replies[i].sensor_id,
            .coord = replies[i].coord,
            .sensor_status = IDLE  // Mark as idle initially
        };
        robot->sensor_db_count++;
        
        printf("Discovered Sensor_%u at (%.2f, %.2f), Status: IDLE\n", 
               replies[i].sensor_id, replies[i].coord.x, replies[i].coord.y);
    }
    
    printf("Topology discovery completed. Found %u sensors in perception range.\n", 
           robot->sensor_db_count);
    
    if (replies) free(replies);
}

// Broadcast robot message for topology discovery
void broadcast_robot_message(mobile_robot_t *robot) {
    robot_broadcast_message_t broadcast = {robot->robot_id};
    printf("Robot_%d broadcasting discovery message Mp(%u)\n", 
           robot->robot_id + 1, broadcast.robot_id);
}

// Simulate receiving sensor replies
sensor_reply_message_t* receive_sensor_replies(mobile_robot_t *robot, uint32_t *reply_count) {
    // Simulate randomly deployed sensors in the LA
    // In real implementation, this would be actual network communication
    
    uint32_t estimated_sensors = robot->no_g * 2;  // Estimate 2 sensors per grid
    sensor_reply_message_t *replies = malloc(estimated_sensors * sizeof(sensor_reply_message_t));
    *reply_count = 0;
    
    // Generate random sensors within robot's perception range
    srand(robot->robot_id + robot->assigned_la_id);  // Deterministic for testing
    
    for (uint32_t i = 0; i < estimated_sensors; i++) {
        coordinate_t sensor_pos;
        sensor_pos.x = robot->current_position.x + 
                      ((float)rand() / RAND_MAX - 0.5) * robot_perception_range;
        sensor_pos.y = robot->current_position.y + 
                      ((float)rand() / RAND_MAX - 0.5) * robot_perception_range;
        
        // Check if sensor is within perception range
        float distance = calculate_distance(robot->current_position, sensor_pos);
        if (distance <= robot_perception_range / 2) {
            replies[*reply_count] = (sensor_reply_message_t){
                .sensor_id = robot->assigned_la_id * 100 + i + 1,
                .coord = sensor_pos,
                .sensor_status = IDLE
            };
            (*reply_count)++;
        }
    }
    
    return replies;
}

// Algorithm 4: Dispersion Phase
void dispersion_phase(mobile_robot_t *robot) {
    printf("\n--- Robot_%d Dispersion Phase ---\n", robot->robot_id + 1);
    
    uint32_t current_grid = 0;
    
    // Algorithm 4: while NO_P > 0 do
    while (robot->no_p > 0) {
        printf("\nProcessing Grid_%u (NO_P remaining: %u)\n", 
               current_grid + 1, robot->no_p);
        
        // Count sensors in current grid
        uint32_t sensors_in_grid = 0;
        for (uint32_t i = 0; i < robot->sensor_db_count; i++) {
            float distance = calculate_distance(robot->grid_db[current_grid].center_coord, 
                                              robot->sensor_db[i].coord);
            if (distance <= sensor_perception_range / 2) {
                sensors_in_grid++;
            }
        }
        
        // Determine which case applies
        if (robot->stock_rs > 0 && sensors_in_grid > 0) {
            // Case 1: Robot has sensors in Stock_RS and G_i has sensors
            handle_dispersion_case_1(robot, current_grid);
        } else if (robot->stock_rs > 0 && sensors_in_grid == 0) {
            // Case 2: Robot has sensors in Stock_RS but G_i has no sensors
            handle_dispersion_case_2(robot, current_grid);
        } else if (robot->stock_rs == 0 && sensors_in_grid > 0) {
            // Case 3: Robot has no sensors in Stock_RS but G_i has sensors
            handle_dispersion_case_3(robot, current_grid);
        } else {
            // Case 4: Robot has no sensors in Stock_RS and G_i has no sensors
            handle_dispersion_case_4(robot, current_grid);
        }
        
        // Decrement NO_P by 1
        robot->no_p--;
        
        // Find the nearest uncovered grid
        current_grid = find_nearest_uncovered_grid(robot, current_grid);
        if (current_grid >= robot->no_g) {
            break;  // No more uncovered grids
        }
    }
    
    printf("Dispersion phase completed. NO_P = %u\n", robot->no_p);
}

// Case 1: Robot has sensors in Stock_RS and G_i has sensors
void handle_dispersion_case_1(mobile_robot_t *robot, uint32_t grid_id) {
    printf("Executing Case 1 for Grid_%u\n", grid_id + 1);
    
    // Place a sensor from Stock_RS at the centre of G_i
    uint32_t new_sensor_id = robot->assigned_la_id * 1000 + grid_id + 1;
    
    // Add new sensor to sensor_DB
    robot->sensor_db[robot->sensor_db_count] = (sensor_db_record_t){
        .sensor_id = new_sensor_id,
        .coord = robot->grid_db[grid_id].center_coord,
        .sensor_status = ACTIVE
    };
    robot->sensor_db_count++;
    robot->stock_rs--;
    
    printf("Placed Sensor_%u at grid center (%.2f, %.2f), Status: ACTIVE\n", 
           new_sensor_id, robot->grid_db[grid_id].center_coord.x, 
           robot->grid_db[grid_id].center_coord.y);
    
    // Collect extra sensors from G_i until Stock_RS capacity is reached
    uint32_t collected = 0;
    for (uint32_t i = 0; i < robot->sensor_db_count && robot->stock_rs < ROBOT_CAPACITY; i++) {
        if (robot->sensor_db[i].sensor_id != new_sensor_id) {
            float distance = calculate_distance(robot->grid_db[grid_id].center_coord, 
                                              robot->sensor_db[i].coord);
            if (distance <= sensor_perception_range / 2 && robot->sensor_db[i].sensor_status == IDLE) {
                // Collect this sensor
                robot->stock_rs++;
                collected++;
                
                // Remove from sensor_DB by marking as collected
                robot->sensor_db[i].sensor_status = ACTIVE;  // Mark as removed
                printf("Collected Sensor_%u from grid\n", robot->sensor_db[i].sensor_id);
            }
        }
    }
    
    // Mark G_i as covered
    robot->grid_db[grid_id].grid_status = COVERED;
    printf("Grid_%u marked as COVERED\n", grid_id + 1);
}

// Case 2: Robot has sensors in Stock_RS but G_i has no sensors
void handle_dispersion_case_2(mobile_robot_t *robot, uint32_t grid_id) {
    printf("Executing Case 2 for Grid_%u\n", grid_id + 1);
    
    // Place a sensor from Stock_RS at the centre of G_i
    uint32_t new_sensor_id = robot->assigned_la_id * 1000 + grid_id + 1;
    
    // Add new sensor to sensor_DB
    robot->sensor_db[robot->sensor_db_count] = (sensor_db_record_t){
        .sensor_id = new_sensor_id,
        .coord = robot->grid_db[grid_id].center_coord,
        .sensor_status = ACTIVE
    };
    robot->sensor_db_count++;
    robot->stock_rs--;
    
    printf("Placed Sensor_%u at grid center (%.2f, %.2f), Status: ACTIVE\n", 
           new_sensor_id, robot->grid_db[grid_id].center_coord.x, 
           robot->grid_db[grid_id].center_coord.y);
    
    // Mark G_i as covered
    robot->grid_db[grid_id].grid_status = COVERED;
    printf("Grid_%u marked as COVERED\n", grid_id + 1);
}

// Case 3: Robot has no sensors in Stock_RS but G_i has sensors
void handle_dispersion_case_3(mobile_robot_t *robot, uint32_t grid_id) {
    printf("Executing Case 3 for Grid_%u\n", grid_id + 1);
    
    // Find sensor with minimum distance to grid center
    uint32_t closest_sensor_idx = UINT32_MAX;
    float min_distance = INFINITY;
    
    for (uint32_t i = 0; i < robot->sensor_db_count; i++) {
        if (robot->sensor_db[i].sensor_status == IDLE) {
            float distance = calculate_distance(robot->grid_db[grid_id].center_coord, 
                                              robot->sensor_db[i].coord);
            if (distance <= sensor_perception_range / 2 && distance < min_distance) {
                min_distance = distance;
                closest_sensor_idx = i;
            }
        }
    }
    
    if (closest_sensor_idx != UINT32_MAX) {
        // Move closest sensor to grid center
        robot->sensor_db[closest_sensor_idx].coord = robot->grid_db[grid_id].center_coord;
        robot->sensor_db[closest_sensor_idx].sensor_status = ACTIVE;
        
        printf("Moved Sensor_%u to grid center (%.2f, %.2f), Status: ACTIVE\n", 
               robot->sensor_db[closest_sensor_idx].sensor_id,
               robot->grid_db[grid_id].center_coord.x, 
               robot->grid_db[grid_id].center_coord.y);
        
        // Collect extra sensors from G_i until Stock_RS capacity is reached
        for (uint32_t i = 0; i < robot->sensor_db_count && robot->stock_rs < ROBOT_CAPACITY; i++) {
            if (i != closest_sensor_idx && robot->sensor_db[i].sensor_status == IDLE) {
                float distance = calculate_distance(robot->grid_db[grid_id].center_coord, 
                                                  robot->sensor_db[i].coord);
                if (distance <= sensor_perception_range / 2) {
                    robot->stock_rs++;
                    robot->sensor_db[i].sensor_status = ACTIVE;  // Mark as collected
                    printf("Collected Sensor_%u from grid\n", robot->sensor_db[i].sensor_id);
                }
            }
        }
        
        // Mark G_i as covered
        robot->grid_db[grid_id].grid_status = COVERED;
        printf("Grid_%u marked as COVERED\n", grid_id + 1);
    }
}

// Case 4: Robot has no sensors in Stock_RS and G_i has no sensors
void handle_dispersion_case_4(mobile_robot_t *robot, uint32_t grid_id) {
    printf("Executing Case 4 for Grid_%u - leaving uncovered\n", grid_id + 1);
    // G_i remains uncovered - no action needed
    robot->grid_db[grid_id].grid_status = UNCOVERED;
}

// Find nearest uncovered grid
uint32_t find_nearest_uncovered_grid(mobile_robot_t *robot, uint32_t current_grid) {
    uint32_t next_grid = current_grid + 1;
    
    // Simple linear search for next uncovered grid
    while (next_grid < robot->no_g && robot->grid_db[next_grid].grid_status == COVERED) {
        next_grid++;
    }
    
    return next_grid;
}

// Create completion message for BS
robot_message_t create_completion_message(mobile_robot_t *robot) {
    uint32_t covered_grids = 0;
    
    // Count covered grids
    for (uint32_t i = 0; i < robot->no_g; i++) {
        if (robot->grid_db[i].grid_status == COVERED) {
            covered_grids++;
        }
    }
    
    robot_message_t msg = {
        .robot_id = robot->robot_id,
        .no_grid = covered_grids
    };
    
    return msg;
}

// Calculate Euclidean distance between two coordinates
float calculate_distance(coordinate_t pos1, coordinate_t pos2) {
    float dx = pos1.x - pos2.x;
    float dy = pos1.y - pos2.y;
    return sqrt(dx * dx + dy * dy);
}

// Cleanup mobile robot resources
void cleanup_mobile_robot(mobile_robot_t *robot) {
    if (robot->grid_db) {
        free(robot->grid_db);
        robot->grid_db = NULL;
    }
    if (robot->sensor_db) {
        free(robot->sensor_db);
        robot->sensor_db = NULL;
    }
    printf("Robot_%d resources cleaned up\n", robot->robot_id + 1);
}

// Main function for testing mobile robot functionality
int main() {
    printf("=== WSN Deployment Mobile Robot Local Phase ===\n");
    
    // Initialize mobile robot
    mobile_robot_t robot;
    coordinate_t la_center = {250.0, 250.0};  // Example LA center
    initialize_mobile_robot(&robot, ROBOT_1, 1, la_center);
    
    // Execute local phase
    execute_local_phase(&robot);
    
    // Cleanup
    cleanup_mobile_robot(&robot);
    
    return 0;
}
