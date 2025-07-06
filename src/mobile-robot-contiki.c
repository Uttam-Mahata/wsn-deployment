/*
 * WSN Deployment Mobile Robot - Contiki-NG Implementation
 * Based on main.tex specifications for local phase execution
 */

#include "common.h"

// Mobile Robot state
static mobile_robot_t robot;
static struct simple_udp_connection udp_conn;
static uip_ipaddr_t base_station_addr;
static uint8_t robot_initialized = 0;

// Contiki processes
PROCESS(mobile_robot_process, "WSN Deployment Mobile Robot Process");
PROCESS(local_phase_process, "Mobile Robot Local Phase Process");
AUTOSTART_PROCESSES(&mobile_robot_process, &local_phase_process);

// Network callback for incoming messages
static void udp_rx_callback(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
    network_message_t *msg = (network_message_t *)data;
    
    LOG_INFO("Mobile Robot received message type %d\n", msg->type);
    
    if (msg->type == MSG_BASE_COMMAND) {
        uint32_t new_la_id = *(uint32_t *)msg->data;
        LOG_INFO("Received deployment command for LA_%u\n", new_la_id);
        
        // Update robot assignment
        robot.assigned_la_id = new_la_id;
        
        // Calculate new LA center coordinates
        float la_size = robot_perception_range;
        uint32_t la_index = new_la_id - 1;
        uint32_t no_la = (uint32_t)floor(target_area_size / robot_perception_range);
        uint32_t row = la_index / (uint32_t)sqrt(no_la);
        uint32_t col = la_index % (uint32_t)sqrt(no_la);
        
        robot.la_center.x = (la_size / 2) + col * la_size;
        robot.la_center.y = (la_size / 2) + row * la_size;
        robot.current_position = robot.la_center;
        
        LOG_INFO("Robot repositioned to LA_%u center (%.2f, %.2f)\n", 
                 new_la_id, robot.la_center.x, robot.la_center.y);
        
        // Start new local phase
        process_post(&local_phase_process, PROCESS_EVENT_MSG, NULL);
    }
    
    if (msg->type == MSG_SENSOR_REPLY) {
        sensor_reply_message_t *sensor_reply = (sensor_reply_message_t *)msg->data;
        
        // Add sensor to local database if within perception range
        float distance = calculate_distance(robot.current_position, sensor_reply->coord);
        if (distance <= robot_perception_range && robot.sensor_db_count < robot.no_g * 4) {
            robot.sensor_db[robot.sensor_db_count] = (sensor_db_record_t){
                .sensor_id = sensor_reply->sensor_id,
                .coord = sensor_reply->coord,
                .sensor_status = IDLE  // Mark as idle initially
            };
            robot.sensor_db_count++;
            
            LOG_INFO("Added Sensor_%u to local database at (%.2f, %.2f)\n", 
                     sensor_reply->sensor_id, sensor_reply->coord.x, sensor_reply->coord.y);
        }
    }
}

// Initialize mobile robot
void initialize_mobile_robot_contiki(robot_id_t id, uint32_t la_id) {
    robot.robot_id = id;
    robot.assigned_la_id = la_id;
    robot.stock_rs = STOCK_RS;  // Initially 10 sensors as per specification
    robot.no_g = (uint32_t)floor(robot_perception_range / sensor_perception_range);
    robot.no_p = robot.no_g;  // Initially NO_P = NO_G
    robot.sensor_db_count = 0;
    
    // Calculate LA center coordinates
    float la_size = robot_perception_range;
    uint32_t la_index = la_id - 1;
    uint32_t no_la = (uint32_t)floor(target_area_size / robot_perception_range);
    uint32_t row = la_index / (uint32_t)sqrt(no_la);
    uint32_t col = la_index % (uint32_t)sqrt(no_la);
    
    robot.la_center.x = (la_size / 2) + col * la_size;
    robot.la_center.y = (la_size / 2) + row * la_size;
    robot.current_position = robot.la_center;
    
    // Allocate memory for local databases
    robot.grid_db = (grid_db_record_t*)malloc(robot.no_g * sizeof(grid_db_record_t));
    robot.sensor_db = (sensor_db_record_t*)malloc(robot.no_g * 4 * sizeof(sensor_db_record_t));
    
    robot_initialized = 1;
    
    LOG_INFO("Robot_%d initialized in LA_%u at center (%.2f, %.2f)\n", 
           robot.robot_id + 1, robot.assigned_la_id, 
           robot.la_center.x, robot.la_center.y);
    LOG_INFO("Robot_%d stock: %u sensors, NO_G: %u, NO_P: %u\n", 
           robot.robot_id + 1, robot.stock_rs, robot.no_g, robot.no_p);
}

// Execute complete local phase
void execute_local_phase_contiki() {
    if (!robot_initialized) return;
    
    LOG_INFO("=== Robot_%d Starting Local Phase in LA_%u ===\n", 
           robot.robot_id + 1, robot.assigned_la_id);
    
    // Algorithm 2: Local phase steps
    
    // Step 1: Divide LA_q into NO_G grids
    divide_la_into_grids_contiki();
    
    // Step 3: Topology Discovery Phase
    topology_discovery_phase_contiki();
    
    // Step 4: Initialize NO_P = NO_G
    robot.no_p = robot.no_g;
    
    // Step 5: Dispersion Phase
    dispersion_phase_contiki();
    
    // Step 6-8: Count covered grids and send message to BS
    uint32_t covered_grids = 0;
    for (uint32_t i = 0; i < robot.no_g; i++) {
        if (robot.grid_db[i].grid_status == COVERED) {
            covered_grids++;
        }
    }
    
    LOG_INFO("Robot_%d completed local phase. Covered grids: %u/%u\n", 
           robot.robot_id + 1, covered_grids, robot.no_g);
    
    // Send completion message to base station
    send_completion_message_to_base(covered_grids);
}

// Divide assigned LA into grids
void divide_la_into_grids_contiki() {
    LOG_INFO("Robot_%d dividing LA_%u into %u grids\n", 
           robot.robot_id + 1, robot.assigned_la_id, robot.no_g);
    
    float grid_size = sensor_perception_range;
    float start_x = robot.la_center.x - (robot_perception_range / 2) + (grid_size / 2);
    float start_y = robot.la_center.y - (robot_perception_range / 2) + (grid_size / 2);
    
    uint32_t grids_per_row = (uint32_t)sqrt(robot.no_g);
    
    for (uint32_t i = 0; i < robot.no_g; i++) {
        uint32_t row = i / grids_per_row;
        uint32_t col = i % grids_per_row;
        
        robot.grid_db[i].grid_id = i + 1;
        robot.grid_db[i].center_coord.x = start_x + col * grid_size;
        robot.grid_db[i].center_coord.y = start_y + row * grid_size;
        robot.grid_db[i].grid_status = UNCOVERED;  // Initially uncovered
        
        LOG_INFO("Grid_%u: Center(%.2f, %.2f), Status: UNCOVERED\n", 
               robot.grid_db[i].grid_id,
               robot.grid_db[i].center_coord.x,
               robot.grid_db[i].center_coord.y);
    }
}

// Algorithm 3: Topology Discovery Phase
void topology_discovery_phase_contiki() {
    LOG_INFO("--- Robot_%d Topology Discovery Phase ---\n", robot.robot_id + 1);
    
    // Step 1: Move Robot_p to centre of LA_q
    robot.current_position = robot.la_center;
    LOG_INFO("Robot_%d moved to center of LA_%u at (%.2f, %.2f)\n", 
           robot.robot_id + 1, robot.assigned_la_id,
           robot.current_position.x, robot.current_position.y);
    
    // Step 2: Broadcast message (Mp) to discover sensors
    broadcast_robot_message_contiki();
    
    // Reset sensor database for new discoveries
    robot.sensor_db_count = 0;
    
    LOG_INFO("Topology discovery initiated. Waiting for sensor responses...\n");
}

// Broadcast robot message for topology discovery
void broadcast_robot_message_contiki() {
    network_message_t msg;
    robot_broadcast_message_t broadcast_data = {robot.robot_id};
    
    msg.type = MSG_ROBOT_BROADCAST;
    msg.length = sizeof(robot_broadcast_message_t);
    memcpy(msg.data, &broadcast_data, sizeof(robot_broadcast_message_t));
    
    // Broadcast to all nodes in the network
    uip_create_linklocal_allnodes_mcast(&base_station_addr);
    simple_udp_sendto(&udp_conn, &msg, sizeof(msg), &base_station_addr);
    
    LOG_INFO("Robot_%d broadcasting discovery message Mp(%u)\n", 
           robot.robot_id + 1, robot.robot_id);
}

// Algorithm 4: Dispersion Phase
void dispersion_phase_contiki() {
    LOG_INFO("--- Robot_%d Dispersion Phase ---\n", robot.robot_id + 1);
    
    uint32_t current_grid = 0;
    
    // Algorithm 4: while NO_P > 0 do
    while (robot.no_p > 0 && current_grid < robot.no_g) {
        LOG_INFO("Processing Grid_%u (NO_P = %u)\n", current_grid + 1, robot.no_p);
        
        // Count sensors in current grid
        uint32_t sensors_in_grid = count_sensors_in_grid(current_grid);
        
        // Determine which case applies and handle accordingly
        if (robot.stock_rs > 0 && sensors_in_grid > 0) {
            // Case 1: Robot has sensors in Stock_RS and G_i has sensors
            handle_dispersion_case_1_contiki(current_grid);
        } else if (robot.stock_rs > 0 && sensors_in_grid == 0) {
            // Case 2: Robot has sensors in Stock_RS but G_i has no sensors
            handle_dispersion_case_2_contiki(current_grid);
        } else if (robot.stock_rs == 0 && sensors_in_grid > 0) {
            // Case 3: Robot has no sensors in Stock_RS but G_i has sensors
            handle_dispersion_case_3_contiki(current_grid);
        } else {
            // Case 4: Robot has no sensors in Stock_RS and G_i has no sensors
            handle_dispersion_case_4_contiki(current_grid);
        }
        
        // Decrement NO_P and find next grid
        robot.no_p--;
        current_grid = find_nearest_uncovered_grid_contiki(current_grid);
    }
    
    LOG_INFO("Dispersion phase completed. NO_P = %u\n", robot.no_p);
}

// Count sensors in a specific grid
uint32_t count_sensors_in_grid(uint32_t grid_id) {
    uint32_t count = 0;
    coordinate_t grid_center = robot.grid_db[grid_id].center_coord;
    
    for (uint32_t i = 0; i < robot.sensor_db_count; i++) {
        float distance = calculate_distance(grid_center, robot.sensor_db[i].coord);
        if (distance <= sensor_perception_range) {
            count++;
        }
    }
    
    return count;
}

// Case implementations for dispersion phase
void handle_dispersion_case_1_contiki(uint32_t grid_id) {
    LOG_INFO("Executing Case 1 for Grid_%u\n", grid_id + 1);
    
    // Place a sensor from Stock_RS at the centre of G_i
    uint32_t new_sensor_id = robot.assigned_la_id * 1000 + grid_id + 1;
    
    // Add new sensor to sensor_DB
    if (robot.sensor_db_count < robot.no_g * 4) {
        robot.sensor_db[robot.sensor_db_count] = (sensor_db_record_t){
            .sensor_id = new_sensor_id,
            .coord = robot.grid_db[grid_id].center_coord,
            .sensor_status = ACTIVE
        };
        robot.sensor_db_count++;
        robot.stock_rs--;
        
        LOG_INFO("Placed Sensor_%u at grid center (%.2f, %.2f), Status: ACTIVE\n", 
               new_sensor_id, robot.grid_db[grid_id].center_coord.x, 
               robot.grid_db[grid_id].center_coord.y);
    }
    
    // Collect extra sensors from G_i until Stock_RS capacity is reached
    collect_extra_sensors_from_grid(grid_id);
    
    // Mark G_i as covered
    robot.grid_db[grid_id].grid_status = COVERED;
    LOG_INFO("Grid_%u marked as COVERED\n", grid_id + 1);
}

void handle_dispersion_case_2_contiki(uint32_t grid_id) {
    LOG_INFO("Executing Case 2 for Grid_%u\n", grid_id + 1);
    
    // Place a sensor from Stock_RS at the centre of G_i
    uint32_t new_sensor_id = robot.assigned_la_id * 1000 + grid_id + 1;
    
    if (robot.sensor_db_count < robot.no_g * 4) {
        robot.sensor_db[robot.sensor_db_count] = (sensor_db_record_t){
            .sensor_id = new_sensor_id,
            .coord = robot.grid_db[grid_id].center_coord,
            .sensor_status = ACTIVE
        };
        robot.sensor_db_count++;
        robot.stock_rs--;
        
        LOG_INFO("Placed Sensor_%u at grid center (%.2f, %.2f), Status: ACTIVE\n", 
               new_sensor_id, robot.grid_db[grid_id].center_coord.x, 
               robot.grid_db[grid_id].center_coord.y);
    }
    
    // Mark G_i as covered
    robot.grid_db[grid_id].grid_status = COVERED;
    LOG_INFO("Grid_%u marked as COVERED\n", grid_id + 1);
}

void handle_dispersion_case_3_contiki(uint32_t grid_id) {
    LOG_INFO("Executing Case 3 for Grid_%u\n", grid_id + 1);
    
    // Find sensor with minimum distance to grid center
    uint32_t closest_sensor_idx = find_closest_sensor_to_grid(grid_id);
    
    if (closest_sensor_idx != UINT32_MAX) {
        // Move the closest sensor to grid center
        robot.sensor_db[closest_sensor_idx].coord = robot.grid_db[grid_id].center_coord;
        robot.sensor_db[closest_sensor_idx].sensor_status = ACTIVE;
        
        LOG_INFO("Moved Sensor_%u to grid center (%.2f, %.2f)\n", 
               robot.sensor_db[closest_sensor_idx].sensor_id,
               robot.grid_db[grid_id].center_coord.x,
               robot.grid_db[grid_id].center_coord.y);
        
        // Collect extra sensors from G_i
        collect_extra_sensors_from_grid(grid_id);
        
        // Mark G_i as covered
        robot.grid_db[grid_id].grid_status = COVERED;
        LOG_INFO("Grid_%u marked as COVERED\n", grid_id + 1);
    }
}

void handle_dispersion_case_4_contiki(uint32_t grid_id) {
    LOG_INFO("Executing Case 4 for Grid_%u - leaving uncovered\n", grid_id + 1);
    // G_i remains uncovered - no action needed
    robot.grid_db[grid_id].grid_status = UNCOVERED;
}

// Helper functions
void collect_extra_sensors_from_grid(uint32_t grid_id) {
    coordinate_t grid_center = robot.grid_db[grid_id].center_coord;
    uint32_t collected = 0;
    
    for (uint32_t i = 0; i < robot.sensor_db_count && robot.stock_rs < ROBOT_CAPACITY; i++) {
        if (robot.sensor_db[i].sensor_status == IDLE) {
            float distance = calculate_distance(grid_center, robot.sensor_db[i].coord);
            if (distance <= sensor_perception_range) {
                // "Collect" this sensor (remove from database)
                for (uint32_t j = i; j < robot.sensor_db_count - 1; j++) {
                    robot.sensor_db[j] = robot.sensor_db[j + 1];
                }
                robot.sensor_db_count--;
                robot.stock_rs++;
                collected++;
                i--; // Adjust index due to removal
                
                LOG_INFO("Collected extra sensor, Stock_RS now: %u\n", robot.stock_rs);
            }
        }
    }
    
    LOG_INFO("Collected %u extra sensors from Grid_%u\n", collected, grid_id + 1);
}

uint32_t find_closest_sensor_to_grid(uint32_t grid_id) {
    uint32_t closest_idx = UINT32_MAX;
    float min_distance = INFINITY;
    coordinate_t grid_center = robot.grid_db[grid_id].center_coord;
    
    for (uint32_t i = 0; i < robot.sensor_db_count; i++) {
        if (robot.sensor_db[i].sensor_status == IDLE) {
            float distance = calculate_distance(grid_center, robot.sensor_db[i].coord);
            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = i;
            }
        }
    }
    
    return closest_idx;
}

uint32_t find_nearest_uncovered_grid_contiki(uint32_t current_grid) {
    uint32_t next_grid = current_grid + 1;
    
    // Simple linear search for next uncovered grid
    while (next_grid < robot.no_g && robot.grid_db[next_grid].grid_status == COVERED) {
        next_grid++;
    }
    
    return next_grid;
}

// Send completion message to base station
void send_completion_message_to_base(uint32_t covered_grids) {
    network_message_t msg;
    robot_message_t completion_data = {robot.robot_id, covered_grids};
    
    msg.type = MSG_ROBOT_COMPLETION;
    msg.length = sizeof(robot_message_t);
    memcpy(msg.data, &completion_data, sizeof(robot_message_t));
    
    simple_udp_sendto(&udp_conn, &msg, sizeof(msg), &base_station_addr);
    
    LOG_INFO("Robot_%d sent completion message to BS: (%u, %u)\n", 
           robot.robot_id + 1, robot.robot_id, covered_grids);
}

// Main mobile robot process
PROCESS_THREAD(mobile_robot_process, ev, data)
{
    PROCESS_BEGIN();
    
    LOG_INFO("=== WSN Deployment Mobile Robot - Contiki-NG ===\n");
    
    // Initialize network
    simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL, UDP_SERVER_PORT, udp_rx_callback);
    
    // Set base station address (should be configured based on network topology)
    uip_ip6addr(&base_station_addr, 0xfe80, 0, 0, 0, 0, 0, 0, 1);
    
    // Initialize mobile robot (Robot ID determined by node ID)
    robot_id_t id = (linkaddr_node_addr.u8[7] == 1) ? ROBOT_1 : ROBOT_2;
    uint32_t initial_la = (id == ROBOT_1) ? 1 : (uint32_t)floor(target_area_size / robot_perception_range);
    
    initialize_mobile_robot_contiki(id, initial_la);
    
    // Start initial local phase
    process_post(&local_phase_process, PROCESS_EVENT_MSG, NULL);
    
    while(1) {
        PROCESS_YIELD();
    }
    
    PROCESS_END();
}

// Local phase execution process
PROCESS_THREAD(local_phase_process, ev, data)
{
    static struct etimer discovery_timer;
    
    PROCESS_BEGIN();
    
    while(1) {
        PROCESS_WAIT_EVENT();
        
        if(ev == PROCESS_EVENT_MSG) {
            LOG_INFO("Starting local phase execution...\n");
            
            // Execute local phase
            execute_local_phase_contiki();
            
            // Wait for sensor responses during topology discovery
            etimer_set(&discovery_timer, CLOCK_SECOND * 5);  // 5 second wait
            PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&discovery_timer));
            
            LOG_INFO("Local phase completed\n");
        }
    }
    
    PROCESS_END();
}
