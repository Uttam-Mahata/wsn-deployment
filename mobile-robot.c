#include "contiki.h"
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "sys/etimer.h"
#include "sys/clock.h"
#include "random.h"
#include "project-conf.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "sys/log.h"
#define LOG_MODULE "MobileRobot"
#define LOG_LEVEL LOG_LEVEL_APP

/* Robot operational phases */
typedef enum {
    ROBOT_PHASE_IDLE = 0,
    ROBOT_PHASE_TOPOLOGY_DISCOVERY = 1,
    ROBOT_PHASE_DISPERSION = 2,
    ROBOT_PHASE_REPORTING = 3
} robot_phase_t;

/* Message structures from base station and sensor communications */
typedef struct {
    uint8_t la_id;
    uint16_t center_x;
    uint16_t center_y;
    uint8_t no_grid;
} la_assignment_msg_t;

typedef struct {
    uint8_t sensor_id;
    uint16_t x_coord;
    uint16_t y_coord;
    uint8_t sensor_status;
} sensor_reply_msg_t;

typedef struct {
    uint8_t robot_id;
} robot_discovery_msg_t;

typedef struct {
    uint8_t robot_id;
    uint8_t covered_grids;
} robot_report_msg_t;

/* Grid and Sensor Database Records */
typedef struct {
    uint8_t grid_id;
    uint16_t center_x;
    uint16_t center_y;
    uint8_t grid_status; // 0 = uncovered, 1 = covered
} grid_db_record_t;

typedef struct {
    uint8_t sensor_id;
    uint16_t x_coord;
    uint16_t y_coord;
    uint8_t sensor_status; // 0 = idle, 1 = active
} sensor_db_record_t;

/* Mobile Robot State */
static struct {
    uint8_t robot_id;
    uint16_t current_x;
    uint16_t current_y;
    robot_phase_t current_phase;
    
    /* Current assignment */
    uint8_t assigned_la_id;
    uint16_t la_center_x;
    uint16_t la_center_y;
    
    /* Local databases */
    grid_db_record_t grid_db[MAX_SENSORS_PER_AREA];
    sensor_db_record_t sensor_db[MAX_SENSORS_PER_AREA];
    uint8_t num_grids;
    uint8_t num_sensors;
    
    /* Robot stock and movement */
    uint8_t stock_rs; // Current sensor stock
    uint8_t no_p;     // Number of permissible moves
    uint8_t current_grid_index;
    
    /* Energy tracking */
    float total_energy_consumed;
    float baseline_energy;
    float radio_energy;
    float mobility_energy;
    float total_distance_moved;
    
    /* Operation counters */
    uint32_t tx_operations;
    uint32_t rx_operations;
    uint32_t movement_operations;
    uint32_t processing_operations;
    
    /* Timing */
    clock_time_t start_time;
    clock_time_t last_energy_calc;
    clock_time_t phase_start_time;
    
    /* Communication */
    uip_ipaddr_t base_station_addr;
    uint8_t bs_reachable;
} mobile_robot;

static struct simple_udp_connection udp_conn;
static struct etimer phase_timer;
static struct etimer energy_timer;
static struct etimer discovery_timer;

PROCESS(mobile_robot_process, "Mobile Robot Process");
AUTOSTART_PROCESSES(&mobile_robot_process);

/* Forward declarations */
static float calculate_distance(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
static int8_t find_nearest_sensor_to_grid(uint8_t grid_index);
static void deploy_or_relocate_sensor_to_grid(uint8_t sensor_index, uint8_t grid_index, uint8_t deploy_from_stock);
static void move_robot(uint16_t target_x, uint16_t target_y);

/* Energy Calculation Functions */
static float calculate_baseline_energy(float time_duration) {
    return time_duration * P_BASELINE_ROBOT;
}

static float calculate_radio_energy(uint32_t tx_ops, uint32_t rx_ops, float avg_tx_time, float avg_rx_time) {
    float tx_energy = tx_ops * P_TRANSMIT_ROBOT * avg_tx_time;
    float rx_energy = rx_ops * P_RECEIVE_ROBOT * avg_rx_time;
    return tx_energy + rx_energy;
}

static float calculate_mobility_energy(float distance_moved) {
    return TAU_MOBILITY * distance_moved;
}

static void update_energy_consumption() {
    clock_time_t current_time = clock_time();
    float time_elapsed = (float)(current_time - mobile_robot.last_energy_calc) / CLOCK_SECOND;
    
    /* Calculate baseline energy */
    mobile_robot.baseline_energy += calculate_baseline_energy(time_elapsed);
    
    /* Calculate radio energy */
    float avg_tx_time = 0.001; // 1ms average transmission time
    float avg_rx_time = 0.001; // 1ms average reception time
    mobile_robot.radio_energy += calculate_radio_energy(
        mobile_robot.tx_operations, mobile_robot.rx_operations,
        avg_tx_time, avg_rx_time);
    
    /* Calculate mobility energy */
    mobile_robot.mobility_energy += calculate_mobility_energy(mobile_robot.total_distance_moved);
    
    /* Update total energy consumption */
    mobile_robot.total_energy_consumed = mobile_robot.baseline_energy + 
                                       mobile_robot.radio_energy + 
                                       mobile_robot.mobility_energy;
    
    mobile_robot.last_energy_calc = current_time;
    
    /* Reset counters */
    mobile_robot.tx_operations = 0;
    mobile_robot.rx_operations = 0;
    mobile_robot.total_distance_moved = 0;
}

/* Movement and Grid Operations */
static float calculate_distance(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    float dx = (float)(x2 - x1);
    float dy = (float)(y2 - y1);
    return sqrt(dx * dx + dy * dy);
}

static void move_robot(uint16_t new_x, uint16_t new_y) {
    float distance = calculate_distance(mobile_robot.current_x, mobile_robot.current_y, new_x, new_y);
    mobile_robot.total_distance_moved += distance;
    mobile_robot.current_x = new_x;
    mobile_robot.current_y = new_y;
    mobile_robot.movement_operations++;
    
    LOG_INFO("Robot moved to (%u, %u), distance: %.2f\n", new_x, new_y, distance);
}

static void initialize_grid_db() {
    uint8_t grid_size = ROBOT_PERCEPTION_RANGE / SENSOR_PERCEPTION_RANGE;
    mobile_robot.num_grids = grid_size * grid_size;
    
    if (mobile_robot.num_grids > MAX_SENSORS_PER_AREA) {
        mobile_robot.num_grids = MAX_SENSORS_PER_AREA;
        grid_size = (uint8_t)sqrt(mobile_robot.num_grids);
    }
    
    uint8_t grid_count = 0;
    uint16_t start_x = mobile_robot.la_center_x - ROBOT_PERCEPTION_RANGE / 2;
    uint16_t start_y = mobile_robot.la_center_y - ROBOT_PERCEPTION_RANGE / 2;
    
    for (uint8_t y = 0; y < grid_size && grid_count < mobile_robot.num_grids; y++) {
        for (uint8_t x = 0; x < grid_size && grid_count < mobile_robot.num_grids; x++) {
            mobile_robot.grid_db[grid_count].grid_id = grid_count + 1;
            mobile_robot.grid_db[grid_count].center_x = start_x + x * SENSOR_PERCEPTION_RANGE + SENSOR_PERCEPTION_RANGE / 2;
            mobile_robot.grid_db[grid_count].center_y = start_y + y * SENSOR_PERCEPTION_RANGE + SENSOR_PERCEPTION_RANGE / 2;
            mobile_robot.grid_db[grid_count].grid_status = 0; // Initially uncovered
            grid_count++;
        }
    }
    
    mobile_robot.no_p = mobile_robot.num_grids; // Set permissible moves
    LOG_INFO("Initialized %u grids in LA %u\n", mobile_robot.num_grids, mobile_robot.assigned_la_id);
}

static int8_t find_uncovered_grid() {
    for (uint8_t i = 0; i < mobile_robot.num_grids; i++) {
        if (mobile_robot.grid_db[i].grid_status == 0) {
            return i;
        }
    }
    return -1; // No uncovered grid found
}

static int8_t find_nearest_sensor_to_grid(uint8_t grid_index) {
    int8_t nearest_sensor = -1;
    float min_distance = 10000.0; // Large initial value
    
    uint16_t grid_x = mobile_robot.grid_db[grid_index].center_x;
    uint16_t grid_y = mobile_robot.grid_db[grid_index].center_y;
    
    for (uint8_t i = 0; i < mobile_robot.num_sensors; i++) {
        if (mobile_robot.sensor_db[i].sensor_status == 0) { // Idle sensor
            float distance = calculate_distance(mobile_robot.sensor_db[i].x_coord, 
                                             mobile_robot.sensor_db[i].y_coord,
                                             grid_x, grid_y);
            if (distance < min_distance) {
                min_distance = distance;
                nearest_sensor = i;
            }
        }
    }
    
    return nearest_sensor;
}

/* Phase Operations */
static void start_topology_discovery() {
    mobile_robot.current_phase = ROBOT_PHASE_TOPOLOGY_DISCOVERY;
    mobile_robot.phase_start_time = clock_time();
    mobile_robot.num_sensors = 0;
    
    /* Move to center of assigned LA first (as per APP_I algorithm) */
    move_robot(mobile_robot.la_center_x, mobile_robot.la_center_y);
    
    /* Initialize grid database after moving to center */
    initialize_grid_db();
    
    LOG_INFO("Robot %u: Topology discovery in LA %u from center (%u, %u)\n", 
             mobile_robot.robot_id, mobile_robot.assigned_la_id, 
             mobile_robot.la_center_x, mobile_robot.la_center_y);
    
    /* Broadcast Mp message as per APP_I specification to discover randomly deployed sensors */
    robot_discovery_msg_t discovery_msg;
    discovery_msg.robot_id = mobile_robot.robot_id;
    
    uip_ipaddr_t sensor_addr;
    uip_ip6addr(&sensor_addr, 0xff02, 0, 0, 0, 0, 0, 0, 1); // Broadcast to all sensors
    simple_udp_sendto(&udp_conn, &discovery_msg, sizeof(discovery_msg), &sensor_addr);
    mobile_robot.tx_operations++;
    
    LOG_INFO("Broadcasted Mp message to discover randomly deployed sensors in LA %u\n", 
             mobile_robot.assigned_la_id);
    
    /* Wait for sensor responses for a short time before proceeding */
    etimer_set(&discovery_timer, 5 * CLOCK_SECOND);
    
    etimer_set(&discovery_timer, 5 * CLOCK_SECOND); // Discovery phase duration
}

static void execute_dispersion_phase() {
    mobile_robot.current_phase = ROBOT_PHASE_DISPERSION;
    mobile_robot.phase_start_time = clock_time();
    mobile_robot.current_grid_index = 0;
    
    LOG_INFO("Started dispersion phase with %u discovered sensors and %u sensors in stock\n", 
             mobile_robot.num_sensors, mobile_robot.stock_rs);
    
    /* Initialize the permissible movement counter (NO_P) for this local phase */
    mobile_robot.no_p = mobile_robot.num_grids; // As per APP_I algorithm
    
    /* Process first grid */
    etimer_set(&phase_timer, 2 * CLOCK_SECOND);
}

static void deploy_or_relocate_sensor_to_grid(uint8_t sensor_index, uint8_t grid_index, uint8_t deploy_from_stock) {
    /* Coordinates for deployment/relocation */
    uint16_t target_x = mobile_robot.grid_db[grid_index].center_x;
    uint16_t target_y = mobile_robot.grid_db[grid_index].center_y;
    
    /* Send deployment/relocation command to sensor */
    uint16_t command_data[3]; // First two bytes are coordinates, third is deploy flag
    command_data[0] = target_x;
    command_data[1] = target_y;
    command_data[2] = deploy_from_stock; // 1 = new deployment from stock, 0 = relocate existing
    
    uip_ipaddr_t sensor_addr;
    
    if (deploy_from_stock) {
        /* Deploying a new sensor from robot stock - use broadcast to reach undeployed sensors */
        uip_ip6addr(&sensor_addr, 0xff02, 0, 0, 0, 0, 0, 0, 1); // Broadcast address
        
        mobile_robot.stock_rs--; // Reduce stock
        LOG_INFO("Deploying new sensor from stock to grid %u at (%u, %u), %u sensors remaining in stock\n", 
                 grid_index, target_x, target_y, mobile_robot.stock_rs);
    } else {
        /* Relocating an existing sensor - use its specific address if available */
        if (sensor_index < mobile_robot.num_sensors) {
            /* We have a specific sensor to relocate */
            uip_ip6addr(&sensor_addr, 0xff02, 0, 0, 0, 0, 0, 0, 1); // Use broadcast for now (simplified)
            
            LOG_INFO("Relocating sensor %u to grid %u at (%u, %u)\n", 
                     mobile_robot.sensor_db[sensor_index].sensor_id,
                     grid_index, target_x, target_y);
        } else {
            LOG_INFO("Error: Invalid sensor index for relocation\n");
            return;
        }
    }
    
    /* Send the command */
    simple_udp_sendto(&udp_conn, command_data, sizeof(command_data), &sensor_addr);
    mobile_robot.tx_operations++;
    
    /* Mark this grid as covered */
    mobile_robot.grid_db[grid_index].grid_status = 1;
}

static void process_grid_deployment(uint8_t grid_index) {
    if (mobile_robot.no_p <= 0 || grid_index >= mobile_robot.num_grids) {
        /* Finished dispersion phase */
        mobile_robot.current_phase = ROBOT_PHASE_REPORTING;
        etimer_set(&phase_timer, 1 * CLOCK_SECOND);
        LOG_INFO("Dispersion phase complete, reporting results\n");
        return;
    }
    
    /* Move to grid center */
    move_robot(mobile_robot.grid_db[grid_index].center_x, 
               mobile_robot.grid_db[grid_index].center_y);
    
    /* Reduce NO_P by 1 after visiting each grid (as per APP_I) */
    mobile_robot.no_p--;
    
    /* Find sensors in current grid (within sensor perception range of grid center) */
    uint8_t sensors_in_grid = 0;
    uint8_t grid_sensor_indices[MAX_SENSORS_PER_AREA];
    
    for (uint8_t i = 0; i < mobile_robot.num_sensors; i++) {
        if (mobile_robot.sensor_db[i].sensor_status == 0) { // Idle sensors only
            float distance = calculate_distance(mobile_robot.sensor_db[i].x_coord,
                                              mobile_robot.sensor_db[i].y_coord,
                                              mobile_robot.current_x, mobile_robot.current_y);
            if (distance <= SENSOR_PERCEPTION_RANGE && sensors_in_grid < MAX_SENSORS_PER_AREA) {
                grid_sensor_indices[sensors_in_grid] = i;
                sensors_in_grid++;
            }
        }
    }
    
    LOG_INFO("Processing Grid %u at (%u, %u): %u sensors in grid, %u sensors in stock\n",
             grid_index + 1, mobile_robot.current_x, mobile_robot.current_y, 
             sensors_in_grid, mobile_robot.stock_rs);
    
    /* APP_I Algorithm: 4 Cases of sensor redeployment (as per section y. Local phase) */
    
    if (mobile_robot.stock_rs > 0 && sensors_in_grid > 0) {
        /* Case 1: Robot has sensor(s) in Stock_RS and Grid has sensor(s) after random deployment */
        LOG_INFO("Case 1: Stock has sensors, grid has sensors\n");
        
        /* Place a sensor from Stock_RS at the center of grid and mark as active */
        deploy_or_relocate_sensor_to_grid(0, grid_index, 1); // Deploy from stock
        mobile_robot.stock_rs--;
        
        /* Collect all extra sensors from grid till Stock_RS is less than 15 */
        uint8_t collected = 0;
        for (uint8_t i = 0; i < sensors_in_grid && mobile_robot.stock_rs < ROBOT_STOCK_CAPACITY; i++) {
            uint8_t sensor_idx = grid_sensor_indices[i];
            mobile_robot.sensor_db[sensor_idx].sensor_status = 2; // Mark as collected
            mobile_robot.stock_rs++;
            collected++;
            LOG_INFO("Collected sensor %u from grid into stock\n", 
                     mobile_robot.sensor_db[sensor_idx].sensor_id);
        }
        
        /* Mark grid as covered */
        mobile_robot.grid_db[grid_index].grid_status = 1;
        LOG_INFO("Grid %u covered: deployed 1 from stock, collected %u sensors\n", 
                 grid_index + 1, collected);
        
    } else if (mobile_robot.stock_rs > 0 && sensors_in_grid == 0) {
        /* Case 2: Robot has sensors in Stock_RS but Grid has no sensors */
        LOG_INFO("Case 2: Stock has sensors, grid has no sensors\n");
        
        /* Place a sensor from Stock_RS at the center of grid and mark as active */
        deploy_or_relocate_sensor_to_grid(0, grid_index, 1); // Deploy from stock
        mobile_robot.stock_rs--;
        
        /* Mark grid as covered */
        mobile_robot.grid_db[grid_index].grid_status = 1;
        LOG_INFO("Grid %u covered: deployed 1 sensor from stock\n", grid_index + 1);
        
    } else if (mobile_robot.stock_rs == 0 && sensors_in_grid > 0) {
        /* Case 3: Robot has no sensors in Stock_RS but Grid has sensors */
        LOG_INFO("Case 3: No stock, grid has sensors\n");
        
        /* Find sensor with minimum distance to grid center */
        int8_t nearest_sensor = find_nearest_sensor_to_grid(grid_index);
        if (nearest_sensor >= 0) {
            /* Place that sensor at grid center and mark as active */
            deploy_or_relocate_sensor_to_grid(nearest_sensor, grid_index, 0); // Relocate existing
            mobile_robot.sensor_db[nearest_sensor].sensor_status = 1; // Mark as active
            
            /* Collect all extra sensors from grid till Stock_RS is less than 15 */
            uint8_t collected = 0;
            for (uint8_t i = 0; i < sensors_in_grid && mobile_robot.stock_rs < ROBOT_STOCK_CAPACITY; i++) {
                uint8_t sensor_idx = grid_sensor_indices[i];
                if (sensor_idx != nearest_sensor) { // Don't collect the one we just placed
                    mobile_robot.sensor_db[sensor_idx].sensor_status = 2; // Mark as collected
                    mobile_robot.stock_rs++;
                    collected++;
                    LOG_INFO("Collected sensor %u from grid into stock\n", 
                             mobile_robot.sensor_db[sensor_idx].sensor_id);
                }
            }
            
            /* Mark grid as covered */
            mobile_robot.grid_db[grid_index].grid_status = 1;
            LOG_INFO("Grid %u covered: relocated nearest sensor, collected %u sensors\n", 
                     grid_index + 1, collected);
        }
        
    } else {
        /* Case 4: Robot has no sensors in Stock_RS and Grid has no sensors */
        LOG_INFO("Case 4: No stock, no sensors in grid - grid remains uncovered\n");
        /* In this case Grid remains uncovered - no action taken */
    }
    
    mobile_robot.processing_operations++;
    
    /* Find next uncovered grid within LA */
    int8_t next_grid = find_uncovered_grid();
    if (next_grid >= 0 && mobile_robot.no_p > 0) {
        /* Continue to next grid */
        LOG_INFO("Moving to next uncovered grid %d, %d permissible moves remaining\n", 
                next_grid, mobile_robot.no_p);
        mobile_robot.current_grid_index = next_grid;
        etimer_set(&phase_timer, 2 * CLOCK_SECOND);
    } else {
        /* All grids processed or no more permissible moves (NO_P = 0), move to reporting phase */
        mobile_robot.current_phase = ROBOT_PHASE_REPORTING;
        
        /* Count covered grids */
        uint8_t covered_grids = 0;
        for (uint8_t i = 0; i < mobile_robot.num_grids; i++) {
            if (mobile_robot.grid_db[i].grid_status == 1) {
                covered_grids++;
            }
        }
        
        LOG_INFO("Dispersion phase complete: %u/%u grids covered, %u permissible moves used\n", 
                 covered_grids, mobile_robot.num_grids, 
                 mobile_robot.num_grids - mobile_robot.no_p);
                 
        etimer_set(&phase_timer, 1 * CLOCK_SECOND);
    }
}

static void send_coverage_report() {
    /* Count covered grids (Cov_G as per APP_I) */
    uint8_t covered_grids = 0;
    for (uint8_t i = 0; i < mobile_robot.num_grids; i++) {
        if (mobile_robot.grid_db[i].grid_status == 1) {
            covered_grids++;
        }
    }
    
    /* Calculate coverage percentage for this LA */
    float coverage_percentage = (mobile_robot.num_grids > 0) ?
        (covered_grids * 100.0f / mobile_robot.num_grids) : 0.0f;
    
    LOG_INFO("Local phase complete: %u/%u grids covered (%.2f%%)\n", 
             covered_grids, mobile_robot.num_grids, coverage_percentage);
    
    /* Send Robot_pM message as specified in APP_I */
    robot_report_msg_t report;
    report.robot_id = mobile_robot.robot_id;
    report.covered_grids = covered_grids;
    
    if (mobile_robot.bs_reachable) {
        simple_udp_sendto(&udp_conn, &report, sizeof(report), &mobile_robot.base_station_addr);
        mobile_robot.tx_operations++;
        
        LOG_INFO("Sent Robot_%uM: (%u, %u) to BS - local phase complete\n", 
                 mobile_robot.robot_id, mobile_robot.robot_id, covered_grids);
    }
    
    /* Reset NO_P to NO_G for next assignment as per APP_I */
    mobile_robot.no_p = mobile_robot.num_grids;
    
    /* Reset for next assignment */
    mobile_robot.current_phase = ROBOT_PHASE_IDLE;
    memset(mobile_robot.grid_db, 0, sizeof(mobile_robot.grid_db));
    memset(mobile_robot.sensor_db, 0, sizeof(mobile_robot.sensor_db));
    mobile_robot.num_grids = 0;
    mobile_robot.num_sensors = 0;
    
    LOG_INFO("Robot %u ready for next LA assignment\n", mobile_robot.robot_id);
}

/* Add new message structure to match base station */
typedef struct {
    uint8_t target_robot_id;
    la_assignment_msg_t la_assignment;
} robot_assignment_msg_t;

/* Communication Handlers */
static void udp_rx_callback(struct simple_udp_connection *c,
                           const uip_ipaddr_t *sender_addr,
                           uint16_t sender_port,
                           const uip_ipaddr_t *receiver_addr,
                           uint16_t receiver_port,
                           const uint8_t *data,
                           uint16_t datalen) {
    
    mobile_robot.rx_operations++;
    
    /* Handle robot assignment message from base station */
    if (datalen == sizeof(robot_assignment_msg_t)) {
        robot_assignment_msg_t *assignment_msg = (robot_assignment_msg_t *)data;
        
        /* Check if this assignment is for this specific robot */
        if (assignment_msg->target_robot_id == mobile_robot.robot_id && 
            mobile_robot.current_phase == ROBOT_PHASE_IDLE) {
            
            la_assignment_msg_t *assignment = &assignment_msg->la_assignment;
            mobile_robot.assigned_la_id = assignment->la_id;
            mobile_robot.la_center_x = assignment->center_x;
            mobile_robot.la_center_y = assignment->center_y;
            
            /* Store base station address */
            uip_ipaddr_copy(&mobile_robot.base_station_addr, sender_addr);
            mobile_robot.bs_reachable = 1;
            
            LOG_INFO("Robot %u received LA assignment: LA %u at (%u, %u)\n", 
                     mobile_robot.robot_id, assignment->la_id, assignment->center_x, assignment->center_y);
            
            /* Start topology discovery */
            start_topology_discovery();
        }
    }
    
    /* Handle sensor replies during topology discovery */
    if (datalen == sizeof(sensor_reply_msg_t) && mobile_robot.current_phase == ROBOT_PHASE_TOPOLOGY_DISCOVERY) {
        sensor_reply_msg_t *sensor_reply = (sensor_reply_msg_t *)data;
        
        /* Check if sensor is within robot's perception range and within assigned LA */
        float distance_to_robot = calculate_distance(sensor_reply->x_coord, sensor_reply->y_coord,
                                                   mobile_robot.current_x, mobile_robot.current_y);
        
        /* Also check if sensor is within the LA boundaries */
        uint16_t la_start_x = mobile_robot.la_center_x - ROBOT_PERCEPTION_RANGE / 2;
        uint16_t la_end_x = mobile_robot.la_center_x + ROBOT_PERCEPTION_RANGE / 2;
        uint16_t la_start_y = mobile_robot.la_center_y - ROBOT_PERCEPTION_RANGE / 2;
        uint16_t la_end_y = mobile_robot.la_center_y + ROBOT_PERCEPTION_RANGE / 2;
        
        bool within_la = (sensor_reply->x_coord >= la_start_x && sensor_reply->x_coord <= la_end_x &&
                         sensor_reply->y_coord >= la_start_y && sensor_reply->y_coord <= la_end_y);
        
        if (distance_to_robot <= ROBOT_PERCEPTION_RANGE && within_la && 
            mobile_robot.num_sensors < MAX_SENSORS_PER_AREA) {
            
            /* Add sensor to database */
            mobile_robot.sensor_db[mobile_robot.num_sensors].sensor_id = sensor_reply->sensor_id;
            mobile_robot.sensor_db[mobile_robot.num_sensors].x_coord = sensor_reply->x_coord;
            mobile_robot.sensor_db[mobile_robot.num_sensors].y_coord = sensor_reply->y_coord;
            mobile_robot.sensor_db[mobile_robot.num_sensors].sensor_status = sensor_reply->sensor_status;
            mobile_robot.num_sensors++;
            
            LOG_INFO("Discovered sensor %u at (%u, %u) within LA %u, status: %u\n", 
                     sensor_reply->sensor_id, sensor_reply->x_coord, 
                     sensor_reply->y_coord, mobile_robot.assigned_la_id, sensor_reply->sensor_status);
        } else if (!within_la) {
            LOG_INFO("Sensor %u at (%u, %u) outside LA %u boundaries - ignored\n", 
                     sensor_reply->sensor_id, sensor_reply->x_coord, 
                     sensor_reply->y_coord, mobile_robot.assigned_la_id);
        }
    }
}

static void print_energy_report() {
    update_energy_consumption();
    
    clock_time_t elapsed = clock_time() - mobile_robot.start_time;
    float elapsed_seconds = (float)elapsed / CLOCK_SECOND;
    
    LOG_INFO("=== ROBOT ENERGY REPORT ===\n");
    LOG_INFO("Robot ID: %u\n", mobile_robot.robot_id);
    LOG_INFO("Position: (%u, %u)\n", mobile_robot.current_x, mobile_robot.current_y);
    LOG_INFO("Phase: %u\n", mobile_robot.current_phase);
    LOG_INFO("Assigned LA: %u\n", mobile_robot.assigned_la_id);
    LOG_INFO("Sensor stock: %u\n", mobile_robot.stock_rs);
    LOG_INFO("Elapsed time: %.2f seconds\n", elapsed_seconds);
    LOG_INFO("Baseline energy: %.6f J\n", mobile_robot.baseline_energy);
    LOG_INFO("Radio energy: %.6f J\n", mobile_robot.radio_energy);
    LOG_INFO("Mobility energy: %.6f J\n", mobile_robot.mobility_energy);
    LOG_INFO("Total energy: %.6f J\n", mobile_robot.total_energy_consumed);
    LOG_INFO("Operations - TX: %u, RX: %u, Moves: %u, Processing: %u\n",
            mobile_robot.tx_operations, mobile_robot.rx_operations,
            mobile_robot.movement_operations, mobile_robot.processing_operations);
    LOG_INFO("==========================\n");
}

PROCESS_THREAD(mobile_robot_process, ev, data) {
    PROCESS_BEGIN();
    
    /* Initialize mobile robot with fixed ID based on node ID */
    mobile_robot.robot_id = (linkaddr_node_addr.u8[0] == 2) ? 0 : 1; // Robot 0 for node 2, Robot 1 for node 3
    mobile_robot.start_time = clock_time();
    mobile_robot.last_energy_calc = mobile_robot.start_time;
    mobile_robot.current_phase = ROBOT_PHASE_IDLE;
    mobile_robot.stock_rs = ROBOT_INITIAL_STOCK;
    mobile_robot.bs_reachable = 0;
    
    /* Initialize position (robots start at base station area) */
    mobile_robot.current_x = TARGET_AREA_WIDTH / 2;
    mobile_robot.current_y = TARGET_AREA_HEIGHT / 2;
    
    /* Initialize UDP connection */
    simple_udp_register(&udp_conn, UDP_SERVER_PORT, NULL, UDP_CLIENT_PORT, udp_rx_callback);
    
    /* Set energy reporting timer */
    etimer_set(&energy_timer, ENERGY_REPORT_INTERVAL);
    
    LOG_INFO("Mobile Robot %u initialized with %u sensors in stock\n", 
             mobile_robot.robot_id, mobile_robot.stock_rs);
    
    while(1) {
        PROCESS_WAIT_EVENT();
        
        if (ev == PROCESS_EVENT_TIMER) {
            if (data == &phase_timer) {
                if (mobile_robot.current_phase == ROBOT_PHASE_DISPERSION) {
                    process_grid_deployment(mobile_robot.current_grid_index);
                } else if (mobile_robot.current_phase == ROBOT_PHASE_REPORTING) {
                    send_coverage_report();
                }
                
            } else if (data == &discovery_timer) {
                if (mobile_robot.current_phase == ROBOT_PHASE_TOPOLOGY_DISCOVERY) {
                    LOG_INFO("Topology discovery complete. Found %u sensors\n", mobile_robot.num_sensors);
                    execute_dispersion_phase();
                }
                
            } else if (data == &energy_timer) {
                print_energy_report();
                etimer_reset(&energy_timer);
            }
        }
    }
    
    PROCESS_END();
}
