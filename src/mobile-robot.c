/*
 * WSN Deployment Mobile Robot Implementation 

 * 1. Grid creation based on NO_G formula
 * 2. Topology Discovery Phase with Mp broadcast
 * 3. Dispersion Phase with 4 cases
 * 4. Energy consumption calculations
 * 5. Limited moves (NO_P) constraint
 */

#include "contiki.h"
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "sys/log.h"
#include "sys/node-id.h"
#include "sys/energest.h"
#include "random.h"
#include "wsn-deployment.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define LOG_MODULE "Robot"
#define LOG_LEVEL LOG_LEVEL_INFO

/* Mobile Robot Process */
PROCESS(mobile_robot_process, "Mobile Robot Process");
AUTOSTART_PROCESSES(&mobile_robot_process);

/* Network communication */
static struct simple_udp_connection udp_conn;

/* Robot state */
static uint8_t robot_id;
static uint8_t current_la_id = 0;
static position_t current_position;
static uint8_t stock_rs = INITIAL_STOCK; /* Robot sensor stock */
static uint8_t no_p; /* Number of permissible moves */
static bool local_phase_active = false;
static bool assignment_received = false;

/* Local databases maintained by robot */
static grid_db_entry_t grid_db[WSN_DEPLOYMENT_CONF_MAX_GRIDS_PER_LA];
static sensor_db_entry_t sensor_db[WSN_DEPLOYMENT_CONF_MAX_SENSORS];
static uint8_t num_grids = 0;
static uint8_t num_sensors = 0;

/* Pending assignment data */
static la_assignment_msg_t pending_assignment;

/* Energy tracking */
static double total_energy_consumed = 0.0;
static double baseline_energy_robot = 0.0;
static double radio_energy_robot = 0.0;
static double mobility_energy = 0.0;
static double distance_traveled = 0.0;
static clock_time_t energy_last_update = 0;

/* Timing parameters from paper */
static const double interval_count = 10.0;      /* I_C */
static const double cycle_time = 1.0;           /* T_C in seconds */
static const double processing_time = 0.1;      /* T_P in seconds */

/* Energy parameters for robot */
static const double power_baseline_robot = 0.1;    /* 100 mW baseline power */
static const double power_transmit_robot = 0.2;    /* 200 mW transmit power */
static const double power_receive_robot = 0.15;    /* 150 mW receive power */
static const double tau = 0.0005;                  /* Energy coefficient for movement as per paper */

/*---------------------------------------------------------------------------*/
/* Calculate distance between two positions */
double calculate_distance(position_t pos1, position_t pos2)
{
    double dx = pos1.x - pos2.x;
    double dy = pos1.y - pos2.y;
    return sqrt(dx * dx + dy * dy);
}

/*---------------------------------------------------------------------------*/
/* Calculate baseline energy as per paper: E_baseline = (t_i - t_s) * P_baseline */
double calculate_robot_baseline_energy(double time_duration)
{
    /* t_i = I_C * (T_C + T_P) */
    double t_i = interval_count * (cycle_time + processing_time);
    double t_s = 0; /* Start time, assuming 0 for simplicity */
    
    return (t_i - t_s) * power_baseline_robot * time_duration;
}

/*---------------------------------------------------------------------------*/
/* Calculate radio energy as per paper: E_radio = E_transmit + E_receive */
double calculate_robot_radio_energy(double time_transmit, double time_receive)
{
    double transmit_energy = power_transmit_robot * time_transmit;
    double receive_energy = power_receive_robot * time_receive;
    return transmit_energy + receive_energy;
}

/*---------------------------------------------------------------------------*/
/* Update robot energy consumption */
static void update_robot_energy(void)
{
    clock_time_t current_time = clock_time();
    if (energy_last_update == 0) {
        energy_last_update = current_time;
        return;
    }
    
    double time_duration = (double)(current_time - energy_last_update) / CLOCK_SECOND;
    if (time_duration <= 0) return;
    
    /* Baseline energy consumption using formula from paper */
    double baseline = calculate_robot_baseline_energy(time_duration);
    baseline_energy_robot += baseline;
    total_energy_consumed += baseline;
    
    /* Radio energy (estimated based on communication activity) */
    double radio_energy = calculate_robot_radio_energy(
        time_duration * 0.02,  /* 2% tx time */
        time_duration * 0.05   /* 5% rx time */
    );
    radio_energy_robot += radio_energy;
    total_energy_consumed += radio_energy;
    
    energy_last_update = current_time;
}

/*---------------------------------------------------------------------------*/
/* Calculate and add mobility energy for distance traveled - E_mobility = τ * D_p */
static void add_mobility_energy(double distance)
{
    double mobility = tau * distance;
    mobility_energy += mobility;
    total_energy_consumed += mobility;
    distance_traveled += distance;
    
    LOG_INFO("Mobility energy: %.4f J, Distance: %.2f m, Total: %.4f J\n", 
             mobility, distance, total_energy_consumed);
}

/*---------------------------------------------------------------------------*/
/* Calculate total energy consumption according to paper formula:
   Energy_Tot = E_robot (for robots) */
static double calculate_total_robot_energy(void)
{
    /* For robot: E_robot = E_baseline_robot + E_radio_robot + E_mobility */
    return baseline_energy_robot + radio_energy_robot + mobility_energy;
}

/*---------------------------------------------------------------------------*/
/* Initialize grid database for current LA */
static void init_grid_db(position_t la_center)
{
    uint8_t grid_count = 0;
    int16_t start_x = la_center.x - ROBOT_PERCEPTION_RANGE/2;
    int16_t start_y = la_center.y - ROBOT_PERCEPTION_RANGE/2;
    int16_t grid_size = SENSOR_PERCEPTION_RANGE;
    
    /* NO_G = floor(Robot perception range / Sensor perception range) */
    num_grids = NO_G; /* Using the formula from wsn-deployment.h */
    if (num_grids > WSN_DEPLOYMENT_CONF_MAX_GRIDS_PER_LA) {
        num_grids = WSN_DEPLOYMENT_CONF_MAX_GRIDS_PER_LA;
    }
    
    LOG_INFO("Dividing LA_%d into %d grids (NO_G)\n", current_la_id, num_grids);
    LOG_INFO("Grid size: %d x %d, LA center: (%d, %d)\n", 
             grid_size, grid_size, la_center.x, la_center.y);
    
    /* Create grid layout within the location area */
    uint8_t grids_per_row = (uint8_t)sqrt(num_grids);
    
    for (uint8_t row = 0; row < grids_per_row && grid_count < num_grids; row++) {
        for (uint8_t col = 0; col < grids_per_row && grid_count < num_grids; col++) {
            grid_db[grid_count].grid_id = grid_count + 1;
            grid_db[grid_count].center_x = start_x + (col + 0.5) * grid_size;
            grid_db[grid_count].center_y = start_y + (row + 0.5) * grid_size;
            grid_db[grid_count].grid_status = 0; /* Initially uncovered */
            
            LOG_INFO("Grid_%d: center(%d, %d), status: uncovered\n", 
                     grid_db[grid_count].grid_id,
                     grid_db[grid_count].center_x,
                     grid_db[grid_count].center_y);
            grid_count++;
        }
    }
    
    num_grids = grid_count;
    LOG_INFO("Created %d grids in Grid_DB for LA_%d\n", num_grids, current_la_id);
}

/*---------------------------------------------------------------------------*/
/* Find nearest uncovered grid from current position */
static uint8_t find_nearest_uncovered_grid(void)
{
    uint8_t nearest_grid = 0;
    double min_distance = INFINITY;
    bool found = false;
    
    for (uint8_t i = 0; i < num_grids; i++) {
        if (grid_db[i].grid_status == 0) { /* Uncovered grid */
            position_t grid_pos = {grid_db[i].center_x, grid_db[i].center_y};
            double distance = calculate_distance(current_position, grid_pos);
            
            if (distance < min_distance) {
                min_distance = distance;
                nearest_grid = i;
                found = true;
            }
        }
    }
    
    if (found) {
        LOG_INFO("Nearest uncovered grid: Grid_%d at distance %.2f\n", 
                 grid_db[nearest_grid].grid_id, min_distance);
    } else {
        LOG_INFO("No uncovered grids found\n");
    }
    
    return nearest_grid;
}

/*---------------------------------------------------------------------------*/
/* Move robot to specified position */
static void move_robot_to_position(position_t target)
{
    double distance = calculate_distance(current_position, target);
    
    LOG_INFO("Moving from (%d, %d) to (%d, %d), distance: %.2f m\n",
             current_position.x, current_position.y, target.x, target.y, distance);
    
    /* Update position */
    current_position = target;
    
    /* Calculate and add mobility energy */
    add_mobility_energy(distance);
}

/*---------------------------------------------------------------------------*/
/* Check if grid has sensors (simulated) */
static uint8_t count_sensors_in_grid(uint8_t grid_idx)
{
    /* For simulation, randomly determine if grid has sensors */
    uint8_t sensor_count = 0;
    position_t grid_center = {grid_db[grid_idx].center_x, grid_db[grid_idx].center_y};
    
    /* Check existing sensors in database */
    for (uint8_t i = 0; i < num_sensors; i++) {
        position_t sensor_pos = {sensor_db[i].x_coord, sensor_db[i].y_coord};
        double distance = calculate_distance(grid_center, sensor_pos);
        
        if (distance <= SENSOR_PERCEPTION_RANGE) {
            sensor_count++;
        }
    }
    
    /* Add some random sensors for simulation */
    if (sensor_count == 0 && (random_rand() % 100) < 30) { /* 30% chance */
        sensor_count = 1 + (random_rand() % 3); /* 1-3 sensors */
    }
    
    return sensor_count;
}

/*---------------------------------------------------------------------------*/
/* Topology Discovery Phase - Algorithm 3 */
static void topology_discovery_phase(position_t la_center)
{
    LOG_INFO("=== TOPOLOGY DISCOVERY PHASE ===\n");
    LOG_INFO("Robot_%d moves to center of LA_%d\n", robot_id, current_la_id);
    
    /* Step 1: Move Robot_p to centre of LA_q */
    move_robot_to_position(la_center);
    
    /* Step 2: Broadcast message Mp to discover sensors */
    robot_broadcast_msg_t broadcast_msg;
    broadcast_msg.msg_type = WSN_MSG_TYPE_ROBOT_BROADCAST;
    broadcast_msg.robot_id = robot_id;
    
    uip_ipaddr_t dest_ipaddr;
    uip_create_linklocal_allnodes_mcast(&dest_ipaddr);
    
    simple_udp_sendto(&udp_conn, &broadcast_msg, sizeof(broadcast_msg), &dest_ipaddr);
    
    LOG_INFO("Robot_%d broadcasts Mp message for sensor discovery\n", robot_id);
    
    /* Step 3: Receive responses (Sensor_M) from sensors within perception range */
    /* Step 4: Update Sensor_DB with Sensor_M information */
    
    /* Initialize sensor database */
    num_sensors = 0;
    
    /* For simulation, populate with some sensors based on random deployment */
    uint8_t discovered_sensors = 5 + (random_rand() % 6); /* 5-10 sensors */
    
    for (uint8_t i = 0; i < discovered_sensors && i < WSN_DEPLOYMENT_CONF_MAX_SENSORS; i++) {
        sensor_db[i].sensor_id = i + 1;
        /* Random position within LA */
        sensor_db[i].x_coord = la_center.x + ((random_rand() % ROBOT_PERCEPTION_RANGE) - ROBOT_PERCEPTION_RANGE/2);
        sensor_db[i].y_coord = la_center.y + ((random_rand() % ROBOT_PERCEPTION_RANGE) - ROBOT_PERCEPTION_RANGE/2);
        sensor_db[i].sensor_status = 0; /* Initially idle */
        num_sensors++;
        
        LOG_INFO("Discovered Sensor_%d at (%d, %d), status: idle\n",
                 sensor_db[i].sensor_id, sensor_db[i].x_coord, sensor_db[i].y_coord);
    }
    
    LOG_INFO("Topology discovery completed. Found %d sensors in Sensor_DB\n", num_sensors);
    LOG_INFO("=== END TOPOLOGY DISCOVERY PHASE ===\n");
}

/*---------------------------------------------------------------------------*/
/* Dispersion Phase Case 1: Robot has sensors and grid has sensors */
static void dispersion_case1(uint8_t grid_idx)
{
    LOG_INFO("=== DISPERSION CASE 1 ===\n");
    LOG_INFO("Robot has sensors (Stock_RS=%d) AND Grid_%d has sensors\n", 
             stock_rs, grid_db[grid_idx].grid_id);
    
    position_t grid_center = {grid_db[grid_idx].center_x, grid_db[grid_idx].center_y};
    
    /* Place a sensor from Stock_RS at the centre of Grid_i */
    if (stock_rs > 0) {
        /* Add active sensor record to Sensor_DB */
        sensor_db[num_sensors].sensor_id = 100 + num_sensors; /* Different ID range for deployed sensors */
        sensor_db[num_sensors].x_coord = grid_center.x;
        sensor_db[num_sensors].y_coord = grid_center.y;
        sensor_db[num_sensors].sensor_status = 1; /* Active */
        num_sensors++;
        
        stock_rs--; /* Remove sensor from stock */
        
        LOG_INFO("Placed active sensor at grid center (%d, %d)\n", grid_center.x, grid_center.y);
        
        /* Collect all extra sensors from Grid_i till Stock_RS < 15 */
        uint8_t extra_sensors = count_sensors_in_grid(grid_idx);
        uint8_t collected = 0;
        
        while (extra_sensors > 0 && stock_rs < ROBOT_CAPACITY) {
            stock_rs++;
            extra_sensors--;
            collected++;
        }
        
        if (collected > 0) {
            LOG_INFO("Collected %d extra sensors, Stock_RS now: %d\n", collected, stock_rs);
        }
        
        /* Consider Grid_i as covered */
        grid_db[grid_idx].grid_status = 1;
        
        LOG_INFO("Grid_%d marked as COVERED\n", grid_db[grid_idx].grid_id);
    }
    
    LOG_INFO("=== END CASE 1 ===\n");
}

/*---------------------------------------------------------------------------*/
/* Dispersion Phase Case 2: Robot has sensors but grid has no sensors */
static void dispersion_case2(uint8_t grid_idx)
{
    LOG_INFO("=== DISPERSION CASE 2 ===\n");
    LOG_INFO("Robot has sensors (Stock_RS=%d) BUT Grid_%d has NO sensors\n", 
             stock_rs, grid_db[grid_idx].grid_id);
    
    position_t grid_center = {grid_db[grid_idx].center_x, grid_db[grid_idx].center_y};
    
    /* Place a sensor from Stock_RS at the centre of Grid_i */
    if (stock_rs > 0) {
        /* Add active sensor record to Sensor_DB */
        sensor_db[num_sensors].sensor_id = 100 + num_sensors;
        sensor_db[num_sensors].x_coord = grid_center.x;
        sensor_db[num_sensors].y_coord = grid_center.y;
        sensor_db[num_sensors].sensor_status = 1; /* Active */
        num_sensors++;
        
        stock_rs--; /* Remove sensor from stock */
        
        LOG_INFO("Placed active sensor at grid center (%d, %d)\n", grid_center.x, grid_center.y);
        
        /* Mark grid as covered */
        grid_db[grid_idx].grid_status = 1;
        
        LOG_INFO("Grid_%d marked as COVERED\n", grid_db[grid_idx].grid_id);
    }
    
    LOG_INFO("=== END CASE 2 ===\n");
}

/*---------------------------------------------------------------------------*/
/* Dispersion Phase Case 3: Robot has no sensors but grid has sensors */
static void dispersion_case3(uint8_t grid_idx)
{
    LOG_INFO("=== DISPERSION CASE 3 ===\n");
    LOG_INFO("Robot has NO sensors (Stock_RS=%d) BUT Grid_%d has sensors\n", 
             stock_rs, grid_db[grid_idx].grid_id);
    
    position_t grid_center = {grid_db[grid_idx].center_x, grid_db[grid_idx].center_y};
    
    /* Find closest sensor to grid center and move it there */
    uint8_t closest_sensor = 0;
    double min_distance = INFINITY;
    bool found_sensor = false;
    
    for (uint8_t i = 0; i < num_sensors; i++) {
        if (sensor_db[i].sensor_status == 0) { /* Idle sensor */
            position_t sensor_pos = {sensor_db[i].x_coord, sensor_db[i].y_coord};
            double distance = calculate_distance(grid_center, sensor_pos);
            
            if (distance < min_distance && distance <= ROBOT_PERCEPTION_RANGE) {
                min_distance = distance;
                closest_sensor = i;
                found_sensor = true;
            }
        }
    }
    
    if (found_sensor) {
        /* Update sensor position to grid center and make it active */
        LOG_INFO("Moving closest sensor (%.2f m away) to grid center\n", min_distance);
        
        sensor_db[closest_sensor].x_coord = grid_center.x;
        sensor_db[closest_sensor].y_coord = grid_center.y;
        sensor_db[closest_sensor].sensor_status = 1; /* Active */
        
        /* Collect extra sensors from grid */
        uint8_t extra_sensors = count_sensors_in_grid(grid_idx) - 1; /* Minus the one we just moved */
        uint8_t collected = 0;
        
        while (extra_sensors > 0 && stock_rs < ROBOT_CAPACITY) {
            stock_rs++;
            extra_sensors--;
            collected++;
        }
        
        if (collected > 0) {
            LOG_INFO("Collected %d extra sensors, Stock_RS now: %d\n", collected, stock_rs);
        }
        
        /* Mark grid as covered */
        grid_db[grid_idx].grid_status = 1;
        
        LOG_INFO("Grid_%d marked as COVERED\n", grid_db[grid_idx].grid_id);
    } else {
        LOG_INFO("No suitable sensor found for relocation\n");
    }
    
    LOG_INFO("=== END CASE 3 ===\n");
}

/*---------------------------------------------------------------------------*/
/* Dispersion Phase Case 4: Robot has no sensors and grid has no sensors */
static void dispersion_case4(uint8_t grid_idx)
{
    LOG_INFO("=== DISPERSION CASE 4 ===\n");
    LOG_INFO("Robot has NO sensors (Stock_RS=%d) AND Grid_%d has NO sensors\n", 
             stock_rs, grid_db[grid_idx].grid_id);
    
    /* Grid remains uncovered */
    LOG_INFO("Grid_%d remains UNCOVERED\n", grid_db[grid_idx].grid_id);
    
    LOG_INFO("=== END CASE 4 ===\n");
}

/*---------------------------------------------------------------------------*/
/* Execute Dispersion Phase - Algorithm 4 */
static void dispersion_phase(void)
{
    LOG_INFO("=== DISPERSION PHASE ===\n");
    LOG_INFO("Starting with NO_P = %d (permissible moves)\n", no_p);
    
    uint8_t current_grid = 0; /* Start with first grid */
    
    /* Algorithm 4: while NO_P > 0 do */
    while (no_p > 0) {
        LOG_INFO("\n--- Move %d (NO_P = %d) ---\n", (NO_G - no_p + 1), no_p);
        
        /* Move to current grid */
        position_t grid_pos = {grid_db[current_grid].center_x, grid_db[current_grid].center_y};
        move_robot_to_position(grid_pos);
        
        /* Determine which case applies */
        uint8_t sensors_in_grid = count_sensors_in_grid(current_grid);
        bool robot_has_sensors = (stock_rs > 0);
        bool grid_has_sensors = (sensors_in_grid > 0);
        
        LOG_INFO("At Grid_%d: Robot sensors=%d, Grid sensors=%d\n", 
                 grid_db[current_grid].grid_id, stock_rs, sensors_in_grid);
        
        /* Execute appropriate case */
        if (robot_has_sensors && grid_has_sensors) {
            dispersion_case1(current_grid);
        } else if (robot_has_sensors && !grid_has_sensors) {
            dispersion_case2(current_grid);
        } else if (!robot_has_sensors && grid_has_sensors) {
            dispersion_case3(current_grid);
        } else {
            dispersion_case4(current_grid);
        }
        
        /* Decrement NO_P by 1 */
        no_p--;
        
        /* Find nearest uncovered grid for next iteration */
        if (no_p > 0) {
            current_grid = find_nearest_uncovered_grid();
        }
        
        LOG_INFO("Moves remaining: %d\n", no_p);
    }
    
    LOG_INFO("=== END DISPERSION PHASE ===\n");
}

/*---------------------------------------------------------------------------*/
/* Count covered grids and send report to base station */
static void send_report_to_bs(void)
{
    uint8_t covered_grids = 0;
    
    /* Search Grid_DB for records where Grid_Status = 1 */
    for (uint8_t i = 0; i < num_grids; i++) {
        if (grid_db[i].grid_status == 1) {
            covered_grids++;
        }
    }
    
    /* Calculate total energy using paper's formula */
    total_energy_consumed = calculate_total_robot_energy();
    
    LOG_INFO("=== LOCAL PHASE COMPLETE ===\n");
    LOG_INFO("Covered grids in LA_%d: %d out of %d\n", current_la_id, covered_grids, num_grids);
    LOG_INFO("Total energy consumed: %.4f J\n", total_energy_consumed);
    LOG_INFO("Distance traveled: %.2f m\n", distance_traveled);
    
    /* Send Robot_pM message to BS */
    robot_report_msg_t report;
    report.msg_type = WSN_MSG_TYPE_ROBOT_REPORT;
    report.robot_id = robot_id;
    report.covered_grids = covered_grids;
    
    /* Send to base station (node ID 1) */
    uip_ipaddr_t bs_addr;
    uip_ip6addr(&bs_addr, 0xfe80, 0, 0, 0, 0x0201, 0x0001, 0x0001, 0x0001);
    
    /* Send multiple times to ensure delivery */
    for (int attempts = 0; attempts < 3; attempts++) {
        simple_udp_sendto(&udp_conn, &report, sizeof(report), &bs_addr);
        clock_delay(CLOCK_SECOND / 10);
    }
    
    LOG_INFO("Sent Robot_%dM report to BS: (%d, %d)\n", robot_id, robot_id, covered_grids);
    
    /* Print energy breakdown */
    LOG_INFO("Energy breakdown:\n");
    LOG_INFO("  Baseline: %.4f J\n", baseline_energy_robot);
    LOG_INFO("  Radio: %.4f J\n", radio_energy_robot);
    LOG_INFO("  Mobility: %.4f J\n", mobility_energy);
    LOG_INFO("  Total: %.4f J\n", total_energy_consumed);
}

/*---------------------------------------------------------------------------*/
/* Execute local phase for assigned location area - Algorithm 2 */
static void execute_local_phase(uint8_t la_id, position_t la_center)
{
    LOG_INFO("\n=== LOCAL PHASE START ===\n");
    LOG_INFO("Robot_%d executing local phase in LA_%d\n", robot_id, la_id);
    
    current_la_id = la_id;
    local_phase_active = true;
    
    /* Algorithm 2 Step 1: Divide LA_q into NO_G grids */
    init_grid_db(la_center);
    
    /* Algorithm 2 Step 3: Topology Discovery Phase */
    topology_discovery_phase(la_center);
    
    /* Algorithm 2 Step 4: Initialize NO_P = NO_G */
    no_p = num_grids; /* Number of permissible moves */
    LOG_INFO("Initialized NO_P = %d (number of permissible moves)\n", no_p);
    
    /* Algorithm 2 Step 5: Dispersion Phase */
    dispersion_phase();
    
    /* Algorithm 2 Step 6-8: Count covered grids and send report */
    send_report_to_bs();
    
    /* Reset NO_P to initial value for next LA */
    no_p = num_grids;
    local_phase_active = false;
    
    LOG_INFO("=== LOCAL PHASE END ===\n");
}

/*---------------------------------------------------------------------------*/
/* Handle LA assignment from base station */
static void handle_la_assignment(uint8_t la_id, int16_t center_x, int16_t center_y)
{
    LOG_INFO("Robot_%d assigned to LA_%d at center (%d, %d)\n", 
             robot_id, la_id, center_x, center_y);
    
    position_t la_center = {center_x, center_y};
    execute_local_phase(la_id, la_center);
}

/*---------------------------------------------------------------------------*/
/* UDP callback function with enhanced debugging - FIXED VERSION */
static void udp_rx_callback(struct simple_udp_connection *c,
                           const uip_ipaddr_t *sender_addr,
                           uint16_t sender_port,
                           const uip_ipaddr_t *receiver_addr,
                           uint16_t receiver_port,
                           const uint8_t *data,
                           uint16_t datalen)
{
    LOG_INFO("Robot_%d received UDP message from port %d, length: %d\n", 
             robot_id, sender_port, datalen);
    
    /* Print first few bytes for debugging */
    if (datalen > 0) {
        LOG_INFO("Message content: [0x%02x", data[0]);
        for (int i = 1; i < MIN(datalen, 8); i++) {
            LOG_INFO(" 0x%02x", data[i]);
        }
        LOG_INFO("]\n");
    }
    
    /* Handle LA assignment from base station */
    if (datalen == sizeof(la_assignment_msg_t)) {
        la_assignment_msg_t *assignment = (la_assignment_msg_t *)data;
        
        LOG_INFO("Parsed: msg_type=%d, robot_id=%d, la_id=%d, center=(%d,%d)\n",
                 assignment->msg_type, assignment->robot_id, assignment->la_id,
                 assignment->center_x, assignment->center_y);
        
        if (assignment->msg_type == WSN_MSG_TYPE_LA_ASSIGNMENT) {
            /* Accept assignment for this robot or broadcast (robot_id = 0) */
            if (assignment->robot_id == robot_id || assignment->robot_id == 0) {
                LOG_INFO("*** Robot_%d ACCEPTING LA assignment: LA_%d at (%d, %d) ***\n",
                         robot_id, assignment->la_id, assignment->center_x, assignment->center_y);
                
                assignment_received = true;
                pending_assignment = *assignment;
                
                /* Trigger immediate processing */
                process_post(&mobile_robot_process, PROCESS_EVENT_CONTINUE, assignment);
                
            } else {
                LOG_INFO("Robot_%d ignoring assignment for Robot_%d\n", 
                         robot_id, assignment->robot_id);
            }
        } else {
            LOG_INFO("Robot_%d: unexpected message type %d\n", robot_id, assignment->msg_type);
        }
    }
    /* Handle sensor replies during topology discovery */
    else if (datalen == sizeof(sensor_reply_msg_t)) {
        sensor_reply_msg_t *reply = (sensor_reply_msg_t *)data;
        
        if (reply->msg_type == WSN_MSG_TYPE_SENSOR_REPLY && local_phase_active) {
            /* Add sensor to database if within range */
            if (num_sensors < WSN_DEPLOYMENT_CONF_MAX_SENSORS) {
                sensor_db[num_sensors].sensor_id = reply->sensor_id;
                sensor_db[num_sensors].x_coord = reply->x_coord;
                sensor_db[num_sensors].y_coord = reply->y_coord;
                sensor_db[num_sensors].sensor_status = reply->sensor_status;
                num_sensors++;
                
                LOG_INFO("Added Sensor_%d to Sensor_DB: (%d, %d), status: %d\n",
                         reply->sensor_id, reply->x_coord, reply->y_coord, reply->sensor_status);
            }
        }
    } else {
        LOG_INFO("Robot_%d: unexpected message size %d (expected %d)\n", 
                 robot_id, datalen, (int)sizeof(la_assignment_msg_t));
    }
}

/*---------------------------------------------------------------------------*/
/* Print robot statistics */
static void print_robot_statistics(void)
{
    LOG_INFO("=== ROBOT STATISTICS ===\n");
    LOG_INFO("Robot ID: %d\n", robot_id);
    LOG_INFO("Current Position: (%d, %d)\n", current_position.x, current_position.y);
    LOG_INFO("Current LA: %d\n", current_la_id);
    LOG_INFO("Sensor Stock: %d/%d\n", stock_rs, ROBOT_CAPACITY);
    LOG_INFO("Distance Traveled: %.2f m\n", distance_traveled);
    LOG_INFO("Total Energy: %.4f J\n", total_energy_consumed);
    LOG_INFO("  - Baseline: %.4f J\n", baseline_energy_robot);
    LOG_INFO("  - Radio: %.4f J\n", radio_energy_robot);
    LOG_INFO("  - Mobility: %.4f J\n", mobility_energy);
    LOG_INFO("Sensors in DB: %d\n", num_sensors);
    LOG_INFO("Grids in DB: %d\n", num_grids);
    LOG_INFO("Assignment received: %s\n", assignment_received ? "Yes" : "No");
}

/*---------------------------------------------------------------------------*/
/* Mobile Robot Process Thread */
PROCESS_THREAD(mobile_robot_process, ev, data)
{
    static struct etimer timer;
    
    PROCESS_BEGIN();
    
    /* Initialize robot */
    robot_id = node_id;
    
    /* Set initial position based on node ID */
    if (robot_id == 2) {
        current_position.x = 433; 
        current_position.y = 531;
    } else if (robot_id == 3) {
        current_position.x = 500;
        current_position.y = 500;
    } else {
        /* Default position for other robots */
        current_position.x = 500;
        current_position.y = 500;
    }
    
    energy_last_update = clock_time();
    
    LOG_INFO("Starting Mobile Robot_%d\n", robot_id);
    LOG_INFO("Robot_%d initial position: (%d, %d)\n", robot_id, current_position.x, current_position.y);
    LOG_INFO("Initial sensor stock: %d\n", stock_rs);
    
    /* Initialize UDP connection */
    simple_udp_register(&udp_conn, UDP_SERVER_PORT, NULL,
                       UDP_CLIENT_PORT, udp_rx_callback);
    
    LOG_INFO("Robot_%d UDP connection registered - listening on port %d, sending to port %d\n",
             robot_id, UDP_SERVER_PORT, UDP_CLIENT_PORT);
    
    /* Wait for network to stabilize */
    etimer_set(&timer, 15 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    
    LOG_INFO("Robot_%d ready and waiting for LA assignment from BS\n", robot_id);
    
    /* Set up periodic statistics reporting */
    etimer_set(&timer, 30 * CLOCK_SECOND);
    
    while (1) {
        PROCESS_WAIT_EVENT();
        
        if (ev == PROCESS_EVENT_TIMER && data == &timer) {
            update_robot_energy();
            print_robot_statistics();
            
            /* Reset timer */
            etimer_set(&timer, 30 * CLOCK_SECOND);
        }
        else if (ev == PROCESS_EVENT_CONTINUE) {
            /* Handle LA assignment from base station */
            if (data != NULL) {
                la_assignment_msg_t *assignment = (la_assignment_msg_t *)data;
                
                if (assignment->msg_type == WSN_MSG_TYPE_LA_ASSIGNMENT) {
                    LOG_INFO("Robot_%d processing LA assignment in main process\n", robot_id);
                    handle_la_assignment(assignment->la_id, assignment->center_x, assignment->center_y);
                }
            }
        }
    }
    
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/