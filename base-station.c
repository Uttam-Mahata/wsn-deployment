#include "contiki.h"
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "sys/etimer.h"
#include "sys/clock.h"
#include "project-conf.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "sys/log.h"
#define LOG_MODULE "BaseStation"
#define LOG_LEVEL LOG_LEVEL_APP

/* Data Structures based on APP_I algorithm */
typedef struct {
    uint8_t la_id;
    uint16_t center_x;
    uint16_t center_y;
    uint8_t no_grid;
} la_db_record_t;

typedef struct {
    uint8_t robot_id;
    uint8_t assigned_la_id;
} robot_db_record_t;

typedef struct {
    uint8_t robot_id;
    uint8_t covered_grids;
} robot_message_t;

/* Add message structure at top level */
typedef struct {
    uint8_t target_robot_id;
    la_db_record_t la_assignment;
} robot_assignment_msg_t;

/* Base Station State */
static struct {
    la_db_record_t la_db[MAX_LOCATION_AREAS];
    robot_db_record_t robot_db[MAX_ROBOTS];
    uint8_t num_location_areas;
    uint8_t active_robots;
    
    /* Energy tracking */
    float total_energy_consumed;
    float processing_energy;
    float radio_energy;
    uint32_t messages_sent;
    uint32_t messages_received;
    uint32_t processing_operations;
    
    /* Timing */
    clock_time_t start_time;
    clock_time_t last_energy_calc;
} base_station;

static struct simple_udp_connection udp_conn;
static struct etimer energy_timer;

PROCESS(base_station_process, "Base Station Process");
AUTOSTART_PROCESSES(&base_station_process);

/* Energy Calculation Functions */
static float calculate_processing_energy(uint32_t operations, float time_elapsed) {
    return operations * P_PROCESSING_BASE * T_PROCESSING_BASE;
}

static float calculate_radio_energy(uint32_t tx_count, uint32_t rx_count, float avg_tx_time, float avg_rx_time) {
    float tx_energy = tx_count * P_TRANSMIT_BASE * avg_tx_time;
    float rx_energy = rx_count * P_RECEIVE_BASE * avg_rx_time;
    return tx_energy + rx_energy;
}

static void update_energy_consumption() {
    clock_time_t current_time = clock_time();
    float time_elapsed = (float)(current_time - base_station.last_energy_calc) / CLOCK_SECOND;
    
    /* Calculate processing energy */
    base_station.processing_energy += calculate_processing_energy(
        base_station.processing_operations, time_elapsed);
    
    /* Calculate radio energy (assuming average message times) */
    float avg_tx_time = 0.001; // 1ms average transmission time
    float avg_rx_time = 0.001; // 1ms average reception time
    
    base_station.radio_energy += calculate_radio_energy(
        base_station.messages_sent, base_station.messages_received,
        avg_tx_time, avg_rx_time);
    
    base_station.total_energy_consumed = base_station.processing_energy + base_station.radio_energy;
    base_station.last_energy_calc = current_time;
    
    /* Reset counters */
    base_station.processing_operations = 0;
    base_station.messages_sent = 0;
    base_station.messages_received = 0;
}

/* Database Operations */
static void initialize_la_db() {
    uint8_t la_count = 0;
    uint16_t area_per_la = ROBOT_PERCEPTION_RANGE;
    
    /* Calculate number of location areas */
    uint8_t las_x = TARGET_AREA_WIDTH / area_per_la;
    uint8_t las_y = TARGET_AREA_HEIGHT / area_per_la;
    base_station.num_location_areas = las_x * las_y;
    
    if (base_station.num_location_areas > MAX_LOCATION_AREAS) {
        base_station.num_location_areas = MAX_LOCATION_AREAS;
    }
    
    /* Initialize LA_DB records */
    for (uint8_t y = 0; y < las_y && la_count < MAX_LOCATION_AREAS; y++) {
        for (uint8_t x = 0; x < las_x && la_count < MAX_LOCATION_AREAS; x++) {
            base_station.la_db[la_count].la_id = la_count + 1;
            base_station.la_db[la_count].center_x = x * area_per_la + area_per_la / 2;
            base_station.la_db[la_count].center_y = y * area_per_la + area_per_la / 2;
            base_station.la_db[la_count].no_grid = 0; // Initially uncovered
            la_count++;
        }
    }
    
    LOG_INFO("Initialized %u location areas\n", base_station.num_location_areas);
    base_station.processing_operations++;
}

static int8_t find_uncovered_la() {
    for (uint8_t i = 0; i < base_station.num_location_areas; i++) {
        if (base_station.la_db[i].no_grid == 0) {
            return i;
        }
    }
    return -1; // No uncovered LA found
}

static void assign_robot_to_la(uint8_t robot_id, uint8_t la_index) {
    if (robot_id < MAX_ROBOTS) {
        base_station.robot_db[robot_id].robot_id = robot_id;
        base_station.robot_db[robot_id].assigned_la_id = base_station.la_db[la_index].la_id;
        
        LOG_INFO("Assigned Robot %u to LA %u at (%u, %u)\n", 
                robot_id, base_station.la_db[la_index].la_id,
                base_station.la_db[la_index].center_x, 
                base_station.la_db[la_index].center_y);
    }
    base_station.processing_operations++;
}

static void update_la_coverage(uint8_t robot_id, uint8_t covered_grids) {
    /* Find robot's assigned LA */
    uint8_t assigned_la_id = base_station.robot_db[robot_id].assigned_la_id;
    
    /* Update LA_DB */
    for (uint8_t i = 0; i < base_station.num_location_areas; i++) {
        if (base_station.la_db[i].la_id == assigned_la_id) {
            base_station.la_db[i].no_grid = covered_grids;
            LOG_INFO("Updated LA %u coverage: %u grids covered\n", assigned_la_id, covered_grids);
            break;
        }
    }
    base_station.processing_operations++;
}

static float calculate_area_coverage_percentage() {
    uint16_t total_grids = 0;
    uint16_t covered_grids = 0;
    uint8_t grids_per_la = (ROBOT_PERCEPTION_RANGE / SENSOR_PERCEPTION_RANGE) * 
                          (ROBOT_PERCEPTION_RANGE / SENSOR_PERCEPTION_RANGE);
    
    for (uint8_t i = 0; i < base_station.num_location_areas; i++) {
        total_grids += grids_per_la;
        covered_grids += base_station.la_db[i].no_grid;
    }
    
    return total_grids > 0 ? ((float)covered_grids / total_grids) * 100.0 : 0.0;
}

/* Communication Handlers */
static void udp_rx_callback(struct simple_udp_connection *c,
                           const uip_ipaddr_t *sender_addr,
                           uint16_t sender_port,
                           const uip_ipaddr_t *receiver_addr,
                           uint16_t receiver_port,
                           const uint8_t *data,
                           uint16_t datalen) {
    
    base_station.messages_received++;
    
    if (datalen == sizeof(robot_message_t)) {
        robot_message_t *msg = (robot_message_t *)data;
        
        LOG_INFO("Received Robot_%uM: (%u, %u) - coverage report\n", 
                msg->robot_id, msg->robot_id, msg->covered_grids);
        
        /* Update LA coverage */
        update_la_coverage(msg->robot_id, msg->covered_grids);
        
        /* Global Phase Algorithm: Search for next uncovered LA as per APP_I */
        int8_t next_la = find_uncovered_la();
        if (next_la >= 0) {
            /* Deploy robot to uncovered LA */
            assign_robot_to_la(msg->robot_id, next_la);
            
            robot_assignment_msg_t assignment_msg;
            assignment_msg.target_robot_id = msg->robot_id;
            assignment_msg.la_assignment = base_station.la_db[next_la];
            
            simple_udp_sendto(&udp_conn, &assignment_msg, sizeof(assignment_msg), sender_addr);
            base_station.messages_sent++;
            
            LOG_INFO("Global Phase: Assigned Robot %u to next uncovered LA %u\n", 
                    msg->robot_id, base_station.la_db[next_la].la_id);
        } else {
            /* No more uncovered LAs found - global phase complete for this robot */
            LOG_INFO("Global Phase: No uncovered LAs remain. Robot %u deployment complete.\n", msg->robot_id);
            
            /* Check if all robots are done */
            bool all_las_covered = true;
            for (uint8_t i = 0; i < base_station.num_location_areas; i++) {
                if (base_station.la_db[i].no_grid == 0) {
                    all_las_covered = false;
                    break;
                }
            }
            
            if (all_las_covered) {
                float coverage_percentage = calculate_area_coverage_percentage();
                LOG_INFO("=== APP_I GLOBAL PHASE COMPLETE ===\n");
                LOG_INFO("All location areas processed\n");
                LOG_INFO("Final area coverage: %.2f%%\n", coverage_percentage);
                LOG_INFO("Total robots deployed: %u\n", base_station.active_robots);
                LOG_INFO("===================================\n");
            }
        }
    }
}

static void deploy_initial_robots() {
    /* Deploy Robot 0 to first LA (LA_id_1) as per APP_I specification */
    uint8_t la_index = 0; // First LA (LA_id_1)
    assign_robot_to_la(0, la_index);
    
    robot_assignment_msg_t assignment_msg;
    assignment_msg.target_robot_id = 0;
    assignment_msg.la_assignment = base_station.la_db[la_index];
    
    /* Send to all robots (they will filter by robot ID) */
    uip_ipaddr_t robot_addr;
    uip_ip6addr(&robot_addr, 0xff02, 0, 0, 0, 0, 0, 0, 1);
    simple_udp_sendto(&udp_conn, &assignment_msg, sizeof(assignment_msg), &robot_addr);
    base_station.messages_sent++;
    
    LOG_INFO("Deployed Robot 0 to LA %u (first LA as per APP_I)\n", 
            base_station.la_db[la_index].la_id);
    
    /* Deploy Robot 1 to last LA (LA_id_NO_LA) as per APP_I specification */
    if (base_station.num_location_areas > 1) {
        la_index = base_station.num_location_areas - 1; // Last LA
        assign_robot_to_la(1, la_index);
        
        robot_assignment_msg_t assignment_msg;
        assignment_msg.target_robot_id = 1;
        assignment_msg.la_assignment = base_station.la_db[la_index];
        
        uip_ipaddr_t robot_addr;
        uip_ip6addr(&robot_addr, 0xff02, 0, 0, 0, 0, 0, 0, 1);
        simple_udp_sendto(&udp_conn, &assignment_msg, sizeof(assignment_msg), &robot_addr);
        base_station.messages_sent++;
        
        LOG_INFO("Deployed Robot 1 to LA %u (last LA as per APP_I)\n", 
                base_station.la_db[la_index].la_id);
    }
    
    base_station.active_robots = (base_station.num_location_areas > 1) ? 2 : 1;
    LOG_INFO("Initial deployment complete: %u robots deployed as per APP_I\n", base_station.active_robots);
}

static void print_energy_report() {
    update_energy_consumption();
    
    float coverage_percentage = calculate_area_coverage_percentage();
    clock_time_t elapsed = clock_time() - base_station.start_time;
    float elapsed_seconds = (float)elapsed / CLOCK_SECOND;
    
    LOG_INFO("=== ENERGY & COVERAGE REPORT ===\n");
    LOG_INFO("Elapsed time: %.2f seconds\n", elapsed_seconds);
    LOG_INFO("Area coverage: %.2f%%\n", coverage_percentage);
    LOG_INFO("Processing energy: %.6f J\n", base_station.processing_energy);
    LOG_INFO("Radio energy: %.6f J\n", base_station.radio_energy);
    LOG_INFO("Total base station energy: %.6f J\n", base_station.total_energy_consumed);
    LOG_INFO("==============================\n");
}

PROCESS_THREAD(base_station_process, ev, data) {
    PROCESS_BEGIN();
    
    /* Initialize base station */
    base_station.start_time = clock_time();
    base_station.last_energy_calc = base_station.start_time;
    memset(&base_station, 0, sizeof(base_station));
    base_station.start_time = clock_time();
    base_station.last_energy_calc = base_station.start_time;
    
    /* Initialize DAG root */
    NETSTACK_ROUTING.root_start();
    
    /* Initialize UDP connection */
    simple_udp_register(&udp_conn, UDP_SERVER_PORT, NULL, UDP_CLIENT_PORT, udp_rx_callback);
    
    /* Initialize databases */
    initialize_la_db();
    
    /* Deploy initial robots */
    deploy_initial_robots();
    
    /* Set energy reporting timer */
    etimer_set(&energy_timer, ENERGY_REPORT_INTERVAL);
    
    LOG_INFO("Base Station initialized. Managing %u location areas.\n", 
             base_station.num_location_areas);
    
    while(1) {
        PROCESS_WAIT_EVENT();
        
        if (ev == PROCESS_EVENT_TIMER && data == &energy_timer) {
            print_energy_report();
            etimer_reset(&energy_timer);
        }
    }
    
    PROCESS_END();
}
