/*
 * WSN Deployment Base Station - Contiki-NG Implementation
 * Based on main.tex specifications for APP_I global phase
 */

#include "common.h"

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

// Network connection
static struct simple_udp_connection udp_conn;

// Contiki processes
PROCESS(base_station_process, "WSN Deployment Base Station Process");
PROCESS(message_handler_process, "Base Station Message Handler");
AUTOSTART_PROCESSES(&base_station_process, &message_handler_process);

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
    
    LOG_INFO("Received message type %d from ", msg->type);
    LOG_INFO_6ADDR(sender_addr);
    LOG_INFO_("\n");
    
    if (msg->type == MSG_ROBOT_COMPLETION) {
        robot_message_t *robot_msg = (robot_message_t *)msg->data;
        
        LOG_INFO("Robot completion message: Robot_%d, NO_Grid=%u\n", 
                 robot_msg->robot_id + 1, robot_msg->no_grid);
        
        // Update LA_DB with received grid count
        for (uint32_t i = 0; i < bs.no_la; i++) {
            if (bs.la_db[i].la_id == bs.robot_db[robot_msg->robot_id].assigned_la_id) {
                bs.la_db[i].no_grid = robot_msg->no_grid;
                LOG_INFO("Updated LA_%u with NO_Grid = %u\n", 
                         bs.la_db[i].la_id, robot_msg->no_grid);
                break;
            }
        }
        
        // Find next deployment location
        uint32_t next_la_id = find_next_deployment_la();
        
        if (next_la_id > 0) {
            // Deploy robot in the identified LA
            bs.robot_db[robot_msg->robot_id].assigned_la_id = next_la_id;
            LOG_INFO("Deploying Robot_%d in LA_%u\n", 
                     robot_msg->robot_id + 1, next_la_id);
            
            // Send deployment command to robot
            send_deployment_command(robot_msg->robot_id, next_la_id, sender_addr);
        } else {
            LOG_INFO("No suitable LA found for Robot_%d deployment\n", 
                     robot_msg->robot_id + 1);
            stop_robot_deployment();
        }
    }
}

// Initialize Base Station
void initialize_base_station() {
    LOG_INFO("Initializing Base Station...\n");
    
    // Calculate number of location areas
    bs.no_la = (uint32_t)floor(target_area_size / robot_perception_range);
    bs.no_g = (uint32_t)floor(robot_perception_range / sensor_perception_range);
    
    LOG_INFO("Number of Location Areas (NO_LA): %u\n", bs.no_la);
    LOG_INFO("Number of Grids per LA (NO_G): %u\n", bs.no_g);
    
    // Allocate memory for databases
    bs.la_db = (la_db_record_t*)malloc(bs.no_la * sizeof(la_db_record_t));
    bs.robot_db = (robot_db_record_t*)malloc(MAX_ROBOTS * sizeof(robot_db_record_t));
    
    if (!bs.la_db || !bs.robot_db) {
        LOG_ERR("Error: Memory allocation failed\n");
        return;
    }
    
    // Calculate database sizes
    bs.size_la_db = calculate_la_db_size(bs.no_la, bs.no_g);
    bs.size_robot_db = calculate_robot_db_size(bs.no_la);
    
    LOG_INFO("LA_DB size: %u bits\n", bs.size_la_db);
    LOG_INFO("Robot_DB size: %u bits\n", bs.size_robot_db);
}

// Initialize Location Area Database
void initialize_la_db() {
    LOG_INFO("Initializing Location Area Database...\n");
    
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
        
        LOG_INFO("LA_%u: Center(%.2f, %.2f), NO_Grid: %u\n", 
               bs.la_db[i].la_id, 
               bs.la_db[i].center_coord.x, 
               bs.la_db[i].center_coord.y, 
               bs.la_db[i].no_grid);
    }
}

// Deploy robots in location areas
void deploy_robots() {
    LOG_INFO("Deploying robots in location areas...\n");
    
    // Deploy Robot_1 in LA_1 and Robot_2 in LA_NO_LA as per specification
    bs.robot_db[0].robot_id = ROBOT_1;
    bs.robot_db[0].assigned_la_id = 1;  // LA_1
    
    bs.robot_db[1].robot_id = ROBOT_2;
    bs.robot_db[1].assigned_la_id = bs.no_la;  // LA_NO_LA
    
    LOG_INFO("Robot_1 deployed in LA_%u\n", bs.robot_db[0].assigned_la_id);
    LOG_INFO("Robot_2 deployed in LA_%u\n", bs.robot_db[1].assigned_la_id);
    
    // Each robot is assigned STOCK_RS sensors
    LOG_INFO("Each robot assigned %d sensors from stock\n", STOCK_RS);
}

// Find next deployment location
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

// Send deployment command to robot
void send_deployment_command(robot_id_t robot_id, uint32_t la_id, const uip_ipaddr_t *robot_addr) {
    network_message_t msg;
    msg.type = MSG_BASE_COMMAND;
    msg.length = sizeof(uint32_t);
    memcpy(msg.data, &la_id, sizeof(uint32_t));
    
    simple_udp_sendto(&udp_conn, &msg, sizeof(msg), robot_addr);
    LOG_INFO("Sent deployment command to Robot_%d for LA_%u\n", robot_id + 1, la_id);
}

// Stop robot deployment and compute area coverage
void stop_robot_deployment() {
    LOG_INFO("Stopping robot deployment...\n");
    
    // Calculate total covered grids
    uint32_t total_covered_grids = 0;
    for (uint32_t i = 0; i < bs.no_la; i++) {
        total_covered_grids += bs.la_db[i].no_grid;
    }
    
    // Calculate percentage area coverage
    uint32_t total_grids = bs.no_g * bs.no_la;
    float per_ac = ((float)total_covered_grids / total_grids) * 100.0;
    
    LOG_INFO("Total covered grids: %u\n", total_covered_grids);
    LOG_INFO("Total grids: %u\n", total_grids);
    LOG_INFO("Percentage Area Coverage (Per_AC): %.2f%%\n", per_ac);
}

// Display database status
void display_database_status() {
    LOG_INFO("=== Base Station Database Status ===\n");
    
    LOG_INFO("Location Area Database (LA_DB):\n");
    for (uint32_t i = 0; i < bs.no_la; i++) {
        LOG_INFO("LA_%u: Center(%.2f, %.2f), NO_Grid: %u\n", 
               bs.la_db[i].la_id,
               bs.la_db[i].center_coord.x,
               bs.la_db[i].center_coord.y,
               bs.la_db[i].no_grid);
    }
    
    LOG_INFO("Robot Database (Robot_DB):\n");
    for (int i = 0; i < MAX_ROBOTS; i++) {
        LOG_INFO("R%d: Assigned_LA_ID = %u\n", 
               bs.robot_db[i].robot_id + 1,
               bs.robot_db[i].assigned_la_id);
    }
}

// Main base station process
PROCESS_THREAD(base_station_process, ev, data)
{
    static struct etimer timer;
    
    PROCESS_BEGIN();
    
    LOG_INFO("=== WSN Deployment Base Station - Contiki-NG ===\n");
    
    // Initialize network
    simple_udp_register(&udp_conn, UDP_SERVER_PORT, NULL, UDP_CLIENT_PORT, udp_rx_callback);
    
    // Initialize base station
    initialize_base_station();
    initialize_la_db();
    deploy_robots();
    
    // Display initial status
    display_database_status();
    
    // Set periodic timer for status updates
    etimer_set(&timer, CLOCK_SECOND * 30);  // 30 second intervals
    
    while(1) {
        PROCESS_WAIT_EVENT();
        
        if(ev == PROCESS_EVENT_TIMER && data == &timer) {
            LOG_INFO("Base Station Status Update\n");
            display_database_status();
            etimer_reset(&timer);
        }
    }
    
    PROCESS_END();
}

// Message handler process
PROCESS_THREAD(message_handler_process, ev, data)
{
    PROCESS_BEGIN();
    
    LOG_INFO("Base Station Message Handler started\n");
    
    while(1) {
        PROCESS_YIELD();
        // Message handling is done in UDP callback
    }
    
    PROCESS_END();
}
