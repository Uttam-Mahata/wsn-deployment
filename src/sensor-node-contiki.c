/*
 * WSN Deployment Sensor Node - Contiki-NG Implementation
 * Based on main.tex specifications for sensor behavior in local phase
 */

#include "common.h"

// Sensor node state
typedef struct {
    uint32_t sensor_id;
    coordinate_t position;
    sensor_status_t status;
    float battery_level;
    uint8_t is_active;
    uint8_t can_be_relocated;
} sensor_node_state_t;

static sensor_node_state_t sensor_state;
static struct simple_udp_connection udp_conn;

// Contiki processes
PROCESS(sensor_node_process, "WSN Deployment Sensor Node Process");
PROCESS(sensing_process, "Sensor Sensing Process");
AUTOSTART_PROCESSES(&sensor_node_process, &sensing_process);

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
    
    if (msg->type == MSG_ROBOT_BROADCAST) {
        robot_broadcast_message_t *broadcast = (robot_broadcast_message_t *)msg->data;
        
        LOG_INFO("Sensor_%u received Robot_%d broadcast\n", 
               sensor_state.sensor_id, broadcast->robot_id + 1);
        
        // Check if robot is within communication range (simplified)
        // In real implementation, this would be based on actual network topology
        
        // Send reply with sensor information
        send_sensor_reply(broadcast->robot_id, sender_addr);
    }
    
    if (msg->type == MSG_DEPLOYMENT_DATA) {
        // Handle sensor relocation commands from robots
        coordinate_t *new_position = (coordinate_t *)msg->data;
        
        LOG_INFO("Sensor_%u received relocation command to (%.2f, %.2f)\n", 
               sensor_state.sensor_id, new_position->x, new_position->y);
        
        if (sensor_state.can_be_relocated) {
            relocate_sensor_contiki(*new_position);
        }
    }
}

// Initialize sensor node
void initialize_sensor_node_contiki() {
    // Generate sensor ID based on node address
    sensor_state.sensor_id = linkaddr_node_addr.u8[6] * 256 + linkaddr_node_addr.u8[7];
    
    // Generate random initial position within target area
    random_init(sensor_state.sensor_id);
    sensor_state.position.x = ((float)(random_rand() % 10000) / 10000.0) * target_area_size;
    sensor_state.position.y = ((float)(random_rand() % 10000) / 10000.0) * target_area_size;
    
    // Initialize sensor state
    sensor_state.status = IDLE;
    sensor_state.battery_level = 100.0;
    sensor_state.is_active = 1;
    sensor_state.can_be_relocated = 1;
    
    LOG_INFO("Sensor_%u initialized at position (%.2f, %.2f)\n", 
           sensor_state.sensor_id, sensor_state.position.x, sensor_state.position.y);
}

// Send sensor reply to robot broadcast
void send_sensor_reply(robot_id_t robot_id, const uip_ipaddr_t *robot_addr) {
    network_message_t msg;
    sensor_reply_message_t reply_data;
    
    reply_data.sensor_id = sensor_state.sensor_id;
    reply_data.coord = sensor_state.position;
    reply_data.sensor_status = sensor_state.status;
    
    msg.type = MSG_SENSOR_REPLY;
    msg.length = sizeof(sensor_reply_message_t);
    memcpy(msg.data, &reply_data, sizeof(sensor_reply_message_t));
    
    simple_udp_sendto(&udp_conn, &msg, sizeof(msg), robot_addr);
    
    LOG_INFO("Sensor_%u replied to Robot_%d: Position(%.2f, %.2f), Status: %s\n", 
           sensor_state.sensor_id, robot_id + 1,
           sensor_state.position.x, sensor_state.position.y,
           sensor_state.status == IDLE ? "IDLE" : "ACTIVE");
}

// Update sensor status
void update_sensor_status_contiki(sensor_status_t new_status) {
    sensor_status_t old_status = sensor_state.status;
    sensor_state.status = new_status;
    
    LOG_INFO("Sensor_%u status updated: %s -> %s\n", 
           sensor_state.sensor_id,
           old_status == IDLE ? "IDLE" : "ACTIVE",
           new_status == IDLE ? "IDLE" : "ACTIVE");
    
    // Adjust battery consumption based on status
    if (new_status == ACTIVE) {
        sensor_state.battery_level -= 1.0;  // Activation cost
    }
}

// Relocate sensor to new position
void relocate_sensor_contiki(coordinate_t new_position) {
    coordinate_t old_position = sensor_state.position;
    sensor_state.position = new_position;
    
    LOG_INFO("Sensor_%u relocated: (%.2f, %.2f) -> (%.2f, %.2f)\n", 
           sensor_state.sensor_id, old_position.x, old_position.y,
           new_position.x, new_position.y);
    
    // Relocation consumes battery
    sensor_state.battery_level -= 2.0;
    
    if (sensor_state.battery_level <= 0) {
        sensor_state.is_active = 0;
        LOG_INFO("Sensor_%u battery depleted after relocation - sensor inactive\n", 
               sensor_state.sensor_id);
    }
}

// Simulate sensor sensing within its range
void sense_environment_contiki() {
    if (sensor_state.status == ACTIVE && sensor_state.is_active) {
        LOG_INFO("Sensor_%u sensing environment at (%.2f, %.2f)\n", 
               sensor_state.sensor_id, sensor_state.position.x, sensor_state.position.y);
        
        // Simulate battery consumption during sensing
        sensor_state.battery_level -= 0.1;
        
        if (sensor_state.battery_level <= 0) {
            sensor_state.is_active = 0;
            LOG_INFO("Sensor_%u battery depleted - sensor inactive\n", sensor_state.sensor_id);
        }
        
        // Simulate event detection (simplified)
        if ((random_rand() % 100) < 5) {  // 5% chance of detecting an event
            LOG_INFO("Sensor_%u detected an event!\n", sensor_state.sensor_id);
            report_event_detection();
        }
    }
}

// Report event detection to network
void report_event_detection() {
    network_message_t msg;
    coordinate_t event_location = sensor_state.position;
    
    msg.type = MSG_DEPLOYMENT_DATA;  // Reuse for event reporting
    msg.length = sizeof(coordinate_t);
    memcpy(msg.data, &event_location, sizeof(coordinate_t));
    
    // Broadcast event to network (simplified)
    uip_ipaddr_t broadcast_addr;
    uip_create_linklocal_allnodes_mcast(&broadcast_addr);
    simple_udp_sendto(&udp_conn, &msg, sizeof(msg), &broadcast_addr);
    
    LOG_INFO("Sensor_%u reported event at (%.2f, %.2f)\n", 
           sensor_state.sensor_id, event_location.x, event_location.y);
}

// Check if sensor can detect an event at given position
int can_detect_event_contiki(coordinate_t event_position) {
    if (sensor_state.status == ACTIVE && sensor_state.is_active) {
        float distance = calculate_distance(sensor_state.position, event_position);
        
        if (distance <= sensor_perception_range) {
            LOG_INFO("Sensor_%u can detect event at (%.2f, %.2f), distance: %.2f\n", 
                   sensor_state.sensor_id, event_position.x, event_position.y, distance);
            return 1;
        }
    }
    return 0;
}

// Get sensor information
sensor_db_record_t get_sensor_info_contiki() {
    sensor_db_record_t info;
    info.sensor_id = sensor_state.sensor_id;
    info.coord = sensor_state.position;
    info.sensor_status = sensor_state.status;
    return info;
}

// Display sensor state
void display_sensor_state_contiki() {
    LOG_INFO("=== Sensor_%u State ===\n", sensor_state.sensor_id);
    LOG_INFO("Position: (%.2f, %.2f)\n", sensor_state.position.x, sensor_state.position.y);
    LOG_INFO("Status: %s\n", sensor_state.status == IDLE ? "IDLE" : "ACTIVE");
    LOG_INFO("Battery: %.1f%%\n", sensor_state.battery_level);
    LOG_INFO("Active: %s\n", sensor_state.is_active ? "YES" : "NO");
    LOG_INFO("Relocatable: %s\n", sensor_state.can_be_relocated ? "YES" : "NO");
}

// Check sensor coverage area
float calculate_sensor_coverage_area() {
    if (sensor_state.status == ACTIVE && sensor_state.is_active) {
        return M_PI * sensor_perception_range * sensor_perception_range;
    }
    return 0.0;
}

// Simulate sensor network operation
void simulate_sensor_operation() {
    if (sensor_state.is_active) {
        // Perform sensing if active
        if (sensor_state.status == ACTIVE) {
            sense_environment_contiki();
        }
        
        // Periodic battery drain
        sensor_state.battery_level -= 0.01;  // Idle consumption
        
        if (sensor_state.battery_level <= 0) {
            sensor_state.is_active = 0;
            LOG_INFO("Sensor_%u battery depleted - going inactive\n", sensor_state.sensor_id);
        }
    }
}

// Main sensor node process
PROCESS_THREAD(sensor_node_process, ev, data)
{
    static struct etimer status_timer;
    
    PROCESS_BEGIN();
    
    LOG_INFO("=== WSN Deployment Sensor Node - Contiki-NG ===\n");
    
    // Initialize network
    simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL, UDP_SERVER_PORT, udp_rx_callback);
    
    // Initialize sensor node
    initialize_sensor_node_contiki();
    
    // Display initial status
    display_sensor_state_contiki();
    
    // Set periodic timer for status updates
    etimer_set(&status_timer, CLOCK_SECOND * 60);  // 60 second intervals
    
    while(1) {
        PROCESS_WAIT_EVENT();
        
        if(ev == PROCESS_EVENT_TIMER && data == &status_timer) {
            display_sensor_state_contiki();
            etimer_reset(&status_timer);
        }
    }
    
    PROCESS_END();
}

// Sensing process for continuous operation
PROCESS_THREAD(sensing_process, ev, data)
{
    static struct etimer sensing_timer;
    
    PROCESS_BEGIN();
    
    LOG_INFO("Sensor_%u sensing process started\n", sensor_state.sensor_id);
    
    // Set periodic timer for sensing operations
    etimer_set(&sensing_timer, CLOCK_SECOND * 10);  // 10 second intervals
    
    while(1) {
        PROCESS_WAIT_EVENT();
        
        if(ev == PROCESS_EVENT_TIMER && data == &sensing_timer) {
            // Perform sensor operations
            simulate_sensor_operation();
            
            // Reset timer
            etimer_reset(&sensing_timer);
        }
    }
    
    PROCESS_END();
}
