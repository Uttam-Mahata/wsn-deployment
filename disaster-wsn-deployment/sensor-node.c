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
#define LOG_MODULE "SensorNode"
#define LOG_LEVEL LOG_LEVEL_APP

/* Sensor operational modes */
typedef enum {
    SENSOR_MODE_IDLE = 0,
    SENSOR_MODE_ACTIVE = 1
} sensor_mode_t;

/* Message structures */
typedef struct {
    uint8_t robot_id;
} robot_discovery_msg_t;

typedef struct {
    uint8_t sensor_id;
    uint16_t x_coord;
    uint16_t y_coord;
    uint8_t sensor_status;
} sensor_reply_msg_t;

/* Sensor Node State */
static struct {
    uint8_t sensor_id;
    uint16_t x_position;
    uint16_t y_position;
    sensor_mode_t current_mode;
    uint8_t is_deployed;  // 0 = randomly deployed, 1 = robot deployed
    
    /* Energy tracking */
    float total_energy_consumed;
    float baseline_energy;
    float sensing_energy;
    float processing_energy;
    float radio_energy;
    
    /* Operation counters for energy calculation */
    uint32_t sensing_operations;
    uint32_t processing_operations;
    uint32_t tx_operations;
    uint32_t rx_operations;
    uint32_t mode_switches;
    
    /* Timing */
    clock_time_t start_time;
    clock_time_t last_energy_calc;
    clock_time_t mode_start_time;
    clock_time_t last_sensing_time;
    
    /* Communication */
    uip_ipaddr_t robot_addr;
    uint8_t robot_in_range;
} sensor_node;

static struct simple_udp_connection udp_conn;
static struct etimer sensing_timer;
static struct etimer energy_timer;
static struct etimer mode_timer;

PROCESS(sensor_node_process, "Sensor Node Process");
AUTOSTART_PROCESSES(&sensor_node_process);

/* Energy Calculation Functions */
static float calculate_baseline_energy(float time_duration) {
    return time_duration * P_BASELINE_SENSOR;
}

static float calculate_sensing_energy(uint32_t sensing_ops) {
    // E_sensing = μ * r_i^2 (from LaTeX document)
    float sensing_range_sq = SENSOR_PERCEPTION_RANGE * SENSOR_PERCEPTION_RANGE;
    return sensing_ops * MU_SENSING * sensing_range_sq;
}

static float calculate_processing_energy(uint32_t processing_ops, float processing_time) {
    return processing_ops * P_PROCESSING_SENSOR * processing_time;
}

static float calculate_radio_energy(uint32_t tx_ops, uint32_t rx_ops, float avg_tx_time, float avg_rx_time) {
    float tx_energy = tx_ops * P_TRANSMIT_SENSOR * avg_tx_time;
    float rx_energy = rx_ops * P_RECEIVE_SENSOR * avg_rx_time;
    return tx_energy + rx_energy;
}

static void update_energy_consumption() {
    clock_time_t current_time = clock_time();
    float time_elapsed = (float)(current_time - sensor_node.last_energy_calc) / CLOCK_SECOND;
    
    /* Calculate baseline energy based on time in current mode */
    sensor_node.baseline_energy += calculate_baseline_energy(time_elapsed);
    
    /* Calculate sensing energy */
    sensor_node.sensing_energy += calculate_sensing_energy(sensor_node.sensing_operations);
    
    /* Calculate processing energy */
    float avg_processing_time = 0.001; // 1ms average processing time
    sensor_node.processing_energy += calculate_processing_energy(
        sensor_node.processing_operations, avg_processing_time);
    
    /* Calculate radio energy */
    float avg_tx_time = 0.001; // 1ms average transmission time
    float avg_rx_time = 0.001; // 1ms average reception time
    sensor_node.radio_energy += calculate_radio_energy(
        sensor_node.tx_operations, sensor_node.rx_operations,
        avg_tx_time, avg_rx_time);
    
    /* Update total energy consumption */
    if (sensor_node.current_mode == SENSOR_MODE_ACTIVE) {
        sensor_node.total_energy_consumed = sensor_node.baseline_energy + 
                                          sensor_node.sensing_energy +
                                          sensor_node.processing_energy + 
                                          sensor_node.radio_energy;
    } else {
        // Idle mode: baseline + radio energy only
        sensor_node.total_energy_consumed = sensor_node.baseline_energy + 
                                          sensor_node.radio_energy;
    }
    
    sensor_node.last_energy_calc = current_time;
    
    /* Reset operation counters */
    sensor_node.sensing_operations = 0;
    sensor_node.processing_operations = 0;
    sensor_node.tx_operations = 0;
    sensor_node.rx_operations = 0;
}

/* Sensor Operations */
static void switch_to_mode(sensor_mode_t new_mode) {
    if (sensor_node.current_mode != new_mode) {
        update_energy_consumption();
        sensor_node.current_mode = new_mode;
        sensor_node.mode_start_time = clock_time();
        sensor_node.mode_switches++;
        
        LOG_INFO("Switched to %s mode\n", 
                (new_mode == SENSOR_MODE_ACTIVE) ? "ACTIVE" : "IDLE");
    }
}

static void perform_sensing_operation() {
    if (sensor_node.current_mode == SENSOR_MODE_ACTIVE) {
        sensor_node.sensing_operations++;
        sensor_node.processing_operations++;
        sensor_node.last_sensing_time = clock_time();
        
        LOG_INFO("Performed sensing operation at (%u, %u)\n", 
                sensor_node.x_position, sensor_node.y_position);
    }
}

static void initialize_sensor_position() {
    /* Generate random position within target area for initial deployment */
    sensor_node.x_position = random_rand() % TARGET_AREA_WIDTH;
    sensor_node.y_position = random_rand() % TARGET_AREA_HEIGHT;
    sensor_node.is_deployed = 0; // Randomly deployed initially
    
    LOG_INFO("Sensor initialized at random position (%u, %u)\n", 
            sensor_node.x_position, sensor_node.y_position);
}

static void update_sensor_position(uint16_t new_x, uint16_t new_y) {
    sensor_node.x_position = new_x;
    sensor_node.y_position = new_y;
    sensor_node.is_deployed = 1; // Robot deployed
    sensor_node.processing_operations++;
    
    LOG_INFO("Sensor relocated to (%u, %u) by robot\n", new_x, new_y);
}

/* Communication Handlers */
static void udp_rx_callback(struct simple_udp_connection *c,
                           const uip_ipaddr_t *sender_addr,
                           uint16_t sender_port,
                           const uip_ipaddr_t *receiver_addr,
                           uint16_t receiver_port,
                           const uint8_t *data,
                           uint16_t datalen) {
    
    sensor_node.rx_operations++;
    sensor_node.processing_operations++;
    
    /* Handle Mp message from robot */
    if (datalen == sizeof(robot_discovery_msg_t)) {
        robot_discovery_msg_t *robot_msg = (robot_discovery_msg_t *)data;
        
        LOG_INFO("Received Mp from Robot %u - sending Sensor_M reply\n", robot_msg->robot_id);
        
        /* Store robot address for future communication */
        uip_ipaddr_copy(&sensor_node.robot_addr, sender_addr);
        sensor_node.robot_in_range = 1;
        
        /* Send Sensor_M reply as per APP_I specification */
        sensor_reply_msg_t reply;
        reply.sensor_id = sensor_node.sensor_id;
        reply.x_coord = sensor_node.x_position;
        reply.y_coord = sensor_node.y_position;
        reply.sensor_status = (sensor_node.current_mode == SENSOR_MODE_ACTIVE) ? 1 : 0;
        
        simple_udp_sendto(&udp_conn, &reply, sizeof(reply), sender_addr);
        sensor_node.tx_operations++;
        
        LOG_INFO("Sent Sensor_M: (ID=%u, Pos=(%u,%u), Status=%u)\n", 
                reply.sensor_id, reply.x_coord, reply.y_coord, reply.sensor_status);
    }
    
    /* Handle relocation command during dispersion phase */
    if (datalen == sizeof(uint16_t) * 2) {
        uint16_t *coords = (uint16_t*)data;
        uint16_t new_x = coords[0];
        uint16_t new_y = coords[1];
        
        LOG_INFO("Robot relocation: moving to grid center (%u, %u)\n", new_x, new_y);
        update_sensor_position(new_x, new_y);
        
        /* Switch to active mode after robot deployment */
        switch_to_mode(SENSOR_MODE_ACTIVE);
        
        /* Send confirmation */
        sensor_reply_msg_t confirm;
        confirm.sensor_id = sensor_node.sensor_id;
        confirm.x_coord = sensor_node.x_position;
        confirm.y_coord = sensor_node.y_position;
        confirm.sensor_status = 1; // Active after robot deployment
        
        simple_udp_sendto(&udp_conn, &confirm, sizeof(confirm), sender_addr);
        sensor_node.tx_operations++;
        
        LOG_INFO("Confirmed relocation - sensor now active at grid center\n");
    }
}

static void send_status_update() {
    if (sensor_node.robot_in_range && sensor_node.current_mode == SENSOR_MODE_ACTIVE) {
        sensor_reply_msg_t status_msg;
        status_msg.sensor_id = sensor_node.sensor_id;
        status_msg.x_coord = sensor_node.x_position;
        status_msg.y_coord = sensor_node.y_position;
        status_msg.sensor_status = 1; // Active
        
        simple_udp_sendto(&udp_conn, &status_msg, sizeof(status_msg), &sensor_node.robot_addr);
        sensor_node.tx_operations++;
        
        LOG_INFO("Sent status update to robot\n");
    }
}

static void print_energy_report() {
    update_energy_consumption();
    
    clock_time_t elapsed = clock_time() - sensor_node.start_time;
    float elapsed_seconds = (float)elapsed / CLOCK_SECOND;
    
    LOG_INFO("=== SENSOR ENERGY REPORT ===\n");
    LOG_INFO("Sensor ID: %u\n", sensor_node.sensor_id);
    LOG_INFO("Position: (%u, %u)\n", sensor_node.x_position, sensor_node.y_position);
    LOG_INFO("Mode: %s\n", (sensor_node.current_mode == SENSOR_MODE_ACTIVE) ? "ACTIVE" : "IDLE");
    LOG_INFO("Deployed by: %s\n", sensor_node.is_deployed ? "Robot" : "Random");
    LOG_INFO("Elapsed time: %.2f seconds\n", elapsed_seconds);
    LOG_INFO("Baseline energy: %.6f J\n", sensor_node.baseline_energy);
    
    if (sensor_node.current_mode == SENSOR_MODE_ACTIVE) {
        LOG_INFO("Sensing energy: %.6f J\n", sensor_node.sensing_energy);
        LOG_INFO("Processing energy: %.6f J\n", sensor_node.processing_energy);
    }
    
    LOG_INFO("Radio energy: %.6f J\n", sensor_node.radio_energy);
    LOG_INFO("Total energy: %.6f J\n", sensor_node.total_energy_consumed);
    LOG_INFO("Operations - Sensing: %u, Processing: %u, TX: %u, RX: %u\n",
            sensor_node.sensing_operations, sensor_node.processing_operations,
            sensor_node.tx_operations, sensor_node.rx_operations);
    LOG_INFO("============================\n");
}

PROCESS_THREAD(sensor_node_process, ev, data) {
    PROCESS_BEGIN();
    
    /* Initialize sensor node with deterministic ID */
    sensor_node.sensor_id = linkaddr_node_addr.u8[0]; // Use node address as sensor ID
    sensor_node.start_time = clock_time();
    sensor_node.last_energy_calc = sensor_node.start_time;
    sensor_node.mode_start_time = sensor_node.start_time;
    sensor_node.current_mode = SENSOR_MODE_IDLE;
    sensor_node.robot_in_range = 0;
    memset(&sensor_node.robot_addr, 0, sizeof(sensor_node.robot_addr));
    
    /* Initialize position */
    initialize_sensor_position();
    
    /* Initialize UDP connection */
    simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL, UDP_SERVER_PORT, udp_rx_callback);
    
    /* Set timers */
    etimer_set(&sensing_timer, MESSAGE_SEND_INTERVAL);
    etimer_set(&energy_timer, ENERGY_REPORT_INTERVAL);
    etimer_set(&mode_timer, 10 * CLOCK_SECOND);
    
    LOG_INFO("Sensor Node %u initialized at (%u, %u)\n", 
             sensor_node.sensor_id, sensor_node.x_position, sensor_node.y_position);
    
    while(1) {
        PROCESS_WAIT_EVENT();
        
        if (ev == PROCESS_EVENT_TIMER) {
            if (data == &sensing_timer) {
                if (sensor_node.current_mode == SENSOR_MODE_ACTIVE) {
                    perform_sensing_operation();
                    send_status_update();
                }
                etimer_reset(&sensing_timer);
                
            } else if (data == &energy_timer) {
                print_energy_report();
                etimer_reset(&energy_timer);
                
            } else if (data == &mode_timer) {
                /* Randomly switch between active and idle modes if not deployed by robot */
                if (!sensor_node.is_deployed) {
                    if (random_rand() % 100 < 30) { // 30% chance to switch mode
                        sensor_mode_t new_mode = (sensor_node.current_mode == SENSOR_MODE_ACTIVE) ? 
                                               SENSOR_MODE_IDLE : SENSOR_MODE_ACTIVE;
                        switch_to_mode(new_mode);
                    }
                } else {
                    /* Robot-deployed sensors stay active */
                    if (sensor_node.current_mode != SENSOR_MODE_ACTIVE) {
                        switch_to_mode(SENSOR_MODE_ACTIVE);
                    }
                }
                etimer_reset(&mode_timer);
            }
        }
    }
    
    PROCESS_END();
}
