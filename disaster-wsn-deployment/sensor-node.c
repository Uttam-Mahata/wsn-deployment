/*
 * Sensor Node for Disaster WSN Deployment
 * Responds to robot topology discovery and deployment commands
 * Author: Generated from main.tex specification
 */

#include "contiki.h"
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "sys/log.h"
#include "lib/random.h"
#include <string.h>
#include <stdio.h>

#define LOG_MODULE "SENSOR"
#define LOG_LEVEL LOG_LEVEL_INFO

/* Message types */
#define MSG_TOPOLOGY_DISCOVERY 4
#define MSG_SENSOR_RESPONSE 5
#define MSG_SENSOR_DEPLOY 6
#define MSG_SENSOR_COLLECT 7

/* Sensor states */
#define SENSOR_IDLE 0
#define SENSOR_ACTIVE 1

/* Sensor node structure */
typedef struct {
    uint8_t sensor_id;
    int16_t pos_x;
    int16_t pos_y;
    uint8_t status; // SENSOR_IDLE or SENSOR_ACTIVE
    uint8_t battery_level;
    uint8_t sensing_range;
} sensor_info_t;

typedef struct {
    uint8_t type;
    uint8_t robot_id;
    uint8_t sensor_id;
    int16_t pos_x;
    int16_t pos_y;
    uint8_t status;
    uint8_t battery_level;
} sensor_message_t;

/* Global variables */
static struct simple_udp_connection udp_conn;
static sensor_info_t my_sensor;
static uint8_t energy_consumption_active = 10;
static uint8_t energy_consumption_idle = 3;

PROCESS(sensor_node_process, "Sensor Node");
AUTOSTART_PROCESSES(&sensor_node_process);

/*---------------------------------------------------------------------------*/
/* Initialize sensor with random position and parameters */
static void initialize_sensor(void)
{
    /* Set sensor ID based on node ID */
    my_sensor.sensor_id = node_id;
    
    /* Random initial position in target area */
    my_sensor.pos_x = random_rand() % 200;
    my_sensor.pos_y = random_rand() % 200;
    
    /* Initial status is idle after random deployment */
    my_sensor.status = SENSOR_IDLE;
    
    /* Initialize battery level (random between 70-100%) */
    my_sensor.battery_level = 70 + (random_rand() % 30);
    
    /* Set sensing range */
    my_sensor.sensing_range = 20;
    
    LOG_INFO("Sensor %d initialized at (%d, %d), battery: %d%%\n", 
             my_sensor.sensor_id, my_sensor.pos_x, my_sensor.pos_y, my_sensor.battery_level);
}

/*---------------------------------------------------------------------------*/
/* Update sensor position (for robot deployment) */
static void update_sensor_position(int16_t new_x, int16_t new_y, uint8_t new_status)
{
    my_sensor.pos_x = new_x;
    my_sensor.pos_y = new_y;
    my_sensor.status = new_status;
    
    LOG_INFO("Sensor %d moved to (%d, %d), status: %s\n", 
             my_sensor.sensor_id, my_sensor.pos_x, my_sensor.pos_y,
             (my_sensor.status == SENSOR_ACTIVE) ? "ACTIVE" : "IDLE");
}

/*---------------------------------------------------------------------------*/
/* Respond to topology discovery from robot */
static void respond_to_topology_discovery(const uip_ipaddr_t *robot_addr, uint8_t robot_id)
{
    sensor_message_t response;
    
    response.type = MSG_SENSOR_RESPONSE;
    response.robot_id = robot_id;
    response.sensor_id = my_sensor.sensor_id;
    response.pos_x = my_sensor.pos_x;
    response.pos_y = my_sensor.pos_y;
    response.status = my_sensor.status;
    response.battery_level = my_sensor.battery_level;
    
    LOG_INFO("Responding to Robot %d topology discovery\n", robot_id);
    simple_udp_sendto(&udp_conn, &response, sizeof(response), robot_addr);
}

/*---------------------------------------------------------------------------*/
/* Handle deployment command from robot */
static void handle_deployment(const sensor_message_t *msg)
{
    LOG_INFO("Deployment command from Robot %d: move to (%d, %d)\n", 
             msg->robot_id, msg->pos_x, msg->pos_y);
    
    /* Update position and activate sensor */
    update_sensor_position(msg->pos_x, msg->pos_y, SENSOR_ACTIVE);
    
    /* Send acknowledgment */
    sensor_message_t ack;
    ack.type = MSG_SENSOR_RESPONSE;
    ack.robot_id = msg->robot_id;
    ack.sensor_id = my_sensor.sensor_id;
    ack.pos_x = my_sensor.pos_x;
    ack.pos_y = my_sensor.pos_y;
    ack.status = my_sensor.status;
    ack.battery_level = my_sensor.battery_level;
    
    uip_ipaddr_t robot_addr;
    uip_create_linklocal_allnodes_mcast(&robot_addr);
    robot_addr.u8[15] = msg->robot_id + 2;
    
    simple_udp_sendto(&udp_conn, &ack, sizeof(ack), &robot_addr);
}

/*---------------------------------------------------------------------------*/
/* Simulate energy consumption */
static void consume_energy(void)
{
    uint8_t consumption = (my_sensor.status == SENSOR_ACTIVE) ? 
                         energy_consumption_active : energy_consumption_idle;
    
    if(my_sensor.battery_level > consumption) {
        my_sensor.battery_level -= consumption;
    } else {
        my_sensor.battery_level = 0;
        my_sensor.status = SENSOR_IDLE; // Deactivate if battery depleted
        LOG_WARN("Sensor %d battery depleted!\n", my_sensor.sensor_id);
    }
}

/*---------------------------------------------------------------------------*/
/* Simulate sensing activity (if active) */
static void perform_sensing(void)
{
    if(my_sensor.status == SENSOR_ACTIVE && my_sensor.battery_level > 0) {
        /* Simulate data sensing and processing */
        LOG_INFO("Sensor %d sensing... (battery: %d%%)\n", 
                 my_sensor.sensor_id, my_sensor.battery_level);
        
        /* Additional energy consumption for sensing */
        consume_energy();
    }
}

/*---------------------------------------------------------------------------*/
/* UDP callback for receiving messages */
static void udp_rx_callback(struct simple_udp_connection *c,
                           const uip_ipaddr_t *sender_addr,
                           uint16_t sender_port,
                           const uip_ipaddr_t *receiver_addr,
                           uint16_t receiver_port,
                           const uint8_t *data,
                           uint16_t datalen)
{
    if(datalen == sizeof(sensor_message_t)) {
        sensor_message_t *msg = (sensor_message_t *)data;
        
        switch(msg->type) {
            case MSG_TOPOLOGY_DISCOVERY:
                respond_to_topology_discovery(sender_addr, msg->robot_id);
                break;
                
            case MSG_SENSOR_DEPLOY:
                handle_deployment(msg);
                break;
                
            case MSG_SENSOR_COLLECT:
                /* Robot is collecting this sensor - set to idle and move */
                LOG_INFO("Being collected by Robot %d\n", msg->robot_id);
                my_sensor.status = SENSOR_IDLE;
                break;
                
            default:
                LOG_WARN("Unknown message type: %d\n", msg->type);
                break;
        }
    }
}

/*---------------------------------------------------------------------------*/
/* Main process */
PROCESS_THREAD(sensor_node_process, ev, data)
{
    static struct etimer periodic_timer;
    static struct etimer sensing_timer;
    
    PROCESS_BEGIN();
    
    LOG_INFO("Sensor Node starting...\n");
    
    /* Initialize UDP connection */
    simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL, UDP_SERVER_PORT, udp_rx_callback);
    
    /* Initialize sensor parameters */
    initialize_sensor();
    
    /* Set periodic timers */
    etimer_set(&periodic_timer, 30 * CLOCK_SECOND); // General updates
    etimer_set(&sensing_timer, 5 * CLOCK_SECOND);   // Sensing activity
    
    while(1) {
        PROCESS_WAIT_EVENT();
        
        if(etimer_expired(&sensing_timer)) {
            perform_sensing();
            etimer_reset(&sensing_timer);
        }
        
        if(etimer_expired(&periodic_timer)) {
            /* Periodic energy consumption and status update */
            consume_energy();
            
            /* Log current status */
            LOG_INFO("Status: %s, Position: (%d, %d), Battery: %d%%\n",
                     (my_sensor.status == SENSOR_ACTIVE) ? "ACTIVE" : "IDLE",
                     my_sensor.pos_x, my_sensor.pos_y, my_sensor.battery_level);
            
            etimer_reset(&periodic_timer);
        }
    }
    
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/ 