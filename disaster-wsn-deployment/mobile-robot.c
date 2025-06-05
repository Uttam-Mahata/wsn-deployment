/*
 * Mobile Robot for Disaster WSN Deployment
 * Implements local phase: topology discovery and dispersion
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

#define LOG_MODULE "ROBOT"
#define LOG_LEVEL LOG_LEVEL_INFO

/* Message types */
#define MSG_ROBOT_REPORT 1
#define MSG_ROBOT_DEPLOY 2
#define MSG_TOPOLOGY_DISCOVERY 4
#define MSG_SENSOR_RESPONSE 5
#define MSG_SENSOR_DEPLOY 6
#define MSG_SENSOR_COLLECT 7

/* Robot states */
#define ROBOT_IDLE 0
#define ROBOT_DEPLOYED 1
#define ROBOT_TOPOLOGY_DISCOVERY 2
#define ROBOT_DISPERSION 3

/* Grid structure */
typedef struct {
    uint8_t grid_id;
    int16_t center_x;
    int16_t center_y;
    uint8_t status; // 0 = uncovered, 1 = covered
} grid_info_t;

/* Sensor database entry */
typedef struct {
    uint8_t sensor_id;
    int16_t pos_x;
    int16_t pos_y;
    uint8_t status;
    uint8_t battery_level;
} sensor_db_entry_t;

/* Robot information */
typedef struct {
    uint8_t robot_id;
    uint8_t assigned_la_id;
    int16_t la_center_x;
    int16_t la_center_y;
    uint8_t state;
    uint8_t stock_sensors;
    uint8_t no_permissible_moves;
    uint8_t current_grid;
} robot_info_t;

/* Message structures */
typedef struct {
    uint8_t type;
    uint8_t robot_id;
    uint8_t la_id;
    uint8_t covered_grids;
    uint8_t total_grids;
} robot_message_t;

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
static robot_info_t my_robot;
static grid_info_t grid_db[MAX_GRIDS_PER_LA];
static sensor_db_entry_t sensor_db[100]; // Max sensors in perception range
static uint8_t num_grids = 0;
static uint8_t num_sensors = 0;
static uint8_t robot_perception_range = 50;
static uint8_t sensor_perception_range = 20;

PROCESS(mobile_robot_process, "Mobile Robot");
AUTOSTART_PROCESSES(&mobile_robot_process);

/*---------------------------------------------------------------------------*/
/* Initialize robot */
static void initialize_robot(void)
{
    my_robot.robot_id = node_id - 2; // Robot IDs start from 0
    my_robot.state = ROBOT_IDLE;
    my_robot.stock_sensors = INITIAL_STOCK_RS;
    my_robot.assigned_la_id = 0;
    my_robot.current_grid = 0;
    
    LOG_INFO("Robot %d initialized with %d sensors in stock\n", 
             my_robot.robot_id, my_robot.stock_sensors);
}

/*---------------------------------------------------------------------------*/
/* Create grids within assigned LA */
static void create_grids(void)
{
    uint8_t grids_per_side = robot_perception_range / sensor_perception_range;
    num_grids = grids_per_side * grids_per_side;
    
    if(num_grids > MAX_GRIDS_PER_LA) {
        num_grids = MAX_GRIDS_PER_LA;
    }
    
    my_robot.no_permissible_moves = num_grids;
    
    LOG_INFO("Creating %d grids in LA %d\n", num_grids, my_robot.assigned_la_id);
    
    /* Initialize grid database */
    for(uint8_t i = 0; i < num_grids; i++) {
        grid_db[i].grid_id = i + 1;
        
        /* Calculate grid center relative to LA center */
        uint8_t grid_x = i % grids_per_side;
        uint8_t grid_y = i / grids_per_side;
        
        grid_db[i].center_x = my_robot.la_center_x - robot_perception_range/2 + 
                              grid_x * sensor_perception_range + sensor_perception_range/2;
        grid_db[i].center_y = my_robot.la_center_y - robot_perception_range/2 + 
                              grid_y * sensor_perception_range + sensor_perception_range/2;
        grid_db[i].status = 0; // Initially uncovered
        
        LOG_INFO("Grid %d: center (%d, %d)\n", 
                grid_db[i].grid_id, grid_db[i].center_x, grid_db[i].center_y);
    }
}

/*---------------------------------------------------------------------------*/
/* Start topology discovery phase */
static void start_topology_discovery(void)
{
    sensor_message_t discovery_msg;
    uip_ipaddr_t broadcast_addr;
    
    LOG_INFO("Starting topology discovery phase in LA %d\n", my_robot.assigned_la_id);
    my_robot.state = ROBOT_TOPOLOGY_DISCOVERY;
    
    /* Broadcast topology discovery message */
    discovery_msg.type = MSG_TOPOLOGY_DISCOVERY;
    discovery_msg.robot_id = my_robot.robot_id;
    discovery_msg.sensor_id = 0;
    discovery_msg.pos_x = my_robot.la_center_x;
    discovery_msg.pos_y = my_robot.la_center_y;
    discovery_msg.status = 0;
    discovery_msg.battery_level = 0;
    
    uip_create_linklocal_allnodes_mcast(&broadcast_addr);
    
    LOG_INFO("Broadcasting topology discovery from (%d, %d)\n", 
             my_robot.la_center_x, my_robot.la_center_y);
    simple_udp_sendto(&udp_conn, &discovery_msg, sizeof(discovery_msg), &broadcast_addr);
    
    /* Clear sensor database */
    num_sensors = 0;
}

/*---------------------------------------------------------------------------*/
/* Handle sensor response to topology discovery */
static void handle_sensor_response(const sensor_message_t *msg)
{
    /* Check if sensor is within robot's perception range */
    int16_t dx = msg->pos_x - my_robot.la_center_x;
    int16_t dy = msg->pos_y - my_robot.la_center_y;
    uint16_t distance = dx*dx + dy*dy; // Squared distance for efficiency
    uint16_t max_distance = robot_perception_range * robot_perception_range;
    
    if(distance <= max_distance && num_sensors < 100) {
        /* Add sensor to database */
        sensor_db[num_sensors].sensor_id = msg->sensor_id;
        sensor_db[num_sensors].pos_x = msg->pos_x;
        sensor_db[num_sensors].pos_y = msg->pos_y;
        sensor_db[num_sensors].status = msg->status;
        sensor_db[num_sensors].battery_level = msg->battery_level;
        num_sensors++;
        
        LOG_INFO("Added Sensor %d at (%d, %d) to database\n", 
                 msg->sensor_id, msg->pos_x, msg->pos_y);
    }
}

/*---------------------------------------------------------------------------*/
/* Find nearest uncovered grid */
static uint8_t find_nearest_uncovered_grid(uint8_t current_grid)
{
    for(uint8_t i = 0; i < num_grids; i++) {
        if(grid_db[i].status == 0) {
            return i;
        }
    }
    return 255; // No uncovered grid found
}

/*---------------------------------------------------------------------------*/
/* Find sensors in a specific grid */
static uint8_t find_sensors_in_grid(uint8_t grid_idx, uint8_t *sensor_indices, uint8_t max_sensors)
{
    uint8_t count = 0;
    int16_t grid_x = grid_db[grid_idx].center_x;
    int16_t grid_y = grid_db[grid_idx].center_y;
    uint8_t half_range = sensor_perception_range / 2;
    
    for(uint8_t i = 0; i < num_sensors && count < max_sensors; i++) {
        int16_t dx = sensor_db[i].pos_x - grid_x;
        int16_t dy = sensor_db[i].pos_y - grid_y;
        
        if(abs(dx) <= half_range && abs(dy) <= half_range) {
            sensor_indices[count] = i;
            count++;
        }
    }
    
    return count;
}

/*---------------------------------------------------------------------------*/
/* Deploy sensor to grid center */
static void deploy_sensor_to_grid(uint8_t grid_idx, uint8_t sensor_idx)
{
    sensor_message_t deploy_msg;
    uip_ipaddr_t sensor_addr;
    
    deploy_msg.type = MSG_SENSOR_DEPLOY;
    deploy_msg.robot_id = my_robot.robot_id;
    deploy_msg.sensor_id = sensor_db[sensor_idx].sensor_id;
    deploy_msg.pos_x = grid_db[grid_idx].center_x;
    deploy_msg.pos_y = grid_db[grid_idx].center_y;
    deploy_msg.status = 1; // Active
    deploy_msg.battery_level = sensor_db[sensor_idx].battery_level;
    
    /* Create sensor address */
    uip_create_linklocal_allnodes_mcast(&sensor_addr);
    sensor_addr.u8[15] = sensor_db[sensor_idx].sensor_id;
    
    LOG_INFO("Deploying Sensor %d to grid %d center (%d, %d)\n", 
             sensor_db[sensor_idx].sensor_id, grid_idx + 1,
             deploy_msg.pos_x, deploy_msg.pos_y);
    
    simple_udp_sendto(&udp_conn, &deploy_msg, sizeof(deploy_msg), &sensor_addr);
    
    /* Update sensor database */
    sensor_db[sensor_idx].pos_x = deploy_msg.pos_x;
    sensor_db[sensor_idx].pos_y = deploy_msg.pos_y;
    sensor_db[sensor_idx].status = 1;
}

/*---------------------------------------------------------------------------*/
/* Execute dispersion phase for a specific grid */
static void execute_dispersion_for_grid(uint8_t grid_idx)
{
    uint8_t sensor_indices[10];
    uint8_t sensors_in_grid = find_sensors_in_grid(grid_idx, sensor_indices, 10);
    
    LOG_INFO("Processing grid %d: found %d sensors, stock: %d\n", 
             grid_idx + 1, sensors_in_grid, my_robot.stock_sensors);
    
    if(my_robot.stock_sensors > 0 && sensors_in_grid > 0) {
        /* Case 1: Robot has sensors and grid has sensors */
        deploy_sensor_to_grid(grid_idx, sensor_indices[0]);
        my_robot.stock_sensors--;
        
        /* Collect extra sensors */
        for(uint8_t i = 1; i < sensors_in_grid && my_robot.stock_sensors < MAX_SENSORS_PER_ROBOT; i++) {
            LOG_INFO("Collecting extra Sensor %d\n", sensor_db[sensor_indices[i]].sensor_id);
            my_robot.stock_sensors++;
        }
        
        grid_db[grid_idx].status = 1; // Covered
        
    } else if(my_robot.stock_sensors > 0 && sensors_in_grid == 0) {
        /* Case 2: Robot has sensors but grid has no sensors */
        /* Would need to deploy from stock - simulate this */
        LOG_INFO("Deploying sensor from stock to grid %d\n", grid_idx + 1);
        my_robot.stock_sensors--;
        grid_db[grid_idx].status = 1; // Covered
        
    } else if(my_robot.stock_sensors == 0 && sensors_in_grid > 0) {
        /* Case 3: Robot has no sensors but grid has sensors */
        deploy_sensor_to_grid(grid_idx, sensor_indices[0]);
        
        /* Collect other sensors */
        for(uint8_t i = 1; i < sensors_in_grid && my_robot.stock_sensors < MAX_SENSORS_PER_ROBOT; i++) {
            LOG_INFO("Collecting Sensor %d\n", sensor_db[sensor_indices[i]].sensor_id);
            my_robot.stock_sensors++;
        }
        
        grid_db[grid_idx].status = 1; // Covered
        
    } else {
        /* Case 4: No sensors available - grid remains uncovered */
        LOG_INFO("Grid %d remains uncovered - no sensors available\n", grid_idx + 1);
    }
    
    my_robot.no_permissible_moves--;
}

/*---------------------------------------------------------------------------*/
/* Start dispersion phase */
static void start_dispersion_phase(void)
{
    LOG_INFO("Starting dispersion phase\n");
    my_robot.state = ROBOT_DISPERSION;
    my_robot.current_grid = 0;
}

/*---------------------------------------------------------------------------*/
/* Complete local phase and report to base station */
static void complete_local_phase(void)
{
    uint8_t covered_grids = 0;
    robot_message_t report;
    uip_ipaddr_t bs_addr;
    
    /* Count covered grids */
    for(uint8_t i = 0; i < num_grids; i++) {
        if(grid_db[i].status == 1) {
            covered_grids++;
        }
    }
    
    LOG_INFO("Local phase complete: %d/%d grids covered\n", covered_grids, num_grids);
    
    /* Send report to base station */
    report.type = MSG_ROBOT_REPORT;
    report.robot_id = my_robot.robot_id;
    report.la_id = my_robot.assigned_la_id;
    report.covered_grids = covered_grids;
    report.total_grids = num_grids;
    
    uip_create_linklocal_allnodes_mcast(&bs_addr);
    bs_addr.u8[15] = 1; // Base station ID
    
    simple_udp_sendto(&udp_conn, &report, sizeof(report), &bs_addr);
    
    /* Reset for next deployment */
    my_robot.state = ROBOT_IDLE;
    my_robot.no_permissible_moves = num_grids;
}

/*---------------------------------------------------------------------------*/
/* Handle deployment command from base station */
static void handle_deployment_command(const robot_message_t *msg)
{
    LOG_INFO("Received deployment to LA %d\n", msg->la_id);
    
    my_robot.assigned_la_id = msg->la_id;
    my_robot.state = ROBOT_DEPLOYED;
    
    /* Calculate LA center (simplified - would be provided by BS) */
    my_robot.la_center_x = ((msg->la_id - 1) % 4) * robot_perception_range + robot_perception_range/2;
    my_robot.la_center_y = ((msg->la_id - 1) / 4) * robot_perception_range + robot_perception_range/2;
    
    /* Create grids within the LA */
    create_grids();
    
    /* Start topology discovery */
    start_topology_discovery();
}

/*---------------------------------------------------------------------------*/
/* UDP callback */
static void udp_rx_callback(struct simple_udp_connection *c,
                           const uip_ipaddr_t *sender_addr,
                           uint16_t sender_port,
                           const uip_ipaddr_t *receiver_addr,
                           uint16_t receiver_port,
                           const uint8_t *data,
                           uint16_t datalen)
{
    if(datalen == sizeof(robot_message_t)) {
        robot_message_t *msg = (robot_message_t *)data;
        if(msg->type == MSG_ROBOT_DEPLOY && msg->robot_id == my_robot.robot_id) {
            handle_deployment_command(msg);
        }
    } else if(datalen == sizeof(sensor_message_t)) {
        sensor_message_t *msg = (sensor_message_t *)data;
        if(msg->type == MSG_SENSOR_RESPONSE && msg->robot_id == my_robot.robot_id) {
            handle_sensor_response(msg);
        }
    }
}

/*---------------------------------------------------------------------------*/
/* Main process */
PROCESS_THREAD(mobile_robot_process, ev, data)
{
    static struct etimer topology_timer;
    static struct etimer dispersion_timer;
    
    PROCESS_BEGIN();
    
    LOG_INFO("Mobile Robot starting...\n");
    
    /* Initialize UDP connection */
    simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL, UDP_SERVER_PORT, udp_rx_callback);
    
    /* Initialize robot */
    initialize_robot();
    
    /* Main state machine */
    while(1) {
        PROCESS_WAIT_EVENT();
        
        if(my_robot.state == ROBOT_TOPOLOGY_DISCOVERY) {
            if(!etimer_expired(&topology_timer)) {
                etimer_set(&topology_timer, 5 * CLOCK_SECOND);
            } else {
                /* Topology discovery complete, start dispersion */
                LOG_INFO("Topology discovery complete, found %d sensors\n", num_sensors);
                start_dispersion_phase();
                etimer_set(&dispersion_timer, 2 * CLOCK_SECOND);
            }
        }
        
        if(my_robot.state == ROBOT_DISPERSION && etimer_expired(&dispersion_timer)) {
            if(my_robot.no_permissible_moves > 0) {
                /* Find next grid to process */
                uint8_t grid_idx = find_nearest_uncovered_grid(my_robot.current_grid);
                if(grid_idx != 255) {
                    execute_dispersion_for_grid(grid_idx);
                    my_robot.current_grid = grid_idx + 1;
                } else {
                    my_robot.no_permissible_moves = 0; // No more uncovered grids
                }
                
                if(my_robot.no_permissible_moves > 0) {
                    etimer_reset(&dispersion_timer);
                }
            }
            
            if(my_robot.no_permissible_moves == 0) {
                /* Dispersion complete */
                complete_local_phase();
            }
        }
    }
    
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/ 