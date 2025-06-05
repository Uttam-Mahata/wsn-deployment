/*
 * Base Station for Disaster WSN Deployment
 * Implements APP_I: Deterministic grid-based deployment
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

#define LOG_MODULE "BS"
#define LOG_LEVEL LOG_LEVEL_INFO

/* Message types */
#define MSG_ROBOT_REPORT 1
#define MSG_ROBOT_DEPLOY 2
#define MSG_COVERAGE_QUERY 3

/* Database structures */
typedef struct {
    uint8_t la_id;
    int16_t center_x;
    int16_t center_y;
    uint8_t no_grid;
} la_db_entry_t;

typedef struct {
    uint8_t robot_id;
    uint8_t assigned_la_id;
} robot_db_entry_t;

/* Global variables */
static struct simple_udp_connection udp_conn;
static la_db_entry_t la_db[MAX_LOCATION_AREAS];
static robot_db_entry_t robot_db[2]; // Two robots as specified
static uint8_t num_las = 0;
static uint8_t active_robots = 0;

/* Target area parameters */
static int16_t target_area_width = 200;
static int16_t target_area_height = 200;
static uint8_t robot_perception_range = 50;
static uint8_t sensor_perception_range = 20;

PROCESS(base_station_process, "Base Station");
AUTOSTART_PROCESSES(&base_station_process);

/*---------------------------------------------------------------------------*/
/* Message structure for robot communication */
typedef struct {
    uint8_t type;
    uint8_t robot_id;
    uint8_t la_id;
    uint8_t covered_grids;
    uint8_t total_grids;
} robot_message_t;

/*---------------------------------------------------------------------------*/
/* Calculate number of location areas based on target area and robot range */
static void calculate_location_areas(void)
{
    uint8_t las_x = target_area_width / robot_perception_range;
    uint8_t las_y = target_area_height / robot_perception_range;
    num_las = las_x * las_y;
    
    if(num_las > MAX_LOCATION_AREAS) {
        num_las = MAX_LOCATION_AREAS;
        LOG_WARN("Limited to %d location areas\n", MAX_LOCATION_AREAS);
    }
    
    LOG_INFO("Calculated %d location areas (%dx%d)\n", num_las, las_x, las_y);
    
    /* Initialize LA database */
    for(uint8_t i = 0; i < num_las; i++) {
        la_db[i].la_id = i + 1;
        la_db[i].center_x = (i % las_x) * robot_perception_range + robot_perception_range/2;
        la_db[i].center_y = (i / las_x) * robot_perception_range + robot_perception_range/2;
        la_db[i].no_grid = 0; // Initially uncovered
        
        LOG_INFO("LA %d: center (%d, %d)\n", 
                la_db[i].la_id, la_db[i].center_x, la_db[i].center_y);
    }
}

/*---------------------------------------------------------------------------*/
/* Find an uncovered location area */
static uint8_t find_uncovered_la(void)
{
    for(uint8_t i = 0; i < num_las; i++) {
        if(la_db[i].no_grid == 0) {
            return la_db[i].la_id;
        }
    }
    return 0; // No uncovered LA found
}

/*---------------------------------------------------------------------------*/
/* Deploy robot to a location area */
static void deploy_robot(uint8_t robot_id, uint8_t la_id)
{
    robot_message_t msg;
    uip_ipaddr_t robot_addr;
    
    /* Store deployment info in robot DB */
    robot_db[robot_id].robot_id = robot_id;
    robot_db[robot_id].assigned_la_id = la_id;
    
    /* Create deployment message */
    msg.type = MSG_ROBOT_DEPLOY;
    msg.robot_id = robot_id;
    msg.la_id = la_id;
    msg.covered_grids = 0;
    msg.total_grids = (robot_perception_range / sensor_perception_range) * 
                      (robot_perception_range / sensor_perception_range);
    
    /* Find robot address based on ID */
    uip_create_linklocal_allnodes_mcast(&robot_addr);
    robot_addr.u8[15] = robot_id + 2; // Robot IDs start from 2
    
    LOG_INFO("Deploying Robot %d to LA %d\n", robot_id, la_id);
    simple_udp_sendto(&udp_conn, &msg, sizeof(msg), &robot_addr);
}

/*---------------------------------------------------------------------------*/
/* Calculate percentage area coverage */
static uint8_t calculate_coverage_percentage(void)
{
    uint16_t total_grids = 0;
    uint16_t covered_grids = 0;
    
    for(uint8_t i = 0; i < num_las; i++) {
        uint8_t grids_per_la = (robot_perception_range / sensor_perception_range) * 
                               (robot_perception_range / sensor_perception_range);
        total_grids += grids_per_la;
        covered_grids += la_db[i].no_grid;
    }
    
    if(total_grids == 0) return 0;
    return (covered_grids * 100) / total_grids;
}

/*---------------------------------------------------------------------------*/
/* Handle robot reports */
static void handle_robot_report(const robot_message_t *msg)
{
    LOG_INFO("Received report from Robot %d: LA %d covered %d grids\n", 
             msg->robot_id, msg->la_id, msg->covered_grids);
    
    /* Update LA database */
    for(uint8_t i = 0; i < num_las; i++) {
        if(la_db[i].la_id == msg->la_id) {
            la_db[i].no_grid = msg->covered_grids;
            break;
        }
    }
    
    /* Try to deploy robot to next uncovered LA */
    uint8_t next_la = find_uncovered_la();
    if(next_la > 0) {
        deploy_robot(msg->robot_id, next_la);
    } else {
        /* All LAs covered, calculate final coverage */
        uint8_t coverage = calculate_coverage_percentage();
        LOG_INFO("Deployment complete! Coverage: %d%%\n", coverage);
        
        /* Log final statistics */
        LOG_INFO("=== Final Coverage Report ===\n");
        for(uint8_t i = 0; i < num_las; i++) {
            LOG_INFO("LA %d: %d grids covered\n", la_db[i].la_id, la_db[i].no_grid);
        }
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
    if(datalen == sizeof(robot_message_t)) {
        robot_message_t *msg = (robot_message_t *)data;
        
        switch(msg->type) {
            case MSG_ROBOT_REPORT:
                handle_robot_report(msg);
                break;
            default:
                LOG_WARN("Unknown message type: %d\n", msg->type);
                break;
        }
    }
}

/*---------------------------------------------------------------------------*/
/* Main process */
PROCESS_THREAD(base_station_process, ev, data)
{
    static struct etimer timer;
    
    PROCESS_BEGIN();
    
    LOG_INFO("Base Station starting...\n");
    
    /* Initialize as DAG root */
    NETSTACK_ROUTING.root_start();
    
    /* Initialize UDP connection */
    simple_udp_register(&udp_conn, UDP_SERVER_PORT, NULL, UDP_CLIENT_PORT, udp_rx_callback);
    
    /* Calculate and initialize location areas */
    calculate_location_areas();
    
    /* Wait for network to stabilize */
    etimer_set(&timer, 10 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    
    LOG_INFO("Starting global phase - deploying robots\n");
    
    /* Deploy robots to initial LAs */
    if(num_las >= 2) {
        deploy_robot(0, 1); // Robot 0 to LA 1
        deploy_robot(1, num_las); // Robot 1 to last LA
        active_robots = 2;
    } else if(num_las >= 1) {
        deploy_robot(0, 1); // Robot 0 to LA 1
        active_robots = 1;
    }
    
    /* Keep the process running */
    while(1) {
        PROCESS_WAIT_EVENT();
    }
    
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/ 