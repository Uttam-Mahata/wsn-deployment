/*
 * WSN Deployment Base Station Implementation
 * 
 * The base station coordinates the global phase of sensor deployment
 * by managing location areas and robot assignments.
 */

#include "contiki.h"
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "sys/log.h"
#include "sys/node-id.h"
#include "sys/energest.h"
#include "wsn-deployment.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define LOG_MODULE "BS"
#define LOG_LEVEL LOG_LEVEL_INFO

/* Base Station Process */
PROCESS(base_station_process, "Base Station Process");
AUTOSTART_PROCESSES(&base_station_process);

/* Network communication */
static struct simple_udp_connection udp_conn;

/* Database structures for the base station */
static la_db_entry_t la_db[WSN_DEPLOYMENT_CONF_MAX_LOCATION_AREAS];
static robot_db_entry_t robot_db[WSN_DEPLOYMENT_CONF_MAX_ROBOTS];
static uint8_t num_las = 0;
static uint8_t num_robots = 0;

/* Statistics */
static uint8_t total_covered_grids = 0;
static uint8_t total_grids = 0;

/* Energy tracking using Energest */
static uint64_t last_cpu, last_lpm, last_transmit, last_listen;
static uint64_t total_cpu_energy = 0;
static uint64_t total_lpm_energy = 0;
static uint64_t total_tx_energy = 0;
static uint64_t total_rx_energy = 0;

/* Energy parameters (in microjoules per tick) */
#define ENERGEST_CPU_POWER    1800  /* 1.8 mW for CPU active */
#define ENERGEST_LPM_POWER    54    /* 0.054 mW for low power mode */
#define ENERGEST_TX_POWER     17400 /* 17.4 mW for transmission */
#define ENERGEST_RX_POWER     18800 /* 18.8 mW for reception */

/* Network energy tracking - reported by nodes */
static double robot_reported_energy[WSN_DEPLOYMENT_CONF_MAX_ROBOTS] = {0};
static double sensor_reported_energy[WSN_DEPLOYMENT_CONF_MAX_SENSORS] = {0};

/* Track assignment attempts */
static bool assignments_sent = false;

/* Forward declarations */
static void send_la_assignment(uint8_t robot_id, uint8_t la_id);
static bool assign_la_to_robot(uint8_t robot_id);
static void update_la_status(uint8_t robot_id, uint8_t covered_grids);
static void update_base_station_energy(void);
static double calculate_total_network_energy(void);

/*---------------------------------------------------------------------------*/
/* Initialize Energest tracking */
static void init_energest_tracking(void)
{
    energest_init();
    
    /* Get initial values - Energest API doesn't take parameters */
    last_cpu = energest_get_total_time();
    last_lpm = energest_get_total_time();
    last_transmit = energest_get_total_time();
    last_listen = energest_get_total_time();
    
    LOG_INFO("Energest tracking initialized\n");
}

/*---------------------------------------------------------------------------*/
/* Update energy consumption using Energest */
static void update_energest_consumption(void)
{
    uint64_t curr_cpu, curr_lpm, curr_transmit, curr_listen;
    uint64_t diff_cpu, diff_lpm, diff_transmit, diff_listen;
    
    /* Get current values */
    curr_cpu = energest_get_total_time();
    curr_lpm = energest_get_total_time();
    curr_transmit = energest_get_total_time();
    curr_listen = energest_get_total_time();
    
    /* Calculate differences */
    diff_cpu = curr_cpu - last_cpu;
    diff_lpm = curr_lpm - last_lpm;
    diff_transmit = curr_transmit - last_transmit;
    diff_listen = curr_listen - last_listen;
    
    /* Calculate energy consumption (in microjoules) */
    total_cpu_energy += (diff_cpu * ENERGEST_CPU_POWER) / ENERGEST_SECOND;
    total_lpm_energy += (diff_lpm * ENERGEST_LPM_POWER) / ENERGEST_SECOND;
    total_tx_energy += (diff_transmit * ENERGEST_TX_POWER) / ENERGEST_SECOND;
    total_rx_energy += (diff_listen * ENERGEST_RX_POWER) / ENERGEST_SECOND;
    
    /* Update last values */
    last_cpu = curr_cpu;
    last_lpm = curr_lpm;
    last_transmit = curr_transmit;
    last_listen = curr_listen;
}

/*---------------------------------------------------------------------------*/
/* Get total base station energy in Joules */
static double get_base_station_energy_joules(void)
{
    update_energest_consumption();
    uint64_t total_microjoules = total_cpu_energy + total_lpm_energy + 
                                total_tx_energy + total_rx_energy;
    return (double)total_microjoules / 1000000.0; /* Convert to Joules */
}

/*---------------------------------------------------------------------------*/
/* Update base station energy consumption using Energest */
static void update_base_station_energy(void)
{
    update_energest_consumption();
}

/*---------------------------------------------------------------------------*/
/* Calculate total network energy using Energest data */
double calculate_total_network_energy(void)
{
    double robot_energy = 0.0;
    double sensor_energy = 0.0;
    
    /* Sum up reported robot energies */
    for (uint8_t i = 0; i < WSN_DEPLOYMENT_CONF_MAX_ROBOTS; i++) {
        robot_energy += robot_reported_energy[i];
    }
    
    /* Sum up reported sensor energies */
    for (uint8_t i = 0; i < WSN_DEPLOYMENT_CONF_MAX_SENSORS; i++) {
        sensor_energy += sensor_reported_energy[i];
    }
    
    /* Return total network energy */
    return sensor_energy + robot_energy + get_base_station_energy_joules();
}

/*---------------------------------------------------------------------------*/
/* Calculate area coverage percentage */
static double calculate_area_coverage(void)
{
    if (total_grids == 0) return 0.0;
    return (double)(total_covered_grids * 100) / total_grids;
}

/*---------------------------------------------------------------------------*/
/* UDP callback function */
static void udp_rx_callback(struct simple_udp_connection *c,
                           const uip_ipaddr_t *sender_addr,
                           uint16_t sender_port,
                           const uip_ipaddr_t *receiver_addr,
                           uint16_t receiver_port,
                           const uint8_t *data,
                           uint16_t datalen)
{
    LOG_INFO("BS received UDP message, length: %d\n", datalen);
    
    if (datalen == sizeof(robot_report_msg_t)) {
        robot_report_msg_t *msg = (robot_report_msg_t *)data;
        
        if (msg->msg_type == WSN_MSG_TYPE_ROBOT_REPORT) {
            LOG_INFO("Received report from Robot_%d: %d grids covered\n", 
                     msg->robot_id, msg->covered_grids);
            
            /* Update LA status based on robot report */
            update_la_status(msg->robot_id, msg->covered_grids);
            
            /* Energy tracking is handled automatically by Energest */
            
            /* Try to assign new LA to robot */
            if (assign_la_to_robot(msg->robot_id)) {
                LOG_INFO("Reassigned Robot_%d to new LA\n", msg->robot_id);
            } else {
                LOG_INFO("No more LAs to assign to Robot_%d\n", msg->robot_id);
            }
        }
    } else {
        LOG_INFO("Unknown message type or size: %d bytes\n", datalen);
    }
}

/*---------------------------------------------------------------------------*/
/* Print deployment statistics with Energest data */
static void print_statistics(void)
{
    double coverage = calculate_area_coverage();
    double network_energy = calculate_total_network_energy();
    double base_energy = get_base_station_energy_joules();
    
    LOG_INFO("=== DEPLOYMENT STATISTICS ===\n");
    LOG_INFO("Total Location Areas: %d\n", num_las);
    LOG_INFO("Total Grids: %d\n", total_grids);
    LOG_INFO("Covered Grids: %d\n", total_covered_grids);
    LOG_INFO("Area Coverage: %.2f%%\n", coverage);
    
    /* Print LA status */
    LOG_INFO("Location Area Status:\n");
    for (uint8_t i = 0; i < num_las; i++) {
        LOG_INFO("  LA_%d: %d/%d grids covered\n", 
                 la_db[i].la_id, la_db[i].no_grid, NO_G);
    }
    
    /* Print energy statistics using Energest */
    LOG_INFO("=== ENERGY STATISTICS (Energest) ===\n");
    LOG_INFO("Base Station Energy: %.6f J\n", base_energy);
    LOG_INFO("  - CPU: %.6f J\n", (double)total_cpu_energy / 1000000.0);
    LOG_INFO("  - LPM: %.6f J\n", (double)total_lpm_energy / 1000000.0);
    LOG_INFO("  - TX: %.6f J\n", (double)total_tx_energy / 1000000.0);
    LOG_INFO("  - RX: %.6f J\n", (double)total_rx_energy / 1000000.0);
    LOG_INFO("Total Network Energy: %.6f J\n", network_energy);
    
    /* Print raw Energest values for debugging */
    LOG_INFO("Energest raw values:\n");
    LOG_INFO("  Total ticks: %lu\n", (unsigned long)energest_get_total_time());
}

/*---------------------------------------------------------------------------*/
/* Initialize Location Area Database */
static void init_la_db(void)
{
    uint8_t la_count = 0;
    int16_t x, y;
    
    /* Calculate number of location areas based on target area and robot perception range */
    num_las = NO_LA;
    if (num_las > WSN_DEPLOYMENT_CONF_MAX_LOCATION_AREAS) {
        num_las = WSN_DEPLOYMENT_CONF_MAX_LOCATION_AREAS;
    }
    
    LOG_INFO("Initializing %d location areas\n", num_las);
    
    /* Create grid of location areas */
    for (y = ROBOT_PERCEPTION_RANGE/2; y < TARGET_AREA_SIZE && la_count < num_las; y += ROBOT_PERCEPTION_RANGE) {
        for (x = ROBOT_PERCEPTION_RANGE/2; x < TARGET_AREA_SIZE && la_count < num_las; x += ROBOT_PERCEPTION_RANGE) {
            la_db[la_count].la_id = la_count + 1;
            la_db[la_count].center_x = x;
            la_db[la_count].center_y = y;
            la_db[la_count].no_grid = 0; /* Initially no grids covered */
            
            LOG_INFO("LA_%d: center(%d, %d)\n", la_db[la_count].la_id, x, y);
            la_count++;
        }
    }
    
    num_las = la_count;
    total_grids = num_las * NO_G;
    
    LOG_INFO("Total location areas: %d, Total grids: %d\n", num_las, total_grids);
}

/*---------------------------------------------------------------------------*/
/* Find an unassigned location area */
static int8_t find_unassigned_la(void)
{
    for (uint8_t i = 0; i < num_las; i++) {
        if (la_db[i].no_grid == 0) {
            /* Check if this LA is already assigned to a robot */
            bool already_assigned = false;
            for (uint8_t j = 0; j < num_robots; j++) {
                if (robot_db[j].assigned_la_id == la_db[i].la_id) {
                    already_assigned = true;
                    break;
                }
            }
            if (!already_assigned) {
                return i;
            }
        }
    }
    return -1; /* No unassigned LA found */
}

/*---------------------------------------------------------------------------*/
/* Assign location area to robot */
static bool assign_la_to_robot(uint8_t robot_id)
{
    int8_t la_index = find_unassigned_la();
    
    if (la_index == -1) {
        LOG_INFO("No unassigned location area found\n");
        return false;
    }
    
    /* Update robot database */
    robot_db[num_robots].robot_id = robot_id;
    robot_db[num_robots].assigned_la_id = la_db[la_index].la_id;
    num_robots++;
    
    LOG_INFO("Assigned LA_%d to Robot_%d\n", la_db[la_index].la_id, robot_id);
    
    /* Send LA assignment to robot */
    send_la_assignment(robot_id, la_db[la_index].la_id);
    
    return true;
}

/*---------------------------------------------------------------------------*/
/* Update location area status after robot reports */
static void update_la_status(uint8_t robot_id, uint8_t covered_grids)
{
    /* Find robot in database */
    for (uint8_t i = 0; i < num_robots; i++) {
        if (robot_db[i].robot_id == robot_id) {
            uint8_t la_id = robot_db[i].assigned_la_id;
            
            /* Find LA in database and update */
            for (uint8_t j = 0; j < num_las; j++) {
                if (la_db[j].la_id == la_id) {
                    la_db[j].no_grid = covered_grids;
                    
                    LOG_INFO("Updated LA_%d: %d grids covered by Robot_%d\n", 
                             la_id, covered_grids, robot_id);
                    
                    /* Recalculate total covered grids */
                    total_covered_grids = 0;
                    for (uint8_t k = 0; k < num_las; k++) {
                        total_covered_grids += la_db[k].no_grid;
                    }
                    
                    break;
                }
            }
            break;
        }
    }
}

/*---------------------------------------------------------------------------*/
/* Send LA assignment with direct addressing */
static void send_la_assignment(uint8_t robot_id, uint8_t la_id)
{
    /* Find LA details */
    for (uint8_t i = 0; i < num_las; i++) {
        if (la_db[i].la_id == la_id) {
            la_assignment_msg_t assignment;
            assignment.msg_type = WSN_MSG_TYPE_LA_ASSIGNMENT;
            assignment.robot_id = robot_id;
            assignment.la_id = la_id;
            assignment.center_x = la_db[i].center_x;
            assignment.center_y = la_db[i].center_y;
            
            LOG_INFO("Sending LA assignment to Robot_%d\n", robot_id);
            LOG_INFO("Assignment: LA_%d at (%d, %d)\n", la_id, assignment.center_x, assignment.center_y);
            
            /* Use both multicast and direct addressing for better delivery */
            uip_ipaddr_t dest_ipaddr;
            
            /* Method 1: Link-local multicast */
            uip_create_linklocal_allnodes_mcast(&dest_ipaddr);
            
            /* Send message multiple times for reliability */
            for (int attempts = 0; attempts < 5; attempts++) {
                simple_udp_sendto(&udp_conn, &assignment, sizeof(assignment), &dest_ipaddr);
                LOG_INFO("Sent multicast attempt %d for Robot_%d\n", attempts + 1, robot_id);
                clock_delay(CLOCK_SECOND / 10); /* 100ms delay */
            }
            
            /* Method 2: Try direct addressing to robot */
            if (robot_id == 2 || robot_id == 3) {
                /* Construct direct IPv6 address for robot */
                uip_ip6addr(&dest_ipaddr, 0xfe80, 0, 0, 0, 
                           0x0200 + robot_id, 0x0000 + robot_id, 
                           0x0000 + robot_id, 0x0000 + robot_id);
                
                for (int attempts = 0; attempts < 3; attempts++) {
                    simple_udp_sendto(&udp_conn, &assignment, sizeof(assignment), &dest_ipaddr);
                    LOG_INFO("Sent direct attempt %d to Robot_%d\n", attempts + 1, robot_id);
                    clock_delay(CLOCK_SECOND / 10);
                }
            }
            
            LOG_INFO("Completed sending LA assignment to Robot_%d: LA_%d at (%d, %d)\n",
                     robot_id, la_id, assignment.center_x, assignment.center_y);
            break;
        }
    }
}

/*---------------------------------------------------------------------------*/
/* Base Station Process Thread */
PROCESS_THREAD(base_station_process, ev, data)
{
    static struct etimer timer, assignment_timer;
    
    PROCESS_BEGIN();
    
    LOG_INFO("Starting Base Station - WSN Deployment Coordinator\n");
    
    /* Initialize as routing root (DAG root) */
    NETSTACK_ROUTING.root_start();
    
    /* Initialize UDP connection */
    simple_udp_register(&udp_conn, UDP_SERVER_PORT, NULL,
                        UDP_CLIENT_PORT, udp_rx_callback);
    
    /* Initialize databases */
    init_la_db();
    
    /* Initialize Energest tracking */
    init_energest_tracking();
    
    LOG_INFO("Base Station initialized. Starting global phase...\n");
    
    /* Reduced wait time for faster startup */
    etimer_set(&timer, 30 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    
    LOG_INFO("Network should be stable now. Starting robot assignments...\n");
    
    /* Set up immediate assignment attempts */
    etimer_set(&assignment_timer, 1 * CLOCK_SECOND);
    
    /* Set up periodic energy update and statistics reporting */
    etimer_set(&timer, 5 * CLOCK_SECOND);
    
    while (1) {
        PROCESS_WAIT_EVENT();
        
        if (ev == PROCESS_EVENT_TIMER) {
            if (data == &assignment_timer && !assignments_sent) {
                LOG_INFO("Attempting robot assignments...\n");
                
                /* Initial robot assignments - assign robots to different LAs */
                /* Robot node IDs start from 2 in the simulation */
                for (uint8_t robot_node_id = 2; robot_node_id <= (1 + MAX_ROBOTS); robot_node_id++) {
                    if (assign_la_to_robot(robot_node_id)) {
                        LOG_INFO("Successfully assigned Robot_%d\n", robot_node_id);
                    } else {
                        LOG_INFO("Failed to assign Robot_%d - no available LAs\n", robot_node_id);
                    }
                }
                assignments_sent = true;
                
                /* Continue sending assignments periodically */
                etimer_set(&assignment_timer, 10 * CLOCK_SECOND);
                
            } else if (data == &assignment_timer && assignments_sent) {
                /* Retry assignments if robots haven't responded */
                LOG_INFO("Retrying robot assignments...\n");
                
                for (uint8_t robot_node_id = 2; robot_node_id <= (1 + MAX_ROBOTS); robot_node_id++) {
                    /* Find this robot's assignment and resend */
                    for (uint8_t i = 0; i < num_robots; i++) {
                        if (robot_db[i].robot_id == robot_node_id) {
                            send_la_assignment(robot_node_id, robot_db[i].assigned_la_id);
                            break;
                        }
                    }
                }
                
                etimer_set(&assignment_timer, 15 * CLOCK_SECOND);
                
            } else if (data == &timer) {
                /* Update base station energy */
                update_base_station_energy();
                
                /* Print statistics every 30 seconds */
                static uint8_t count = 0;
                if (++count >= 6) {
                    print_statistics();
                    count = 0;
                    
                    /* Check if deployment is complete */
                    bool deployment_complete = true;
                    for (uint8_t i = 0; i < num_las; i++) {
                        if (la_db[i].no_grid == 0) {
                            deployment_complete = false;
                            break;
                        }
                    }
                    
                    if (deployment_complete) {
                        LOG_INFO("Deployment complete! Final coverage: %.2f%%\n", 
                                 calculate_area_coverage());
                        LOG_INFO("Final energy consumption: %.4f J\n",
                                 calculate_total_network_energy());
                    }
                }
                
                etimer_reset(&timer);
            }
        }
    }
    
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/