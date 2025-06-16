/*
 * WSN Deployment Base Station Implementation
 * Based on the APP_I algorithm 
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

/* Energy tracking */
static double total_energy_consumed = 0.0;
static double processing_energy = 0.0;
static double radio_energy = 0.0;
static clock_time_t energy_last_update = 0;

/* Energy parameters for base station */
static const double power_processing_base = 0.5;    /* 500 mW processing power */
static const double power_transmit_base = 0.3;      /* 300 mW transmit power */
static const double power_receive_base = 0.2;       /* 200 mW receive power */

/* Timing parameters - used in baseline energy calculation */
static const double processing_time = 0.2;      /* T_P in seconds (higher for BS) */

/* Network energy tracking - reported by nodes */
static double robot_reported_energy[WSN_DEPLOYMENT_CONF_MAX_ROBOTS] = {0};
static double sensor_reported_energy[WSN_DEPLOYMENT_CONF_MAX_SENSORS] = {0};

/* Track assignment attempts */
static bool assignments_sent = false;

/* Forward declarations */
static void send_la_assignment(uint8_t robot_id, uint8_t la_id);

/*---------------------------------------------------------------------------*/
/* Calculate processing energy for base station - E_processing_base = P_processing_base * t_processing */
double calculate_processing_energy_base(double time_duration)
{
    return power_processing_base * processing_time * time_duration;
}

/*---------------------------------------------------------------------------*/
/* Calculate radio energy for base station - E_radio_base = E_transmit + E_receive */
double calculate_radio_energy_base(double time_transmit, double time_receive)
{
    double transmit_energy = power_transmit_base * time_transmit;
    double receive_energy = power_receive_base * time_receive;
    return transmit_energy + receive_energy;
}

/*---------------------------------------------------------------------------*/
/* Calculate total base station energy as per paper:
   E_base_station = E_processing_base + E_radio_base */
double calculate_base_station_energy(double processing, double radio)
{
    return processing + radio;
}

/*---------------------------------------------------------------------------*/
/* Calculate total network energy as per paper:
   Energy_tot = E_active + E_idle + E_robot + E_base_station */
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
    return sensor_energy + robot_energy + total_energy_consumed;
}

/*---------------------------------------------------------------------------*/
/* Update base station energy consumption */
static void update_base_station_energy(void)
{
    clock_time_t current_time = clock_time();
    if (energy_last_update == 0) {
        energy_last_update = current_time;
        return;
    }
    
    double time_duration = (double)(current_time - energy_last_update) / CLOCK_SECOND;
    if (time_duration <= 0) return;
    
    /* Processing energy */
    double processing = calculate_processing_energy_base(time_duration);
    processing_energy += processing;
    
    /* Radio energy (estimated based on communication activity) */
    double radio = calculate_radio_energy_base(
        time_duration * 0.05,  /* 5% tx time */
        time_duration * 0.10   /* 10% rx time */
    );
    radio_energy += radio;
    
    /* Calculate total energy using paper formula */
    double energy = calculate_base_station_energy(processing, radio);
    total_energy_consumed += energy;
    
    energy_last_update = current_time;
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
            
            /* Update radio energy for receiving the report */
            double rx_energy = power_receive_base * 0.01; /* Estimate 10ms reception */
            radio_energy += rx_energy;
            total_energy_consumed += rx_energy;
            
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
/* Print deployment statistics */
static void print_statistics(void)
{
    double coverage = calculate_area_coverage();
    double network_energy = calculate_total_network_energy();
    
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
    
    /* Print energy statistics */
    LOG_INFO("=== ENERGY STATISTICS ===\n");
    LOG_INFO("Base Station Energy: %.4f J\n", total_energy_consumed);
    LOG_INFO("  - Processing: %.4f J\n", processing_energy);
    LOG_INFO("  - Radio: %.4f J\n", radio_energy);
    LOG_INFO("Total Network Energy: %.4f J\n", network_energy);
}

/*---------------------------------------------------------------------------*/
/* Send LA assignment with simplified addressing - FIXED VERSION */
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
            
            /* Use link-local multicast for reliable delivery */
            uip_ipaddr_t dest_ipaddr;
            uip_create_linklocal_allnodes_mcast(&dest_ipaddr);
            
            /* Send message multiple times for reliability */
            for (int attempts = 0; attempts < 10; attempts++) {
                simple_udp_sendto(&udp_conn, &assignment, sizeof(assignment), &dest_ipaddr);
                LOG_INFO("Sent attempt %d for Robot_%d\n", attempts + 1, robot_id);
                
                /* Add delay between transmissions */
                clock_delay(CLOCK_SECOND / 4); /* 250ms delay */
            }
            
            /* Update radio energy for transmission */
            double tx_energy = power_transmit_base * 0.1; /* Multiple transmissions */
            radio_energy += tx_energy;
            total_energy_consumed += tx_energy;
            
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
    simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL,
                        UDP_SERVER_PORT, udp_rx_callback);
    
    /* Initialize databases */
    init_la_db();
    
    /* Initialize energy tracking */
    energy_last_update = clock_time();
    
    LOG_INFO("Base Station initialized. Starting global phase...\n");
    
    /* Wait longer for network to stabilize and routing to be established */
    etimer_set(&timer, 60 * CLOCK_SECOND); /* Increased to 60 seconds */
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    
    LOG_INFO("Network should be stable now. Starting robot assignments...\n");
    
    /* Set up periodic assignment attempts */
    etimer_set(&assignment_timer, 2 * CLOCK_SECOND);
    
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
                
                /* Continue sending assignments periodically until robots respond */
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