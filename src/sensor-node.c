/*
 * WSN Deployment Sensor Node Implementation
 
 * 1. Response to robot discovery messages (Sensor_M)
 * 2. Active/Idle mode energy calculations
 * 3. Comprehensive energy model including baseline, sensing, processing, radio
 * 4. Proper sensor status management
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

#define LOG_MODULE "Sensor"
#define LOG_LEVEL LOG_LEVEL_INFO

/* Sensor Node Process */
PROCESS(sensor_node_process, "Sensor Node Process");
AUTOSTART_PROCESSES(&sensor_node_process);

/* Network communication */
static struct simple_udp_connection udp_conn;

/* Sensor state */
static uint8_t sensor_id;
static position_t sensor_position;
static uint8_t sensor_status = 0; /* 0 = idle, 1 = active */
static bool is_deployed = false;

/* Energy tracking */
static double total_energy_consumed = 0.0;
static double baseline_energy = 0.0;
static double sensing_energy = 0.0;
static double processing_energy = 0.0;
static double radio_energy = 0.0;
static double active_energy = 0.0;
static double idle_energy = 0.0;
static clock_time_t energy_last_update = 0;

/* Sensing parameters */
static uint8_t sensing_range = SENSOR_PERCEPTION_RANGE;
static uint8_t communication_range = WSN_DEPLOYMENT_CONF_COMM_RANGE;

/* Energy parameters (typical values for sensor nodes) */
static const double power_baseline = 0.003;     /* 3 mW baseline power */
static const double power_processing = 0.020;   /* 20 mW processing power */
static const double power_transmit = 0.050;     /* 50 mW transmit power */
static const double power_receive = 0.030;      /* 30 mW receive power */
static const double mu = 0.0005;                /* Energy coefficient for sensing */

/* Timing parameters from paper */
static const double interval_count = 10.0;      /* I_C */
static const double cycle_time = 1.0;           /* T_C in seconds */
static const double processing_time = 0.1;      /* T_P in seconds */

/*---------------------------------------------------------------------------*/
/* Calculate baseline energy consumption - E_baseline = (t_i - t_s) * P_baseline */
double calculate_baseline_energy(double time_duration, double power_baseline)
{
    /* t_i = I_C * (T_C + T_P) */
    double t_i = interval_count * (cycle_time + processing_time);
    double t_s = 0; /* Start time, assuming 0 for simplicity */
    
    return (t_i - t_s) * power_baseline * time_duration;
}

/*---------------------------------------------------------------------------*/
/* Calculate sensing energy consumption - E_sensing = μ * r_i^2 */
double calculate_sensing_energy(double sensing_range)
{
    return mu * sensing_range * sensing_range;
}

/*---------------------------------------------------------------------------*/
/* Calculate processing energy consumption - E_processing = P_processing * t_processing */
double calculate_processing_energy(double power_processing, double time_processing)
{
    return power_processing * time_processing;
}

/*---------------------------------------------------------------------------*/
/* Calculate radio energy consumption - E_radio = E_transmit + E_receive */
double calculate_radio_energy(double power_transmit, double time_transmit, 
                             double power_receive, double time_receive)
{
    double transmit_energy = power_transmit * time_transmit;
    double receive_energy = power_receive * time_receive;
    return transmit_energy + receive_energy;
}

/*---------------------------------------------------------------------------*/
/* Calculate active mode energy as per paper:
   E_active = E_baseline + E_sensing + E_processing + E_radio */
static double calculate_active_energy(double baseline, double sensing, 
                                    double processing, double radio)
{
    return baseline + sensing + processing + radio;
}

/*---------------------------------------------------------------------------*/
/* Calculate idle mode energy as per paper:
   E_idle = E_baseline + E_radio (reduced) */
static double calculate_idle_energy(double baseline, double radio)
{
    return baseline + radio;
}

/*---------------------------------------------------------------------------*/
/* Update energy consumption based on current mode */
static void update_energy_consumption(void)
{
    clock_time_t current_time = clock_time();
    if (energy_last_update == 0) {
        energy_last_update = current_time;
        return;
    }
    
    double time_duration = (double)(current_time - energy_last_update) / CLOCK_SECOND;
    if (time_duration <= 0) return;
    
    /* Calculate baseline energy (always consumed) */
    double baseline = calculate_baseline_energy(time_duration, power_baseline);
    baseline_energy += baseline;
    
    if (sensor_status == 1) { /* Active mode */
        LOG_INFO("Sensor_%d in ACTIVE mode\n", sensor_id);
        
        /* Sensing energy (only in active mode) */
        double sensing = calculate_sensing_energy(sensing_range);
        sensing_energy += sensing;
        
        /* Processing energy (more in active mode) */
        double processing = calculate_processing_energy(power_processing, processing_time * time_duration);
        processing_energy += processing;
        
        /* Radio energy (estimated based on activity) */
        double radio = calculate_radio_energy(power_transmit, time_duration * 0.02, /* 2% tx time */
                                            power_receive, time_duration * 0.05); /* 5% rx time */
        radio_energy += radio;
        
        /* Calculate active mode energy using paper formula */
        double active = calculate_active_energy(baseline, sensing, processing, radio);
        active_energy += active;
        total_energy_consumed += active;
        
    } else { /* Idle mode */
        /* Only baseline and minimal radio energy in idle mode */
        double radio = calculate_radio_energy(power_transmit, time_duration * 0.001, /* 0.1% tx time */
                                            power_receive, time_duration * 0.02);  /* 2% rx time */
        radio_energy += radio;
        
        /* Calculate idle mode energy using paper formula */
        double idle = calculate_idle_energy(baseline, radio);
        idle_energy += idle;
        total_energy_consumed += idle;
    }
    
    energy_last_update = current_time;
}

/*---------------------------------------------------------------------------*/
/* Calculate total energy consumption according to paper formula:
   Energy_Tot = E_active + E_idle (for sensors) */
static double calculate_total_sensor_energy(void)
{
    return active_energy + idle_energy;
}

/*---------------------------------------------------------------------------*/
/* Handle robot discovery message */
static void handle_robot_discovery(const uip_ipaddr_t *sender_addr, 
                                 robot_broadcast_msg_t *msg)
{
    LOG_INFO("Received Mp discovery message from Robot_%d\n", msg->robot_id);
    
    /* Calculate distance to robot (simplified approach) */
    /* In real implementation, would use actual robot position */
    
    /* Send sensor reply message (Sensor_M) */
    sensor_reply_msg_t reply;
    reply.msg_type = WSN_MSG_TYPE_SENSOR_REPLY;
    reply.sensor_id = sensor_id;
    reply.x_coord = sensor_position.x;
    reply.y_coord = sensor_position.y;
    reply.sensor_status = sensor_status;
    
    simple_udp_sendto(&udp_conn, &reply, sizeof(reply), sender_addr);
    
    LOG_INFO("Sent Sensor_M reply to Robot_%d: Sensor_%d at (%d, %d), status: %s\n",
             msg->robot_id, sensor_id, sensor_position.x, sensor_position.y,
             sensor_status ? "active" : "idle");
    
    /* Update radio energy for transmission */
    double tx_energy = power_transmit * 0.01; /* Estimate 10ms transmission */
    radio_energy += tx_energy;
    total_energy_consumed += tx_energy;
}

/*---------------------------------------------------------------------------*/
/* Simulate sensor deployment by robot */
static void simulate_deployment(position_t new_position) __attribute__((unused));
static void simulate_deployment(position_t new_position)
{
    LOG_INFO("Sensor_%d being deployed to new position (%d, %d)\n",
             sensor_id, new_position.x, new_position.y);
    
    sensor_position = new_position;
    sensor_status = 1; /* Activate sensor */
    is_deployed = true;
    
    LOG_INFO("Sensor_%d now ACTIVE at (%d, %d)\n", 
             sensor_id, sensor_position.x, sensor_position.y);
}

/*---------------------------------------------------------------------------*/
/* Simulate sensing operation */
static void perform_sensing(void)
{
    if (sensor_status == 1) { /* Only active sensors perform sensing */
        /* Simulate data capture and processing */
        uint16_t sensor_data = random_rand() % 1024; /* Random sensor reading */
        
        LOG_INFO("Sensor_%d captured data: %d (range: %d m)\n", 
                 sensor_id, sensor_data, sensing_range);
        
        /* Add sensing energy */
        double sensing = calculate_sensing_energy(sensing_range);
        sensing_energy += sensing;
        
        /* Add processing energy for data processing */
        double processing = calculate_processing_energy(power_processing, 0.1); /* 100ms processing */
        processing_energy += processing;
        
        /* Calculate active energy contribution */
        double baseline = calculate_baseline_energy(0.1, power_baseline);
        double radio = calculate_radio_energy(power_transmit, 0.01, power_receive, 0.01);
        
        double active = calculate_active_energy(baseline, sensing, processing, radio);
        active_energy += active;
        total_energy_consumed += active;
    }
}

/*---------------------------------------------------------------------------*/
/* Print sensor statistics */
static void print_sensor_statistics(void)
{
    /* Recalculate total energy using paper's formula */
    total_energy_consumed = calculate_total_sensor_energy();
    
    LOG_INFO("=== SENSOR STATISTICS ===\n");
    LOG_INFO("Sensor ID: %d\n", sensor_id);
    LOG_INFO("Position: (%d, %d)\n", sensor_position.x, sensor_position.y);
    LOG_INFO("Status: %s\n", sensor_status ? "ACTIVE" : "IDLE");
    LOG_INFO("Sensing Range: %d m\n", sensing_range);
    LOG_INFO("Communication Range: %d m\n", communication_range);
    LOG_INFO("Deployed: %s\n", is_deployed ? "Yes" : "No");
    LOG_INFO("Total Energy: %.6f J\n", total_energy_consumed);
    
    /* Energy breakdown */
    LOG_INFO("Energy Breakdown:\n");
    LOG_INFO("  Baseline: %.6f J (%.1f%%)\n", 
             baseline_energy, (baseline_energy/total_energy_consumed)*100);
    LOG_INFO("  Sensing: %.6f J (%.1f%%)\n", 
             sensing_energy, (sensing_energy/total_energy_consumed)*100);
    LOG_INFO("  Processing: %.6f J (%.1f%%)\n", 
             processing_energy, (processing_energy/total_energy_consumed)*100);
    LOG_INFO("  Radio: %.6f J (%.1f%%)\n", 
             radio_energy, (radio_energy/total_energy_consumed)*100);
    
    /* Report by mode */
    LOG_INFO("  Active Mode: %.6f J\n", active_energy);
    LOG_INFO("  Idle Mode: %.6f J\n", idle_energy);
    
    /* Calculate theoretical values according to paper */
    if (sensor_status == 1) {
        LOG_INFO("Current mode: ACTIVE (E_active)\n");
    } else {
        LOG_INFO("Current mode: IDLE (E_idle)\n");
    }
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
    /* Handle robot discovery messages */
    if (datalen == sizeof(robot_broadcast_msg_t)) {
        robot_broadcast_msg_t *msg = (robot_broadcast_msg_t *)data;
        
        if (msg->msg_type == WSN_MSG_TYPE_ROBOT_BROADCAST) {
            handle_robot_discovery(sender_addr, msg);
        }
    }
    
    /* Handle deployment commands (if any) */
    /* This could be extended to handle robot deployment commands */
}

/*---------------------------------------------------------------------------*/
/* Sensor Node Process Thread */
PROCESS_THREAD(sensor_node_process, ev, data)
{
    static struct etimer energy_timer, sensing_timer, stats_timer;
    
    PROCESS_BEGIN();
    
    /* Initialize sensor */
    sensor_id = node_id;
    
    /* Set initial position based on node ID (from log file) */
    switch(sensor_id) {
        case 4:
            sensor_position.x = 912;
            sensor_position.y = 556;
            break;
        case 6:
            sensor_position.x = 377;
            sensor_position.y = 929;
            break;
        case 7:
            sensor_position.x = 720;
            sensor_position.y = 476;
            break;
        case 8:
            sensor_position.x = 277;
            sensor_position.y = 483;
            break;
        case 11:
            sensor_position.x = 609;
            sensor_position.y = 123;
            break;
        default:
            /* Random position for other sensors */
            sensor_position.x = 100 + (random_rand() % 800);
            sensor_position.y = 100 + (random_rand() % 800);
            break;
    }
    
    energy_last_update = clock_time();
    
    LOG_INFO("Starting Sensor Node_%d\n", sensor_id);
    LOG_INFO("Sensor_%d initial position: (%d, %d)\n", 
             sensor_id, sensor_position.x, sensor_position.y);
    LOG_INFO("Sensor range: %d m, Communication range: %d m\n", 
             sensing_range, communication_range);
    
    /* Initialize UDP connection */
    simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL,
                       UDP_SERVER_PORT, udp_rx_callback);
    
    /* Set up timers */
    etimer_set(&energy_timer, 1 * CLOCK_SECOND);      /* Update energy every second */
    etimer_set(&sensing_timer, 5 * CLOCK_SECOND);     /* Perform sensing every 5 seconds */
    etimer_set(&stats_timer, 60 * CLOCK_SECOND);      /* Print stats every minute */
    
    while (1) {
        PROCESS_WAIT_EVENT();
        
        if (ev == PROCESS_EVENT_TIMER) {
            if (data == &energy_timer) {
                /* Update energy consumption */
                update_energy_consumption();
                
                /* Reset timer */
                etimer_set(&energy_timer, 1 * CLOCK_SECOND);
                
            } else if (data == &sensing_timer) {
                /* Perform sensing operation */
                perform_sensing();
                
                /* Reset timer */
                etimer_set(&sensing_timer, 5 * CLOCK_SECOND);
                
            } else if (data == &stats_timer) {
                /* Print statistics */
                print_sensor_statistics();
                
                /* Reset timer */
                etimer_set(&stats_timer, 60 * CLOCK_SECOND);
            }
        }
    }
    
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
