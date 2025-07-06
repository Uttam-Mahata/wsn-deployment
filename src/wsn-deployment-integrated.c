/*
 * WSN Deployment Integrated Application - Contiki-NG Implementation
 * This file demonstrates the complete local phase implementation
 * based on the main.tex specifications
 */

#include "common.h"

// Process declarations
PROCESS(wsn_deployment_main, "WSN Deployment Main Process");
PROCESS(system_monitor, "System Monitor Process");
AUTOSTART_PROCESSES(&wsn_deployment_main, &system_monitor);

// Global system state
static struct {
    uint8_t system_initialized;
    uint8_t base_station_active;
    uint8_t robots_deployed;
    uint8_t sensors_deployed;
    uint32_t total_coverage_percentage;
    uint32_t simulation_time;
} system_state = {0};

// Network connection for inter-node communication
static struct simple_udp_connection broadcast_conn;

// System initialization
void initialize_wsn_system() {
    LOG_INFO("=== Initializing WSN Deployment System ===\n");
    
    // Print system parameters
    print_system_parameters();
    
    // Initialize network for broadcast communication
    simple_udp_register(&broadcast_conn, UDP_CLIENT_PORT, NULL, 
                       UDP_SERVER_PORT, NULL);
    
    system_state.system_initialized = 1;
    LOG_INFO("WSN Deployment System initialized successfully\n");
}

// Simulate the complete local phase scenario
void simulate_local_phase_scenario() {
    LOG_INFO("\n=== Simulating Complete Local Phase Scenario ===\n");
    
    // Scenario parameters based on main.tex
    uint32_t no_la = (uint32_t)floor(target_area_size / robot_perception_range);
    uint32_t no_g = (uint32_t)floor(robot_perception_range / sensor_perception_range);
    
    LOG_INFO("Scenario Setup:\n");
    LOG_INFO("- Target area: %.2f x %.2f\n", target_area_size, target_area_size);
    LOG_INFO("- Number of Location Areas: %u\n", no_la);
    LOG_INFO("- Grids per Location Area: %u\n", no_g);
    LOG_INFO("- Total grids in system: %u\n", no_la * no_g);
    
    // Simulate Algorithm 1: Global Phase
    LOG_INFO("\n--- Algorithm 1: Global Phase Execution ---\n");
    LOG_INFO("1. Base station partitions target area into %u LAs\n", no_la);
    LOG_INFO("2. Base station initializes databases (LA_DB, Robot_DB)\n");
    LOG_INFO("3. Robot_1 assigned to LA_1, Robot_2 assigned to LA_%u\n", no_la);
    LOG_INFO("4. Each robot receives %u sensors in stock\n", STOCK_RS);
    
    // Simulate Algorithm 2: Local Phase for both robots
    for (uint32_t robot_id = 0; robot_id < MAX_ROBOTS; robot_id++) {
        uint32_t assigned_la = (robot_id == 0) ? 1 : no_la;
        
        LOG_INFO("\n--- Algorithm 2: Local Phase for Robot_%u in LA_%u ---\n", 
                 robot_id + 1, assigned_la);
        
        // Step 1: Divide LA into grids
        LOG_INFO("1. Robot_%u divides LA_%u into %u grids\n", 
                 robot_id + 1, assigned_la, no_g);
        
        // Step 2: Insert records into Grid_DB
        LOG_INFO("2. Robot_%u inserts %u records in Grid_DB\n", 
                 robot_id + 1, no_g);
        
        // Step 3: Algorithm 3 - Topology Discovery Phase
        simulate_topology_discovery_phase(robot_id, assigned_la);
        
        // Step 4: Initialize NO_P
        LOG_INFO("4. Robot_%u initializes NO_P = %u\n", robot_id + 1, no_g);
        
        // Step 5: Algorithm 4 - Dispersion Phase
        uint32_t covered_grids = simulate_dispersion_phase(robot_id, assigned_la, no_g);
        
        // Step 6-8: Count and report covered grids
        LOG_INFO("6. Robot_%u counts %u covered grids in LA_%u\n", 
                 robot_id + 1, covered_grids, assigned_la);
        LOG_INFO("7. Robot_%u sends completion message to base station\n", 
                 robot_id + 1);
        
        // Update system state
        system_state.total_coverage_percentage += 
            (covered_grids * 100) / (no_la * no_g);
    }
    
    // Calculate final area coverage
    float final_coverage = (float)system_state.total_coverage_percentage / MAX_ROBOTS;
    LOG_INFO("\n--- Final System Results ---\n");
    LOG_INFO("Total area coverage achieved: %.2f%%\n", final_coverage);
    
    // Demonstrate message size calculations from main.tex
    demonstrate_message_sizes(no_g);
}

// Algorithm 3: Topology Discovery Phase simulation
void simulate_topology_discovery_phase(uint32_t robot_id, uint32_t la_id) {
    LOG_INFO("\n--- Algorithm 3: Topology Discovery Phase ---\n");
    LOG_INFO("1. Robot_%u moves to center of LA_%u\n", robot_id + 1, la_id);
    LOG_INFO("2. Robot_%u broadcasts Mp message (size: %u bit)\n", 
             robot_id + 1, calculate_mp_message_size());
    
    // Simulate sensor responses
    uint32_t sensors_in_range = (random_rand() % 8) + 2;  // 2-9 sensors
    LOG_INFO("3. Robot_%u receives %u Sensor_M replies\n", 
             robot_id + 1, sensors_in_range);
    
    for (uint32_t i = 0; i < sensors_in_range; i++) {
        uint32_t sensor_id = la_id * 100 + i + 1;
        LOG_INFO("   - Sensor_%u replied with position and status\n", sensor_id);
    }
    
    LOG_INFO("4. Robot_%u updates local Sensor_DB with %u entries\n", 
             robot_id + 1, sensors_in_range);
}

// Algorithm 4: Dispersion Phase simulation
uint32_t simulate_dispersion_phase(uint32_t robot_id, uint32_t la_id, uint32_t no_g) {
    LOG_INFO("\n--- Algorithm 4: Dispersion Phase ---\n");
    
    uint32_t stock_rs = STOCK_RS;
    uint32_t no_p = no_g;
    uint32_t covered_grids = 0;
    
    LOG_INFO("Robot_%u starting dispersion with Stock_RS=%u, NO_P=%u\n", 
             robot_id + 1, stock_rs, no_p);
    
    for (uint32_t grid = 1; grid <= no_g && no_p > 0; grid++) {
        // Simulate random grid conditions
        uint8_t grid_has_sensors = (random_rand() % 100) < 60;  // 60% chance
        uint32_t sensors_in_grid = grid_has_sensors ? (random_rand() % 3) + 1 : 0;
        
        LOG_INFO("\nProcessing Grid_%u (NO_P=%u):\n", grid, no_p);
        LOG_INFO("- Grid has %u sensors\n", sensors_in_grid);
        LOG_INFO("- Robot stock: %u sensors\n", stock_rs);
        
        // Apply the four cases from main.tex
        if (stock_rs > 0 && sensors_in_grid > 0) {
            // Case 1: Robot has sensors in Stock_RS and G_i has sensors
            LOG_INFO("- Executing Case 1:\n");
            LOG_INFO("  * Place sensor from stock at grid center\n");
            LOG_INFO("  * Collect %u extra sensors from grid\n", sensors_in_grid);
            LOG_INFO("  * Mark Grid_%u as COVERED\n", grid);
            
            stock_rs = (stock_rs - 1 + sensors_in_grid > ROBOT_CAPACITY) ? 
                      ROBOT_CAPACITY : stock_rs - 1 + sensors_in_grid;
            covered_grids++;
            
        } else if (stock_rs > 0 && sensors_in_grid == 0) {
            // Case 2: Robot has sensors but grid is empty
            LOG_INFO("- Executing Case 2:\n");
            LOG_INFO("  * Place sensor from stock at grid center\n");
            LOG_INFO("  * Mark Grid_%u as COVERED\n", grid);
            
            stock_rs--;
            covered_grids++;
            
        } else if (stock_rs == 0 && sensors_in_grid > 0) {
            // Case 3: Robot has no sensors but grid has sensors
            LOG_INFO("- Executing Case 3:\n");
            LOG_INFO("  * Find closest sensor to grid center\n");
            LOG_INFO("  * Move closest sensor to grid center\n");
            LOG_INFO("  * Collect %u extra sensors\n", sensors_in_grid - 1);
            LOG_INFO("  * Mark Grid_%u as COVERED\n", grid);
            
            stock_rs += (sensors_in_grid - 1);
            if (stock_rs > ROBOT_CAPACITY) stock_rs = ROBOT_CAPACITY;
            covered_grids++;
            
        } else {
            // Case 4: No sensors available
            LOG_INFO("- Executing Case 4:\n");
            LOG_INFO("  * Grid_%u remains UNCOVERED\n", grid);
        }
        
        no_p--;
        LOG_INFO("- Decremented NO_P to %u\n", no_p);
        
        if (no_p > 0) {
            LOG_INFO("- Moving to next uncovered grid\n");
        }
    }
    
    LOG_INFO("\nRobot_%u dispersion completed:\n", robot_id + 1);
    LOG_INFO("- Covered grids: %u/%u\n", covered_grids, no_g);
    LOG_INFO("- Final stock: %u sensors\n", stock_rs);
    LOG_INFO("- Remaining moves: %u\n", no_p);
    
    return covered_grids;
}

// Demonstrate message sizes from main.tex
void demonstrate_message_sizes(uint32_t no_g) {
    LOG_INFO("\n--- Message Size Analysis (from main.tex) ---\n");
    
    uint32_t robot_pm_size = calculate_robot_pm_message_size(no_g);
    uint32_t mp_size = calculate_mp_message_size();
    uint32_t sensor_m_size = calculate_sensor_m_message_size(total_sensors);
    
    LOG_INFO("Robot_pM message size: %u bits\n", robot_pm_size);
    LOG_INFO("- Format: (Robot_p, Cov_G)\n");
    LOG_INFO("- Robot_p: 1 bit, Cov_G: %u bits\n", 
             (uint32_t)ceil(log2f((float)no_g)));
    
    LOG_INFO("Mp broadcast message size: %u bit\n", mp_size);
    LOG_INFO("- Format: (Robot_p)\n");
    LOG_INFO("- Robot_p: 1 bit\n");
    
    LOG_INFO("Sensor_M reply message size: %u bits\n", sensor_m_size);
    LOG_INFO("- Format: (Sensor_id, X-Y coordinate, Sensor_Status)\n");
    LOG_INFO("- Sensor_id: %u bits, Coordinate: 16 bits, Status: 1 bit\n", 
             (uint32_t)ceil(log2f((float)total_sensors)));
}

// System monitoring and statistics
void monitor_system_performance() {
    system_state.simulation_time++;
    
    if (system_state.simulation_time % 100 == 0) {  // Every 100 time units
        LOG_INFO("\n=== System Performance Monitor ===\n");
        LOG_INFO("Simulation time: %u units\n", system_state.simulation_time);
        LOG_INFO("System initialized: %s\n", 
                 system_state.system_initialized ? "YES" : "NO");
        LOG_INFO("Base station active: %s\n", 
                 system_state.base_station_active ? "YES" : "NO");
        LOG_INFO("Robots deployed: %s\n", 
                 system_state.robots_deployed ? "YES" : "NO");
        LOG_INFO("Sensors deployed: %s\n", 
                 system_state.sensors_deployed ? "YES" : "NO");
        
        // Calculate and display database sizes
        uint32_t no_la = (uint32_t)floor(target_area_size / robot_perception_range);
        uint32_t no_g = (uint32_t)floor(robot_perception_range / sensor_perception_range);
        
        LOG_INFO("\nDatabase Memory Usage:\n");
        LOG_INFO("- LA_DB: %u bits (%.2f bytes)\n", 
                 calculate_la_db_size(no_la, no_g),
                 calculate_la_db_size(no_la, no_g) / 8.0f);
        LOG_INFO("- Robot_DB: %u bits (%.2f bytes)\n", 
                 calculate_robot_db_size(no_la),
                 calculate_robot_db_size(no_la) / 8.0f);
        LOG_INFO("- Grid_DB: %u bits (%.2f bytes)\n", 
                 calculate_grid_db_size(no_g),
                 calculate_grid_db_size(no_g) / 8.0f);
        LOG_INFO("- Sensor_DB: %u bits (%.2f bytes)\n", 
                 calculate_sensor_db_size(total_sensors),
                 calculate_sensor_db_size(total_sensors) / 8.0f);
    }
}

// Main WSN deployment process
PROCESS_THREAD(wsn_deployment_main, ev, data)
{
    static struct etimer init_timer, scenario_timer;
    
    PROCESS_BEGIN();
    
    LOG_INFO("=== WSN Deployment Integrated Application - Contiki-NG ===\n");
    LOG_INFO("Implementation based on main.tex APP_I specifications\n");
    
    // Initialize system
    etimer_set(&init_timer, CLOCK_SECOND * 2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&init_timer));
    
    initialize_wsn_system();
    
    // Start local phase scenario simulation
    etimer_set(&scenario_timer, CLOCK_SECOND * 5);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&scenario_timer));
    
    simulate_local_phase_scenario();
    
    // Mark system as fully operational
    system_state.base_station_active = 1;
    system_state.robots_deployed = 1;
    system_state.sensors_deployed = 1;
    
    LOG_INFO("\n=== WSN Deployment System Fully Operational ===\n");
    
    // Continue running for monitoring
    while(1) {
        PROCESS_YIELD();
    }
    
    PROCESS_END();
}

// System monitor process
PROCESS_THREAD(system_monitor, ev, data)
{
    static struct etimer monitor_timer;
    
    PROCESS_BEGIN();
    
    // Start monitoring after system initialization
    etimer_set(&monitor_timer, CLOCK_SECOND * 10);
    
    while(1) {
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&monitor_timer));
        
        monitor_system_performance();
        
        // Reset timer for next monitoring cycle
        etimer_set(&monitor_timer, CLOCK_SECOND * 30);
    }
    
    PROCESS_END();
}
