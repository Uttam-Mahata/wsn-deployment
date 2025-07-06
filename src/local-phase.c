#include "common.h"
#include <string.h>

// External function declarations from mobile-robot.c
extern void initialize_mobile_robot(mobile_robot_t *robot, robot_id_t id, uint32_t la_id, coordinate_t la_center);
extern void execute_local_phase(mobile_robot_t *robot);
extern void cleanup_mobile_robot(mobile_robot_t *robot);

// External function declarations from sensor-node.c
extern void deploy_random_sensors(uint32_t num_sensors, float area_size);
extern void display_sensor_states();

// Integrated local phase execution
void run_integrated_local_phase() {
    printf("=== Integrated WSN Deployment Local Phase ===\n");
    
    // Step 1: Deploy random sensors in the environment
    printf("\n--- Phase 1: Random Sensor Deployment ---\n");
    deploy_random_sensors(50, target_area_size);
    display_sensor_states();
    
    // Step 2: Initialize and deploy mobile robots
    printf("\n--- Phase 2: Mobile Robot Deployment ---\n");
    
    // Create two mobile robots as per specification
    mobile_robot_t robot1, robot2;
    
    // Initialize Robot_1 in LA_1
    coordinate_t la1_center = {robot_perception_range / 2, robot_perception_range / 2};
    initialize_mobile_robot(&robot1, ROBOT_1, 1, la1_center);
    
    // Initialize Robot_2 in LA_NO_LA (last LA)
    uint32_t no_la = (uint32_t)floor(target_area_size / robot_perception_range);
    coordinate_t la_last_center = {
        target_area_size - robot_perception_range / 2, 
        target_area_size - robot_perception_range / 2
    };
    initialize_mobile_robot(&robot2, ROBOT_2, no_la, la_last_center);
    
    // Step 3: Execute local phase for both robots
    printf("\n--- Phase 3: Local Phase Execution ---\n");
    
    // Execute local phase for Robot_1
    printf("\n### Robot_1 Local Phase ###\n");
    execute_local_phase(&robot1);
    
    // Execute local phase for Robot_2
    printf("\n### Robot_2 Local Phase ###\n");
    execute_local_phase(&robot2);
    
    // Step 4: Display final deployment results
    printf("\n--- Phase 4: Final Deployment Results ---\n");
    
    // Calculate overall area coverage
    uint32_t total_grids = robot1.no_g + robot2.no_g;
    uint32_t total_covered = 0;
    
    for (uint32_t i = 0; i < robot1.no_g; i++) {
        if (robot1.grid_db[i].grid_status == COVERED) {
            total_covered++;
        }
    }
    
    for (uint32_t i = 0; i < robot2.no_g; i++) {
        if (robot2.grid_db[i].grid_status == COVERED) {
            total_covered++;
        }
    }
    
    float percentage_coverage = ((float)total_covered / total_grids) * 100.0;
    
    printf("\n=== Deployment Summary ===\n");
    printf("Total Grids: %u\n", total_grids);
    printf("Covered Grids: %u\n", total_covered);
    printf("Percentage Area Coverage: %.2f%%\n", percentage_coverage);
    printf("Robot_1 Stock Remaining: %u sensors\n", robot1.stock_rs);
    printf("Robot_2 Stock Remaining: %u sensors\n", robot2.stock_rs);
    
    // Step 5: Display message sizes
    printf("\n=== Message Size Analysis ===\n");
    uint32_t no_g = robot1.no_g;  // Assuming same for both robots
    
    // Robot_pM message size
    uint32_t robot_pm_size = 1 + (uint32_t)ceil(log2(no_g));
    printf("Robot_pM message size: %u bits\n", robot_pm_size);
    
    // Mp message size
    uint32_t mp_size = 1;
    printf("Mp broadcast message size: %u bits\n", mp_size);
    
    // Sensor_M message size
    uint32_t sensor_m_size = (uint32_t)ceil(log2(total_sensors)) + 17;
    printf("Sensor_M reply message size: %u bits\n", sensor_m_size);
    
    // Step 6: Cleanup
    cleanup_mobile_robot(&robot1);
    cleanup_mobile_robot(&robot2);
    
    printf("\nLocal phase execution completed successfully!\n");
}

// Demonstrate the four dispersion cases
void demonstrate_dispersion_cases() {
    printf("\n=== Demonstrating Dispersion Cases ===\n");
    
    mobile_robot_t demo_robot;
    coordinate_t demo_center = {300.0, 300.0};
    initialize_mobile_robot(&demo_robot, ROBOT_1, 3, demo_center);
    
    // Manually set up scenarios for each case
    divide_la_into_grids(&demo_robot);
    
    printf("\n--- Case Demonstrations ---\n");
    
    // Case 1: Robot has sensors, grid has sensors
    printf("\nCase 1 Demo: Robot has sensors in stock, grid has sensors\n");
    demo_robot.stock_rs = 5;
    handle_dispersion_case_1(&demo_robot, 0);
    
    // Case 2: Robot has sensors, grid empty
    printf("\nCase 2 Demo: Robot has sensors in stock, grid is empty\n");
    demo_robot.stock_rs = 3;
    handle_dispersion_case_2(&demo_robot, 1);
    
    // Case 3: Robot has no sensors, grid has sensors
    printf("\nCase 3 Demo: Robot has no sensors in stock, grid has sensors\n");
    // Add some sensors to the sensor database first
    demo_robot.sensor_db[demo_robot.sensor_db_count++] = (sensor_db_record_t){
        .sensor_id = 301,
        .coord = {305.0, 305.0},
        .sensor_status = IDLE
    };
    demo_robot.stock_rs = 0;
    handle_dispersion_case_3(&demo_robot, 2);
    
    // Case 4: Robot has no sensors, grid empty
    printf("\nCase 4 Demo: Robot has no sensors in stock, grid is empty\n");
    demo_robot.stock_rs = 0;
    handle_dispersion_case_4(&demo_robot, 3);
    
    cleanup_mobile_robot(&demo_robot);
}

// Performance analysis
void analyze_performance() {
    printf("\n=== Performance Analysis ===\n");
    
    // Calculate database sizes
    uint32_t no_la = (uint32_t)floor(target_area_size / robot_perception_range);
    uint32_t no_g = (uint32_t)floor(robot_perception_range / sensor_perception_range);
    
    uint32_t la_db_size = calculate_la_db_size(no_la, no_g);
    uint32_t robot_db_size = calculate_robot_db_size(no_la);
    uint32_t grid_db_size = calculate_grid_db_size(no_g);
    uint32_t sensor_db_size = calculate_sensor_db_size(total_sensors);
    
    printf("Database Size Analysis:\n");
    printf("LA_DB size: %u bits (%.2f KB)\n", la_db_size, la_db_size / 8192.0);
    printf("Robot_DB size: %u bits (%.2f KB)\n", robot_db_size, robot_db_size / 8192.0);
    printf("Grid_DB size: %u bits (%.2f KB)\n", grid_db_size, grid_db_size / 8192.0);
    printf("Sensor_DB size: %u bits (%.2f KB)\n", sensor_db_size, sensor_db_size / 8192.0);
    
    printf("\nSystem Parameters:\n");
    printf("Target area size: %.2f x %.2f\n", target_area_size, target_area_size);
    printf("Robot perception range: %.2f\n", robot_perception_range);
    printf("Sensor perception range: %.2f\n", sensor_perception_range);
    printf("Number of Location Areas (NO_LA): %u\n", no_la);
    printf("Number of Grids per LA (NO_G): %u\n", no_g);
    printf("Robot capacity: %u sensors\n", ROBOT_CAPACITY);
    printf("Initial stock per robot (Stock_RS): %u sensors\n", STOCK_RS);
}

// Main function for local phase testing
int main() {
    printf("=== WSN Deployment Local Phase Implementation ===\n");
    
    // Run the complete integrated local phase
    run_integrated_local_phase();
    
    // Demonstrate the four dispersion cases
    demonstrate_dispersion_cases();
    
    // Analyze performance
    analyze_performance();
    
    return 0;
}