#include "common.h"
#include <string.h>

// Global sensor registry
typedef struct {
    uint32_t sensor_id;
    coordinate_t position;
    sensor_status_t status;
    float battery_level;
    uint32_t is_active;
} sensor_node_t;

static sensor_node_t sensor_registry[1000];
static uint32_t sensor_count = 0;

// Initialize sensor node
void initialize_sensor_node(uint32_t sensor_id, coordinate_t position) {
    if (sensor_count < 1000) {
        sensor_registry[sensor_count] = (sensor_node_t){
            .sensor_id = sensor_id,
            .position = position,
            .status = IDLE,
            .battery_level = 100.0,
            .is_active = 1
        };
        sensor_count++;
        
        printf("Sensor_%u initialized at position (%.2f, %.2f)\n", 
               sensor_id, position.x, position.y);
    }
}

// Handle robot broadcast message
sensor_reply_message_t handle_robot_broadcast(robot_broadcast_message_t *broadcast) {
    sensor_reply_message_t reply = {0};
    
    // Find this sensor in registry (simplified - in real implementation, 
    // each sensor would know its own ID)
    for (uint32_t i = 0; i < sensor_count; i++) {
        if (sensor_registry[i].is_active) {
            // Simulate this sensor responding to broadcast
            reply.sensor_id = sensor_registry[i].sensor_id;
            reply.coord = sensor_registry[i].position;
            reply.sensor_status = sensor_registry[i].status;
            
            printf("Sensor_%u responding to Robot_%u broadcast\n", 
                   reply.sensor_id, broadcast->robot_id + 1);
            printf("Sensor_%u position: (%.2f, %.2f), Status: %s\n", 
                   reply.sensor_id, reply.coord.x, reply.coord.y,
                   reply.sensor_status == IDLE ? "IDLE" : "ACTIVE");
            
            break;  // Simplified - return first active sensor
        }
    }
    
    return reply;
}

// Update sensor status
void update_sensor_status(uint32_t sensor_id, sensor_status_t new_status) {
    for (uint32_t i = 0; i < sensor_count; i++) {
        if (sensor_registry[i].sensor_id == sensor_id) {
            sensor_status_t old_status = sensor_registry[i].status;
            sensor_registry[i].status = new_status;
            
            printf("Sensor_%u status updated: %s -> %s\n", 
                   sensor_id,
                   old_status == IDLE ? "IDLE" : "ACTIVE",
                   new_status == IDLE ? "IDLE" : "ACTIVE");
            break;
        }
    }
}

// Relocate sensor to new position
void relocate_sensor(uint32_t sensor_id, coordinate_t new_position) {
    for (uint32_t i = 0; i < sensor_count; i++) {
        if (sensor_registry[i].sensor_id == sensor_id) {
            coordinate_t old_position = sensor_registry[i].position;
            sensor_registry[i].position = new_position;
            
            printf("Sensor_%u relocated: (%.2f, %.2f) -> (%.2f, %.2f)\n", 
                   sensor_id, old_position.x, old_position.y,
                   new_position.x, new_position.y);
            break;
        }
    }
}

// Simulate sensor sensing within its range
void sense_environment(uint32_t sensor_id) {
    for (uint32_t i = 0; i < sensor_count; i++) {
        if (sensor_registry[i].sensor_id == sensor_id && 
            sensor_registry[i].status == ACTIVE) {
            
            printf("Sensor_%u sensing environment at (%.2f, %.2f)\n", 
                   sensor_id, sensor_registry[i].position.x, 
                   sensor_registry[i].position.y);
            
            // Simulate battery consumption
            sensor_registry[i].battery_level -= 0.1;
            if (sensor_registry[i].battery_level <= 0) {
                sensor_registry[i].is_active = 0;
                printf("Sensor_%u battery depleted - sensor inactive\n", sensor_id);
            }
            break;
        }
    }
}

// Check if sensor can detect an event at given position
int can_detect_event(uint32_t sensor_id, coordinate_t event_position) {
    for (uint32_t i = 0; i < sensor_count; i++) {
        if (sensor_registry[i].sensor_id == sensor_id && 
            sensor_registry[i].status == ACTIVE &&
            sensor_registry[i].is_active) {
            
            float distance = sqrt(pow(sensor_registry[i].position.x - event_position.x, 2) +
                                pow(sensor_registry[i].position.y - event_position.y, 2));
            
            if (distance <= sensor_perception_range) {
                printf("Sensor_%u detected event at (%.2f, %.2f)\n", 
                       sensor_id, event_position.x, event_position.y);
                return 1;
            }
            break;
        }
    }
    return 0;
}

// Get sensor information
sensor_db_record_t get_sensor_info(uint32_t sensor_id) {
    sensor_db_record_t info = {0};
    
    for (uint32_t i = 0; i < sensor_count; i++) {
        if (sensor_registry[i].sensor_id == sensor_id) {
            info.sensor_id = sensor_registry[i].sensor_id;
            info.coord = sensor_registry[i].position;
            info.sensor_status = sensor_registry[i].status;
            break;
        }
    }
    
    return info;
}

// Display all sensor states
void display_sensor_states() {
    printf("\n=== Sensor Node States ===\n");
    printf("ID\tPosition\t\tStatus\tBattery\tActive\n");
    
    for (uint32_t i = 0; i < sensor_count; i++) {
        printf("%u\t(%.2f, %.2f)\t%s\t%.1f%%\t%s\n", 
               sensor_registry[i].sensor_id,
               sensor_registry[i].position.x,
               sensor_registry[i].position.y,
               sensor_registry[i].status == IDLE ? "IDLE" : "ACTIVE",
               sensor_registry[i].battery_level,
               sensor_registry[i].is_active ? "YES" : "NO");
    }
}

// Simulate random sensor deployment in target area
void deploy_random_sensors(uint32_t num_sensors, float area_size) {
    printf("Deploying %u sensors randomly in %.2fx%.2f area\n", 
           num_sensors, area_size, area_size);
    
    srand(42);  // Fixed seed for reproducible results
    
    for (uint32_t i = 0; i < num_sensors && sensor_count < 1000; i++) {
        coordinate_t pos;
        pos.x = ((float)rand() / RAND_MAX) * area_size;
        pos.y = ((float)rand() / RAND_MAX) * area_size;
        
        initialize_sensor_node(i + 1, pos);
    }
}

// Calculate coverage area of all active sensors
float calculate_total_coverage_area() {
    float total_area = 0.0;
    float sensor_coverage_area = M_PI * sensor_perception_range * sensor_perception_range;
    
    for (uint32_t i = 0; i < sensor_count; i++) {
        if (sensor_registry[i].status == ACTIVE && sensor_registry[i].is_active) {
            total_area += sensor_coverage_area;
        }
    }
    
    printf("Total coverage area: %.2f square units\n", total_area);
    return total_area;
}

// Simulate sensor network operation
void simulate_sensor_network() {
    printf("\n=== Simulating Sensor Network Operation ===\n");
    
    // Simulate some sensing operations
    for (uint32_t i = 0; i < sensor_count && i < 5; i++) {
        if (sensor_registry[i].status == ACTIVE) {
            sense_environment(sensor_registry[i].sensor_id);
        }
    }
    
    // Simulate event detection
    coordinate_t event_pos = {100.0, 150.0};
    printf("\nEvent occurred at (%.2f, %.2f)\n", event_pos.x, event_pos.y);
    
    int detected = 0;
    for (uint32_t i = 0; i < sensor_count; i++) {
        if (can_detect_event(sensor_registry[i].sensor_id, event_pos)) {
            detected = 1;
        }
    }
    
    if (!detected) {
        printf("No sensors detected the event - coverage gap identified\n");
    }
}

// Main function for testing sensor node functionality
int main() {
    printf("=== WSN Deployment Sensor Nodes ===\n");
    
    // Deploy random sensors
    deploy_random_sensors(20, target_area_size);
    
    // Display initial sensor states
    display_sensor_states();
    
    // Calculate initial coverage
    calculate_total_coverage_area();
    
    // Simulate network operation
    simulate_sensor_network();
    
    // Simulate robot interaction
    printf("\n=== Simulating Robot-Sensor Interaction ===\n");
    robot_broadcast_message_t broadcast = {ROBOT_1};
    
    for (uint32_t i = 0; i < 3 && i < sensor_count; i++) {
        sensor_reply_message_t reply = handle_robot_broadcast(&broadcast);
        if (reply.sensor_id > 0) {
            // Simulate robot collecting/relocating sensor
            update_sensor_status(reply.sensor_id, ACTIVE);
            coordinate_t new_pos = {200.0 + i * 10, 200.0 + i * 10};
            relocate_sensor(reply.sensor_id, new_pos);
        }
    }
    
    // Display final sensor states
    printf("\n=== Final Sensor States ===\n");
    display_sensor_states();
    
    return 0;
}
