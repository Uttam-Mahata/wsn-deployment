#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"
#include "sys/node-id.h" // For node_id
#include <stdio.h>
#include <math.h>   // For sqrt, pow (for distance), ceil, log2
#include <string.h> // For memcpy, memset

// --- Global Defines and Constants ---
// Node Types - ONLY ONE OF THESE SHOULD BE UNCOMMENTED OR DEFINED IN Makefile.cooja
// #define NODE_TYPE NODE_TYPE_BS
// #define NODE_TYPE NODE_TYPE_ROBOT
// #define NODE_TYPE NODE_TYPE_SENSOR

#define NODE_TYPE_BS 1
#define NODE_TYPE_ROBOT 2
#define NODE_TYPE_SENSOR 3

// Node IDs - Assuming a specific assignment for simplicity in Cooja
#define BS_NODE_ID 1
#define ROBOT_NODE_ID_START 2
#define NUM_ROBOTS 2 // Robot IDs will be ROBOT_NODE_ID_START and ROBOT_NODE_ID_START + 1
#define SENSOR_NODE_ID_START (ROBOT_NODE_ID_START + NUM_ROBOTS)
#define MAX_SENSORS 20 // Total sensor nodes in the simulation. Adjust this based on your .csc file.
#define MAX_TOTAL_NODES (BS_NODE_ID + NUM_ROBOTS + MAX_SENSORS) // For node_energy_stats array sizing

// Target Area Dimensions (arbitrary units for Cooja visualization and calculations)
#define TARGET_AREA_SIZE_X 1000
#define TARGET_AREA_SIZE_Y 1000

// Robot and Sensor Physical Parameters
#define ROBOT_PERCEPTION_RANGE 200 // R (Perception range of robot in units)
#define SENSOR_SENSING_RANGE 20    // Rs (Sensing range of a sensor in units)

// Robot Deployment Parameters
#define ROBOT_STOCK_CAPACITY 15    // Max sensors a robot can carry
#define ROBOT_INITIAL_STOCK 10     // Sensors assigned to each robot initially

// Calculated values based on dimensions and ranges
// NO_LA = floor(Size of target area / (Perception range of robot)^2)
// Assuming rectangular LAs, and perception range is a side length for simplicity
#define LA_WIDTH ROBOT_PERCEPTION_RANGE
#define LA_HEIGHT ROBOT_PERCEPTION_RANGE
#define NUM_LAs_X (TARGET_AREA_SIZE_X / LA_WIDTH)
#define NUM_LAs_Y (TARGET_AREA_SIZE_Y / LA_HEIGHT)
#define NO_LA (NUM_LAs_X * NUM_LAs_Y)

// NO_G = floor(Perception range of robot / Perception range of sensor)
// Assuming square grids within an LA, and sensing range is a side length for grids
#define GRID_WIDTH SENSOR_SENSING_RANGE
#define GRID_HEIGHT SENSOR_SENSING_RANGE
#define NUM_GRIDS_X_PER_LA (ROBOT_PERCEPTION_RANGE / GRID_WIDTH)
#define NUM_GRIDS_Y_PER_LA (ROBOT_PERCEPTION_RANGE / GRID_HEIGHT)
#define NO_G_PER_DIM (ROBOT_PERCEPTION_RANGE / SENSOR_SENSING_RANGE) // NO_G in problem is likely per dimension
#define MAX_GRIDS_PER_LA (NO_G_PER_DIM * NO_G_PER_DIM) // Total grids in one LA

#define MAX_SENSORS_PER_LA MAX_SENSORS // Max sensors a robot can track in its DB for one LA

// Communication Channels/Ports for Rime
#define BROADCAST_CHANNEL 123
#define ROBOT_TO_BS_UNICAST_PORT 3000
#define SENSOR_TO_ROBOT_UNICAST_PORT 3001
#define ROBOT_TO_SENSOR_ACTIVATION_PORT 3002 // For robots to tell sensors to become active/idle

// Energy Model Constants (Assumed values - tune for realism)
// Power values in Watts (W)
#define P_BASELINE_SENSOR 0.0001
#define P_PROCESSING_SENSOR 0.00005
#define P_TRANSMIT_SENSOR 0.005
#define P_RECEIVE_SENSOR 0.004
#define P_IDLE_RADIO_SENSOR 0.00001

#define P_BASELINE_ROBOT 0.001
#define P_PROCESSING_ROBOT 0.0005
#define P_TRANSMIT_ROBOT 0.01
#define P_RECEIVE_ROBOT 0.008

#define P_BASELINE_BASE 0.005
#define P_PROCESSING_BASE 0.001
#define P_TRANSMIT_BASE 0.01
#define P_RECEIVE_BASE 0.008

// Energy coefficients (J/unit)
#define MU_SENSING 0.0005 // J/m^2 (energy per unit area for sensing)
#define TAU_MOBILITY 0.0005 // J/m (energy per unit distance for robot movement)

// Assumed data rate for energy calculation (bytes per second)
#define BYTES_PER_SECOND_RADIO 1000.0 // For calculating transmission/reception time

// --- Data Structures ---

// Coordinates structure
typedef struct {
    int x;
    int y;
} coord_t;

// LA_DB record (on BS)
typedef struct {
    int la_id;
    coord_t center_coord; // Center of the LA
    int num_covered_grids; // NO_Grid (number of covered grids in THIS LA)
} la_db_record_t;

// Robot_DB record (on BS)
typedef struct {
    int robot_id;
    int assigned_la_id;
} robot_db_record_t;

// Grid_DB record (on Robot)
typedef struct {
    int grid_id;
    coord_t center_coord;
    int grid_status; // 0: uncovered, 1: covered
} grid_db_record_t;

// Sensor_DB record (on Robot) - Tracks sensors detected by the robot
typedef struct {
    int sensor_node_id; // Cooja's node_id
    coord_t coord;       // Last known coordinate of the sensor
    int sensor_status;   // 0: idle, 1: active (as reported by sensor)
} robot_sensor_db_record_t;

// --- Message Formats ---

// Robot_pM (Robot to BS)
typedef struct {
    int robot_id;
    int covered_grids_in_la; // Cov_G
} robot_pm_msg_t;

// Mp (Robot broadcast for Topology Discovery)
typedef struct {
    int robot_id;
    coord_t robot_coord; // Robot's current position for perception
} mp_msg_t;

// Sensor_M (Sensor to Robot reply to Mp)
typedef struct {
    int sensor_id;
    coord_t sensor_coord;
    int sensor_status; // 0: idle, 1: active
} sensor_m_msg_t;

// Robot to Sensor Activation Message (for moving/activating sensors)
typedef struct {
    int sensor_id;
    int activate_status; // 0: idle, 1: activate
    coord_t new_coord; // If sensor is being "moved" (only in case 3 conceptually)
} robot_to_sensor_msg_t;

// --- Energy Tracking Structures ---
typedef struct {
    double total_baseline_energy;
    double total_sensing_energy;
    double total_processing_energy;
    double total_transmit_energy;
    double total_receive_energy;
    double total_mobility_energy; // Only for robot
    double total_idle_radio_energy; // Energy when radio is on but not tx/rx
} energy_stats_t;

// Global array to store energy stats for each node (indexed by node_id)
energy_stats_t node_energy_stats[MAX_TOTAL_NODES + 1];

// --- Global Variables and Data Structures (Shared and Node-Specific) ---

// BS related global variables (declared globally for simplicity in shared memory simulation)
#if defined(NODE_TYPE) && NODE_TYPE == NODE_TYPE_BS
la_db_record_t la_db[NO_LA];
robot_db_record_t robot_db[NUM_ROBOTS];
int total_covered_grids_global_bs = 0; // For Per_AC calculation on BS
int robots_finished_current_phase = 0; // Counter for BS to know when to proceed
#endif

// Robot related global variables
#if defined(NODE_TYPE) && NODE_TYPE == NODE_TYPE_ROBOT
grid_db_record_t robot_grid_db[MAX_GRIDS_PER_LA];
robot_sensor_db_record_t robot_sensor_db[MAX_SENSORS_PER_LA]; // Sensors *seen* by this robot
int robot_stock_rs;
int robot_current_la_id;
int robot_no_p; // Permissible moves
coord_t robot_current_pos; // Robot's current simulated position
double robot_distance_moved_total = 0.0; // Cumulative distance for mobility energy
#endif

// Sensor related global variables
#if defined(NODE_TYPE) && NODE_TYPE == NODE_TYPE_SENSOR
coord_t my_sensor_pos;
int is_sensor_active = 0; // 0: idle, 1: active (i.e., contributing to coverage)
#endif

// --- Cooja Processes ---
PROCESS(main_node_process, "Main Node Process");
AUTOSTART_PROCESSES(&main_node_process);

// --- Helper Functions ---

// Calculate Euclidean distance between two points
double calculate_distance(coord_t p1, coord_t p2) {
    return sqrt(pow((double)p1.x - p2.x, 2) + pow((double)p1.y - p2.y, 2));
}

// --- Energy Calculation Functions ---

// Note: Clock ticks are used for duration, then converted to seconds using CLOCK_SECOND.
void update_baseline_energy(int id, double power_W, clock_time_t duration_ticks) {
    node_energy_stats[id].total_baseline_energy += power_W * ((double)duration_ticks / CLOCK_SECOND);
}

void update_sensing_energy(int id, double sensing_range) {
    node_energy_stats[id].total_sensing_energy += MU_SENSING * sensing_range * sensing_range;
}

void update_processing_energy(int id, double power_W, clock_time_t duration_ticks) {
    node_energy_stats[id].total_processing_energy += power_W * ((double)duration_ticks / CLOCK_SECOND);
}

void update_transmit_energy(int id, double power_W, size_t msg_size_bytes) {
    double transmit_time_s = (double)msg_size_bytes / BYTES_PER_SECOND_RADIO;
    node_energy_stats[id].total_transmit_energy += power_W * transmit_time_s;
}

void update_receive_energy(int id, double power_W, size_t msg_size_bytes) {
    double receive_time_s = (double)msg_size_bytes / BYTES_PER_SECOND_RADIO;
    node_energy_stats[id].total_receive_energy += power_W * receive_time_s;
}

void update_idle_radio_energy(int id, double power_W, clock_time_t duration_ticks) {
    node_energy_stats[id].total_idle_radio_energy += power_W * ((double)duration_ticks / CLOCK_SECOND);
}

void update_mobility_energy(int id, double distance_units) {
    node_energy_stats[id].total_mobility_energy += TAU_MOBILITY * distance_units;
}

// --- Rime Callback Functions ---
// A single set of Rime connections are defined globally,
// and their callbacks are assigned based on NODE_TYPE.
static struct unicast_conn unicast_conn_general;
static struct broadcast_conn broadcast_conn_general;

#if defined(NODE_TYPE) && NODE_TYPE == NODE_TYPE_BS
// BS unicast receive callback (from robots)
static void unicast_recv_bs(struct unicast_conn *c, const rimeaddr_t *from) {
    robot_pm_msg_t msg;
    if (packetbuf_datalen() == sizeof(robot_pm_msg_t)) {
        memcpy(&msg, packetbuf_dataptr(), packetbuf_datalen());

        printf("BS (%d): Received Robot_pM from Robot %d (covered %d grids).\n", node_id, msg.robot_id, msg.covered_grids_in_la);
        update_receive_energy(node_id, P_RECEIVE_BASE, packetbuf_datalen());

        // Find assigned LA for this robot and update LA_DB
        int assigned_la_idx = -1;
        for (int i = 0; i < NUM_ROBOTS; i++) {
            if (robot_db[i].robot_id == msg.robot_id) {
                assigned_la_idx = robot_db[i].assigned_la_id;
                break;
            }
        }

        if (assigned_la_idx != -1 && la_db[assigned_la_idx].num_covered_grids == 0) { // Only update if LA was not yet counted
            la_db[assigned_la_idx].num_covered_grids = msg.covered_grids_in_la;
            total_covered_grids_global_bs += msg.covered_grids_in_la;
            printf("BS (%d): LA %d updated with %d covered grids. Global covered: %d.\n", node_id, assigned_la_idx, msg.covered_grids_in_la, total_covered_grids_global_bs);
            update_processing_energy(node_id, P_PROCESSING_BASE, CLOCK_SECOND / 10); // Small processing cost
        }

        robots_finished_current_phase++;
        if (robots_finished_current_phase == NUM_ROBOTS) {
            printf("BS (%d): All robots reported. Triggering next assignment phase.\n", node_id);
            process_post(&main_node_process, PROCESS_EVENT_CONTINUE, NULL); // Signal BS process to continue
        }
    } else {
        printf("BS (%d): Received malformed Robot_pM from %d.\n", node_id, from->u8[0]);
    }
}
static const struct unicast_callbacks unicast_callbacks_bs = {unicast_recv_bs};
#endif // NODE_TYPE_BS

#if defined(NODE_TYPE) && NODE_TYPE == NODE_TYPE_ROBOT
// Robot unicast receive callback (from sensors)
static void unicast_recv_robot(struct unicast_conn *c, const rimeaddr_t *from) {
    sensor_m_msg_t msg;
    if (packetbuf_datalen() == sizeof(sensor_m_msg_t)) {
        memcpy(&msg, packetbuf_dataptr(), packetbuf_datalen());

        // printf("Robot %d: Rcvd Sensor_M from S%d @(%d,%d), status %d.\n", node_id, msg.sensor_id, msg.sensor_coord.x, msg.sensor_coord.y, msg.sensor_status);
        update_receive_energy(node_id, P_RECEIVE_ROBOT, packetbuf_datalen());

        // Add/update sensor in robot's Sensor_DB
        int found = 0;
        for (int i = 0; i < MAX_SENSORS_PER_LA; i++) {
            if (robot_sensor_db[i].sensor_node_id == msg.sensor_id) {
                robot_sensor_db[i].coord = msg.sensor_coord;
                robot_sensor_db[i].sensor_status = msg.sensor_status;
                found = 1;
                break;
            }
        }
        if (!found) { // Add new sensor if space available
            for (int i = 0; i < MAX_SENSORS_PER_LA; i++) {
                if (robot_sensor_db[i].sensor_node_id == 0) { // Empty slot (assuming node_id 0 is invalid)
                    robot_sensor_db[i].sensor_node_id = msg.sensor_id;
                    robot_sensor_db[i].coord = msg.sensor_coord;
                    robot_sensor_db[i].sensor_status = msg.sensor_status;
                    break;
                }
            }
        }
        update_processing_energy(node_id, P_PROCESSING_ROBOT, CLOCK_SECOND / 20); // Small processing cost
    } else {
         printf("Robot %d: Received malformed Sensor_M from %d.\n", node_id, from->u8[0]);
    }
}

static const struct unicast_callbacks unicast_callbacks_robot = {unicast_recv_robot};

// Robot broadcast receive callback (not used for robot's own logic, but required by Rime)
static void broadcast_recv_robot(struct broadcast_conn *c, const rimeaddr_t *from) {
    // Robots generally ignore broadcasts from other robots for this simulation logic
}
static const struct broadcast_callbacks broadcast_callbacks_robot = {broadcast_recv_robot};
#endif // NODE_TYPE_ROBOT

#if defined(NODE_TYPE) && NODE_TYPE == NODE_TYPE_SENSOR
// Sensor broadcast receive callback (from robot Mp)
static void broadcast_recv_sensor(struct broadcast_conn *c, const rimeaddr_t *from) {
    mp_msg_t msg;
    if (packetbuf_datalen() == sizeof(mp_msg_t)) {
        memcpy(&msg, packetbuf_dataptr(), packetbuf_datalen());

        update_receive_energy(node_id, P_RECEIVE_SENSOR, packetbuf_datalen());

        // Check if robot is within perception range
        if (calculate_distance(my_sensor_pos, msg.robot_coord) <= ROBOT_PERCEPTION_RANGE) {
            // printf("S%d: Rcvd Mp from R%d. Robot @(%d,%d). My pos (%d,%d). In range.\n",
            //        node_id, msg.robot_id, msg.robot_coord.x, msg.robot_coord.y, my_sensor_pos.x, my_sensor_pos.y);

            // Reply with Sensor_M to the robot
            sensor_m_msg_t reply_msg;
            reply_msg.sensor_id = node_id;
            reply_msg.sensor_coord = my_sensor_pos;
            reply_msg.sensor_status = is_sensor_active; // 0 for idle, 1 for active

            rimeaddr_t robot_addr;
            robot_addr.u8[0] = msg.robot_id; // Sender of Mp is the robot
            robot_addr.u8[1] = 0; // Rime address second byte is usually 0 for single-byte IDs

            packetbuf_copyfrom(&reply_msg, sizeof(reply_msg));
            unicast_send(&unicast_conn_general, &robot_addr);
            update_transmit_energy(node_id, P_TRANSMIT_SENSOR, sizeof(reply_msg));
        }
    } else {
        printf("S%d: Rcvd malformed Mp from %d.\n", node_id, from->u8[0]);
    }
}
static const struct broadcast_callbacks broadcast_callbacks_sensor = {broadcast_recv_sensor};

// Sensor unicast receive callback (from robot for activation/deactivation/move)
static void unicast_recv_sensor_activation(struct unicast_conn *c, const rimeaddr_t *from) {
    robot_to_sensor_msg_t msg;
    if (packetbuf_datalen() == sizeof(robot_to_sensor_msg_t)) {
        memcpy(&msg, packetbuf_dataptr(), packetbuf_datalen());

        update_receive_energy(node_id, P_RECEIVE_SENSOR, packetbuf_datalen());

        if (msg.sensor_id == node_id) { // Message is for me
            is_sensor_active = msg.activate_status;
            // If the sensor is conceptually "moved" (Case 3), update its position
            if (is_sensor_active) { // Only move if activating it for coverage
                my_sensor_pos = msg.new_coord;
                printf("S%d: Activated and moved to (%d,%d).\n", node_id, my_sensor_pos.x, my_sensor_pos.y);
            } else {
                printf("S%d: Deactivated.\n", node_id);
            }
            update_processing_energy(node_id, P_PROCESSING_SENSOR, CLOCK_SECOND / 20);
        }
    } else {
        printf("S%d: Rcvd malformed activation msg from %d.\n", node_id, from->u8[0]);
    }
}
static const struct unicast_callbacks unicast_callbacks_sensor = {unicast_recv_sensor_activation};
#endif // NODE_TYPE_SENSOR


// --- Main Process Thread ---
PROCESS_THREAD(main_node_process, ev, data) {
    PROCESS_BEGIN();

    // Initialize all energy stats to zero for all possible node IDs
    memset(node_energy_stats, 0, sizeof(node_energy_stats));

#if defined(NODE_TYPE) && NODE_TYPE == NODE_TYPE_BS
    printf("BS (Node ID: %d): Starting...\n", node_id);
    // Open unicast for receiving from robots
    unicast_open(&unicast_conn_general, ROBOT_TO_BS_UNICAST_PORT, &unicast_callbacks_bs);

    // Initialize LA_DB for the entire target area
    for (int y = 0; y < NUM_LAs_Y; y++) {
        for (int x = 0; x < NUM_LAs_X; x++) {
            int i = y * NUM_LAs_X + x;
            la_db[i].la_id = i;
            la_db[i].center_coord.x = x * LA_WIDTH + LA_WIDTH / 2;
            la_db[i].center_coord.y = y * LA_HEIGHT + LA_HEIGHT / 2;
            la_db[i].num_covered_grids = 0; // Initially 0 covered grids
            printf("BS: LA %d at (%d,%d).\n", la_db[i].la_id, la_db[i].center_coord.x, la_db[i].center_coord.y);
        }
    }
    printf("BS: LA_DB initialized with %d LAs.\n", NO_LA);

    // Initial Robot_DB assignment as per the problem description
    robot_db[0].robot_id = ROBOT_NODE_ID_START;
    robot_db[0].assigned_la_id = 0; // Assign first LA to Robot 1
    printf("BS: Robot %d assigned to LA %d.\n", robot_db[0].robot_id, robot_db[0].assigned_la_id);

    if (NUM_ROBOTS > 1) {
        robot_db[1].robot_id = ROBOT_NODE_ID_START + 1;
        robot_db[1].assigned_la_id = NO_LA - 1; // Assign last LA to Robot 2
        printf("BS: Robot %d assigned to LA %d.\n", robot_db[1].robot_id, robot_db[1].assigned_la_id);
    }

    static struct etimer bs_timer;
    etimer_set(&bs_timer, CLOCK_SECOND * 5); // Give robots time to initialize and read their assignments

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&bs_timer));

    // Global Phase loop - BS coordinates robot deployments
    while (1) {
        update_baseline_energy(node_id, P_BASELINE_BASE, CLOCK_SECOND); // Baseline energy per global phase cycle
        robots_finished_current_phase = 0; // Reset counter for this iteration

        // Wait for all robots to report back after completing their local phase
        printf("BS (%d): Waiting for all %d robots to complete their local phase...\n", node_id, NUM_ROBOTS);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && robots_finished_current_phase == NUM_ROBOTS);

        int assigned_new_la_count = 0;
        for (int p_idx = 0; p_idx < NUM_ROBOTS; p_idx++) {
            int robot_id = robot_db[p_idx].robot_id;
            int found_new_la = 0;
            // Search for an uncovered LA (num_covered_grids == 0)
            for (int i = 0; i < NO_LA; i++) {
                if (la_db[i].num_covered_grids == 0) {
                    robot_db[p_idx].assigned_la_id = la_db[i].la_id;
                    found_new_la = 1;
                    assigned_new_la_count++;
                    printf("BS (%d): Re-assigned Robot %d to LA %d.\n", node_id, robot_id, la_db[i].la_id);
                    update_processing_energy(node_id, P_PROCESSING_BASE, CLOCK_SECOND / 10);
                    break; // Assign this LA and move to next robot
                }
            }
            if (!found_new_la) {
                printf("BS (%d): No more uncovered LAs to assign to Robot %d.\n", node_id, robot_id);
            }
        }

        if (assigned_new_la_count == 0) { // All LAs are covered or no new ones found for any robot
            printf("\nBS (%d): All LAs covered or no new assignments possible. Simulation complete.\n", node_id);
            // Calculate and print final area coverage
            double per_ac = (double)total_covered_grids_global_bs / (NO_LA * MAX_GRIDS_PER_LA) * 100.0;
            printf("BS (%d): Final Percentage of Area Coverage (Per_AC): %.2f%%\n", node_id, per_ac);
            
            // Print total energy consumption for all nodes
            printf("\n--- TOTAL ENERGY CONSUMPTION REPORT ---\n");
            double total_sys_energy = 0;
            for(int i = 1; i <= MAX_TOTAL_NODES; i++) { // Iterate through all possible node IDs
                double node_total_energy =
                    node_energy_stats[i].total_baseline_energy +
                    node_energy_stats[i].total_sensing_energy +
                    node_energy_stats[i].total_processing_energy +
                    node_energy_stats[i].total_transmit_energy +
                    node_energy_stats[i].total_receive_energy +
                    node_energy_stats[i].total_mobility_energy +
                    node_energy_stats[i].total_idle_radio_energy;

                total_sys_energy += node_total_energy;

                if (i == BS_NODE_ID) printf("  BS (Node %d) Energy: %.4f J\n", i, node_total_energy);
                else if (i >= ROBOT_NODE_ID_START && i < SENSOR_NODE_ID_START) printf("  Robot (Node %d) Energy: %.4f J\n", i, node_total_energy);
                else if (i >= SENSOR_NODE_ID_START && i <= MAX_TOTAL_NODES) printf("  Sensor (Node %d) Energy: %.4f J\n", i, node_total_energy);
            }
            printf("  TOTAL SYSTEM ENERGY CONSUMPTION: %.4f J\n", total_sys_energy);
            printf("--- END OF REPORT ---\n");
            break; // End simulation for BS
        }
    }

#elif defined(NODE_TYPE) && NODE_TYPE == NODE_TYPE_ROBOT
    printf("Robot (Node ID: %d): Starting...\n", node_id);
    // Open Rime connections for broadcast (Mp) and unicast (Sensor_M from sensors, Robot_pM to BS)
    broadcast_open(&broadcast_conn_general, BROADCAST_CHANNEL, &broadcast_callbacks_robot);
    unicast_open(&unicast_conn_general, SENSOR_TO_ROBOT_UNICAST_PORT, &unicast_callbacks_robot);

    robot_stock_rs = ROBOT_INITIAL_STOCK;
    robot_distance_moved_total = 0.0; // Reset cumulative distance

    static struct etimer robot_timer;
    static coord_t prev_pos; // To track robot's previous position for mobility energy

    // Initialize robot's current position to some default (e.g., center of target area)
    robot_current_pos.x = TARGET_AREA_SIZE_X / 2;
    robot_current_pos.y = TARGET_AREA_SIZE_Y / 2;
    prev_pos = robot_current_pos; // No initial movement cost

    while (1) {
        update_baseline_energy(node_id, P_BASELINE_ROBOT, CLOCK_SECOND);

        // Fetch current LA assignment from "BS_DB" (simulated shared memory / lookup)
        robot_current_la_id = -1;
        for (int i = 0; i < NUM_ROBOTS; i++) {
            if (robot_db[i].robot_id == node_id) {
                robot_current_la_id = robot_db[i].assigned_la_id;
                break;
            }
        }

        // If no LA is assigned or the current LA is already processed, wait
        if (robot_current_la_id == -1 || la_db[robot_current_la_id].num_covered_grids != 0) {
            printf("Robot %d: No new LA assignment or current LA %d already processed. Waiting 5s.\n", node_id, robot_current_la_id);
            etimer_set(&robot_timer, CLOCK_SECOND * 5);
            PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&robot_timer));
            continue; // Re-check assignment
        }

        printf("Robot %d: Starting Local Phase in LA %d.\n", node_id, robot_current_la_id);
        update_processing_energy(node_id, P_PROCESSING_ROBOT, CLOCK_SECOND / 2); // For overall local phase setup

        // Divide LA into grids and insert records into Grid_DB
        coord_t la_origin; // Bottom-left corner of LA
        la_origin.x = la_db[robot_current_la_id].center_coord.x - (LA_WIDTH / 2);
        la_origin.y = la_db[robot_current_la_id].center_coord.y - (LA_HEIGHT / 2);

        int grid_count_in_la = 0;
        for (int gy = 0; gy < NUM_GRIDS_Y_PER_LA; gy++) {
            for (int gx = 0; gx < NUM_GRIDS_X_PER_LA; gx++) {
                robot_grid_db[grid_count_in_la].grid_id = grid_count_in_la;
                robot_grid_db[grid_count_in_la].center_coord.x = la_origin.x + (gx * GRID_WIDTH) + (GRID_WIDTH / 2);
                robot_grid_db[grid_count_in_la].center_coord.y = la_origin.y + (gy * GRID_HEIGHT) + (GRID_HEIGHT / 2);
                robot_grid_db[grid_count_in_la].grid_status = 0; // Initially uncovered
                grid_count_in_la++;
            }
        }
        printf("Robot %d: LA %d divided into %d grids.\n", node_id, robot_current_la_id, grid_count_in_la);

        // Initialize Sensor_DB (sensors seen by this robot) to empty
        memset(robot_sensor_db, 0, sizeof(robot_sensor_db));

        // --- Topology Discovery Phase ---
        printf("Robot %d: Starting Topology Discovery Phase.\n", node_id);
        prev_pos = robot_current_pos;
        robot_current_pos = la_db[robot_current_la_id].center_coord; // Move to center of LA
        update_mobility_energy(node_id, calculate_distance(prev_pos, robot_current_pos));

        // Broadcast message (Mp) for sensors to reply
        mp_msg_t mp_msg;
        mp_msg.robot_id = node_id;
        mp_msg.robot_coord = robot_current_pos;
        packetbuf_copyfrom(&mp_msg, sizeof(mp_msg));
        broadcast_send(&broadcast_conn_general);
        printf("Robot %d: Broadcasted Mp from (%d,%d).\n", node_id, robot_current_pos.x, robot_current_pos.y);
        update_transmit_energy(node_id, P_TRANSMIT_ROBOT, sizeof(mp_msg));

        // Wait for sensor replies to populate robot_sensor_db
        etimer_set(&robot_timer, CLOCK_SECOND * 2);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&robot_timer));
        printf("Robot %d: Topology Discovery Phase finished. Sensors discovered.\n", node_id);

        // --- Dispersion Phase ---
        printf("Robot %d: Starting Dispersion Phase.\n", node_id);
        robot_no_p = MAX_GRIDS_PER_LA; // Reset permissible moves for this LA
        int num_covered_grids_in_this_la = 0;

        while (robot_no_p > 0) {
            update_baseline_energy(node_id, P_BASELINE_ROBOT, CLOCK_SECOND / 10); // Small baseline per grid operation

            int target_grid_idx = -1;
            double min_dist_to_uncovered = -1.0;
            // Find the nearest uncovered grid
            for (int i = 0; i < MAX_GRIDS_PER_LA; i++) {
                if (robot_grid_db[i].grid_status == 0) { // Check if grid is uncovered
                    double dist = calculate_distance(robot_current_pos, robot_grid_db[i].center_coord);
                    if (target_grid_idx == -1 || dist < min_dist_to_uncovered) {
                        min_dist_to_uncovered = dist;
                        target_grid_idx = i;
                    }
                }
            }

            if (target_grid_idx == -1) {
                printf("Robot %d: All grids in LA %d seem covered or no more uncovered grids to visit. Breaking dispersion.\n", node_id, robot_current_la_id);
                break; // All grids in the current LA are covered
            }

            prev_pos = robot_current_pos;
            robot_current_pos = robot_grid_db[target_grid_idx].center_coord; // Move robot to grid center
            update_mobility_energy(node_id, calculate_distance(prev_pos, robot_current_pos));
            printf("Robot %d: Moving to grid %d at (%d,%d). Dist %.1f. NO_P: %d.\n", node_id, target_grid_idx, robot_current_pos.x, robot_current_pos.y, calculate_distance(prev_pos, robot_current_pos), robot_no_p);

            // Identify sensors physically present near this grid's center (within Rs/2)
            int sensors_physically_in_grid_count = 0;
            int sensors_to_collect_node_ids[MAX_SENSORS_PER_LA];
            int collected_list_idx = 0;

            for (int i = 0; i < MAX_SENSORS_PER_LA; i++) {
                if (robot_sensor_db[i].sensor_node_id != 0) { // If this slot in robot's DB holds a sensor
                    if (calculate_distance(robot_sensor_db[i].coord, robot_current_pos) <= SENSOR_SENSING_RANGE / 2.0) {
                        sensors_physically_in_grid_count++;
                        sensors_to_collect_node_ids[collected_list_idx++] = robot_sensor_db[i].sensor_node_id;
                    }
                }
            }
            update_processing_energy(node_id, P_PROCESSING_ROBOT, CLOCK_SECOND / 20); // Cost for checking sensors

            int grid_became_covered = 0;
            robot_to_sensor_msg_t sensor_control_msg; // Message to activate/deactivate actual sensors
            rimeaddr_t target_sensor_addr;

            if (robot_stock_rs > 0 && sensors_physically_in_grid_count > 0) {
                // Case 1: Robot has sensors and grid has sensors
                robot_stock_rs--; // Place one sensor (from stock)
                grid_became_covered = 1;
                printf("Robot %d, Grid %d: Case 1. Placed new sensor from stock. Stock: %d.\n", node_id, target_grid_idx, robot_stock_rs);

                // Collect extra sensors from the grid until stock capacity is reached
                int num_collected_this_turn = 0;
                for (int i = 0; i < collected_list_idx; i++) {
                    if (robot_stock_rs < ROBOT_STOCK_CAPACITY) {
                        // Find the sensor in robot_sensor_db and remove it (conceptually collected)
                        for (int j = 0; j < MAX_SENSORS_PER_LA; j++) {
                            if (robot_sensor_db[j].sensor_node_id == sensors_to_collect_node_ids[i]) {
                                robot_sensor_db[j].sensor_node_id = 0; // Mark slot as empty
                                robot_stock_rs++;
                                num_collected_this_turn++;
                                // Tell the actual sensor node it is idle/collected
                                sensor_control_msg.sensor_id = sensors_to_collect_node_ids[i];
                                sensor_control_msg.activate_status = 0; // Set to Idle
                                sensor_control_msg.new_coord = robot_sensor_db[j].coord; // Keep its existing coordinate
                                target_sensor_addr.u8[0] = sensor_control_msg.sensor_id; target_sensor_addr.u8[1] = 0;
                                packetbuf_copyfrom(&sensor_control_msg, sizeof(sensor_control_msg));
                                unicast_send(&unicast_conn_general, &target_sensor_addr);
                                update_transmit_energy(node_id, P_TRANSMIT_ROBOT, sizeof(sensor_control_msg));
                                update_processing_energy(node_id, P_PROCESSING_ROBOT, CLOCK_SECOND / 50); // For collecting
                                break;
                            }
                        }
                    } else {
                        break; // Stock capacity reached
                    }
                }
                printf("Robot %d: Collected %d extra sensors. Stock now %d.\n", node_id, num_collected_this_turn, robot_stock_rs);

            } else if (robot_stock_rs > 0 && sensors_physically_in_grid_count == 0) {
                // Case 2: Robot has sensors but grid has no sensors
                robot_stock_rs--; // Place one sensor from stock
                grid_became_covered = 1;
                printf("Robot %d, Grid %d: Case 2. Placed new sensor. Stock: %d.\n", node_id, target_grid_idx, robot_stock_rs);
            } else if (robot_stock_rs == 0 && sensors_physically_in_grid_count > 0) {
                // Case 3: Robot has no sensors in stock but grid has sensors
                grid_became_covered = 1; // Grid is covered by an existing sensor

                // Find a sensor to "move" to center (conceptually activating it for coverage)
                // We'll pick the first sensor found in `sensors_to_collect_node_ids`
                int sensor_to_activate_id = sensors_to_collect_node_ids[0];
                
                // Send activation message to this sensor, effectively moving it to grid center
                sensor_control_msg.sensor_id = sensor_to_activate_id;
                sensor_control_msg.activate_status = 1; // Set to Active
                sensor_control_msg.new_coord = robot_current_pos; // Tell it its new "effective" position
                target_sensor_addr.u8[0] = sensor_control_msg.sensor_id; target_sensor_addr.u8[1] = 0;
                packetbuf_copyfrom(&sensor_control_msg, sizeof(sensor_control_msg));
                unicast_send(&unicast_conn_general, &target_sensor_addr);
                update_transmit_energy(node_id, P_TRANSMIT_ROBOT, sizeof(sensor_control_msg));
                update_processing_energy(node_id, P_PROCESSING_ROBOT, CLOCK_SECOND / 20); // For identifying/moving

                printf("Robot %d, Grid %d: Case 3. Moved sensor %d to cover grid. Stock: %d.\n", node_id, target_grid_idx, sensor_to_activate_id, robot_stock_rs);

                // Collect extra sensors (excluding the one "moved" to cover)
                int num_collected_this_turn = 0;
                for (int i = 0; i < collected_list_idx; i++) {
                    if (sensors_to_collect_node_ids[i] == sensor_to_activate_id) continue; // Don't collect the one used for coverage

                    if (robot_stock_rs < ROBOT_STOCK_CAPACITY) {
                        for (int j = 0; j < MAX_SENSORS_PER_LA; j++) {
                            if (robot_sensor_db[j].sensor_node_id == sensors_to_collect_node_ids[i]) {
                                robot_sensor_db[j].sensor_node_id = 0; // Mark slot as empty
                                robot_stock_rs++;
                                num_collected_this_turn++;
                                // Tell the actual sensor node it is idle/collected
                                sensor_control_msg.sensor_id = sensors_to_collect_node_ids[i];
                                sensor_control_msg.activate_status = 0; // Set to Idle
                                sensor_control_msg.new_coord = robot_sensor_db[j].coord; // Keep its existing coordinate
                                target_sensor_addr.u8[0] = sensor_control_msg.sensor_id; target_sensor_addr.u8[1] = 0;
                                packetbuf_copyfrom(&sensor_control_msg, sizeof(sensor_control_msg));
                                unicast_send(&unicast_conn_general, &target_sensor_addr);
                                update_transmit_energy(node_id, P_TRANSMIT_ROBOT, sizeof(sensor_control_msg));
                                update_processing_energy(node_id, P_PROCESSING_ROBOT, CLOCK_SECOND / 50); // For collecting
                                break;
                            }
                        }
                    } else {
                        break; // Stock capacity reached
                    }
                }
                printf("Robot %d: Collected %d extra sensors. Stock now %d.\n", node_id, num_collected_this_turn, robot_stock_rs);

            } else {
                // Case 4: Neither robot nor grid has sensors
                grid_became_covered = 0;
                printf("Robot %d, Grid %d: Case 4. Grid remains uncovered. Stock: %d.\n", node_id, target_grid_idx, robot_stock_rs);
            }

            if (grid_became_covered && robot_grid_db[target_grid_idx].grid_status == 0) {
                robot_grid_db[target_grid_idx].grid_status = 1; // Mark as covered
                num_covered_grids_in_this_la++;
                update_processing_energy(node_id, P_PROCESSING_ROBOT, CLOCK_SECOND / 50);
            }

            robot_no_p--; // Decrement permissible moves
            // Simulate processing time before next move
            etimer_set(&robot_timer, CLOCK_SECOND / 2);
            PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&robot_timer));
        }

        // End of local phase, send message to BS
        printf("Robot %d: Dispersion Phase completed. Covered %d grids in LA %d.\n", node_id, num_covered_grids_in_this_la, robot_current_la_id);
        robot_pm_msg_t robot_pm;
        robot_pm.robot_id = node_id;
        robot_pm.covered_grids_in_la = num_covered_grids_in_this_la;

        rimeaddr_t bs_addr;
        bs_addr.u8[0] = BS_NODE_ID;
        bs_addr.u8[1] = 0;

        packetbuf_copyfrom(&robot_pm, sizeof(robot_pm));
        unicast_send(&unicast_conn_general, &bs_addr);
        printf("Robot %d: Sent Robot_pM to BS.\n", node_id);
        update_transmit_energy(node_id, P_TRANSMIT_ROBOT, sizeof(robot_pm));

        // Reset for next local phase (if any)
        robot_no_p = MAX_GRIDS_PER_LA; // Reset permissible moves
        robot_stock_rs = ROBOT_INITIAL_STOCK; // Reset stock for next LA

        etimer_set(&robot_timer, CLOCK_SECOND * 5); // Wait before attempting next LA
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&robot_timer));
    }

#elif defined(NODE_TYPE) && NODE_TYPE == NODE_TYPE_SENSOR
    printf("Sensor (Node ID: %d): Starting...\n", node_id);
    // Open Rime connections for broadcast (from robots) and unicast (to robots and for activation)
    broadcast_open(&broadcast_conn_general, BROADCAST_CHANNEL, &broadcast_callbacks_sensor);
    unicast_open(&unicast_conn_general, ROBOT_TO_SENSOR_ACTIVATION_PORT, &unicast_callbacks_sensor); // For activation messages

    // Set initial random position for the sensor node.
    // This position will be used for distance calculations. Cooja's visual position is separate.
    random_init(node_id); // Seed random with node_id for varied initial positions
    my_sensor_pos.x = random_rand() % TARGET_AREA_SIZE_X;
    my_sensor_pos.y = random_rand() % TARGET_AREA_SIZE_Y;
    printf("Sensor %d: Initial position is (%d,%d).\n", node_id, my_sensor_pos.x, my_sensor_pos.y);
    is_sensor_active = 0; // All sensors initially idle as per text

    static struct etimer sensor_timer;

    while (1) {
        // Sensor nodes primarily listen for Mp messages and reply.
        // They also consume baseline energy and sensing energy if active.
        update_baseline_energy(node_id, P_BASELINE_SENSOR, CLOCK_SECOND);

        if (is_sensor_active) {
            update_sensing_energy(node_id, SENSOR_SENSING_RANGE);
            update_processing_energy(node_id, P_PROCESSING_SENSOR, CLOCK_SECOND / 10);
            // printf("Sensor %d: Active, sensing.\n", node_id); // Commented to reduce output
        } else {
            update_idle_radio_energy(node_id, P_IDLE_RADIO_SENSOR, CLOCK_SECOND / 2); // Radio is always on for listening
            // printf("Sensor %d: Idle, listening.\n", node_id); // Commented to reduce output
        }

        etimer_set(&sensor_timer, CLOCK_SECOND * 1); // Periodically consume energy and listen
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sensor_timer));
    }

#else
    printf("Unknown Node Type (Node ID: %d). Please define NODE_TYPE correctly in Makefile.cooja.\n", node_id);
    // This node will just sleep if its type is not defined
    static struct etimer et;
    etimer_set(&et, CLOCK_SECOND * 1000); // Sleep for a long time
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
#endif // NODE_TYPE

    PROCESS_END();
}