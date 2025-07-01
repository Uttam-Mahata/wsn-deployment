#ifndef PROJECT_CONF_H
#define PROJECT_CONF_H

/* Enable logging */
#define LOG_CONF_LEVEL_MAIN LOG_LEVEL_INFO

/* Network configuration */
#define NETSTACK_CONF_WITH_IPV6 1
#define UIP_CONF_ROUTER 1
#define RPL_CONF_SUPPORTED 1

/* UDP Configuration */
#define UIP_CONF_UDP 1
#define UIP_CONF_UDP_CHECKSUMS 1

/* Logging Configuration for Debugging */
#define LOG_CONF_LEVEL_IPV6 LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_RPL LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_6LOWPAN LOG_LEVEL_WARN

/* Buffer sizes for improved communication */
#define UIP_CONF_BUFFER_SIZE 1280
#define UIP_CONF_RECEIVE_WINDOW 60

/* NBR table size for multiple robots */
#define NBR_TABLE_CONF_MAX_NEIGHBORS 10

/* Energy monitoring */
#define ENERGEST_CONF_ON 1

/* WSN Deployment specific configurations */
#define WSN_DEPLOYMENT_CONF_MAX_SENSORS 20
#define WSN_DEPLOYMENT_CONF_MAX_ROBOTS 2
#define WSN_DEPLOYMENT_CONF_MAX_LOCATION_AREAS 10
#define WSN_DEPLOYMENT_CONF_MAX_GRIDS_PER_LA 16
#define WSN_DEPLOYMENT_CONF_ROBOT_CAPACITY 15
#define WSN_DEPLOYMENT_CONF_INITIAL_STOCK 10

/* Network parameters */
#define WSN_DEPLOYMENT_CONF_SENSOR_RANGE 50    /* meters */
#define WSN_DEPLOYMENT_CONF_COMM_RANGE 100     /* meters */
#define WSN_DEPLOYMENT_CONF_ROBOT_PERCEPTION_RANGE 200 /* meters */

/* Node types */
#define WSN_NODE_TYPE_BASE_STATION 1
#define WSN_NODE_TYPE_MOBILE_ROBOT 2
#define WSN_NODE_TYPE_SENSOR 3

/* Communication reliability settings */
#define UIP_CONF_TCP 0
#define UIP_CONF_MAX_ROUTES 10
#define UIP_CONF_MAX_CONNECTIONS 4

/* Radio configuration for better range */
#define RF_CORE_CONF_CHANNEL 26
#define RADIO_CONF_CCA_THRESHOLD -95

#endif /* PROJECT_CONF_H */
