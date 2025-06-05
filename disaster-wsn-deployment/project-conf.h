#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/* Enable RPL routing */
#define UIP_CONF_ROUTER 1

/* Set IPv6 address buffer size */
#define NBR_TABLE_CONF_MAX_NEIGHBORS 30
#define UIP_CONF_BUFFER_SIZE 1300

/* Enable UDP */
#define UIP_CONF_UDP 1

/* Logging configuration */
#define LOG_CONF_LEVEL_DEFAULT LOG_LEVEL_INFO

/* Network configuration */
#define NETSTACK_CONF_RDC nullrdc_driver
#define NETSTACK_CONF_MAC csma_driver

/* Timer configuration for periodic operations */
#define CLOCK_SECOND 128

/* Application specific defines */
#define MAX_SENSORS_PER_ROBOT 15
#define INITIAL_STOCK_RS 10
#define MAX_GRIDS_PER_LA 100
#define MAX_LOCATION_AREAS 50

/* UDP ports */
#define UDP_SERVER_PORT 5678
#define UDP_CLIENT_PORT 8765

#endif /* PROJECT_CONF_H_ */
