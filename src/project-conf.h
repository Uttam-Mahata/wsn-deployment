/*
 * WSN Deployment Project Configuration for Contiki-NG
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/* Network Configuration */
#define UIP_CONF_MAX_CONNECTIONS 10
#define UIP_CONF_MAX_LISTENPORTS 10
#define UIP_CONF_UDP_CONNS 10

/* IPv6 Configuration */
#define NETSTACK_CONF_WITH_IPV6 1
#define UIP_CONF_IPV6_QUEUE_PKT 1
#define UIP_CONF_IPV6_CHECKS 1
#define UIP_CONF_IPV6_REASSEMBLY 0
#define UIP_CONF_ND6_SEND_RA 0
#define UIP_CONF_ND6_SEND_NS 1
#define UIP_CONF_ND6_SEND_NA 1

/* Routing Configuration */
#define UIP_CONF_ROUTER 1
#define NETSTACK_CONF_WITH_IPV6 1

/* Radio and MAC Configuration */
#define NETSTACK_CONF_RADIO NETSTACK_RADIO
#define NETSTACK_CONF_MAC csma_driver
#define NETSTACK_CONF_RDC nullrdc_driver
#define NETSTACK_CONF_FRAMER framer_802154

/* Logging Configuration */
#define LOG_CONF_LEVEL_MAIN LOG_LEVEL_INFO
#define LOG_CONF_LEVEL_IPV6 LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_RPL LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_6LOWPAN LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_MAC LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_FRAMER LOG_LEVEL_WARN

/* WSN Deployment Specific Configuration */
#define WSN_TARGET_AREA_SIZE 1000.0f
#define WSN_ROBOT_PERCEPTION_RANGE 100.0f
#define WSN_SENSOR_PERCEPTION_RANGE 50.0f
#define WSN_MAX_SENSORS 100
#define WSN_MAX_ROBOTS 2

/* Memory Configuration */
#define UIP_CONF_BUFFER_SIZE 1280
#define QUEUEBUF_CONF_NUM 16

/* Timer Configuration */
#define CLOCK_CONF_SECOND 128

#endif /* PROJECT_CONF_H_ */
