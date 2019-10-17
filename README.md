nRF52 sample clients for nRF91 newtork service
==============================================
This repository contains nRF52 sample projects as nrf91 nrf9160/network_service_proxy clients.

More about nrf91-network project, please refer to below link first
https://github.com/NordicPlayground/fw-nrfconnect-nrf/tree/master/samples/nrf9160/network_service_proxy

Environments
------------
To build nRF52 projects:
* DK: nRF52840/PCA10056 or nRF52832/PCA10040
* nRF52 SDK: SDK15.3.0
* nRF52 Toolchain: Keil

Instructions
------------
Please clone this repository into <local_path>\nRF5_SDK_15.3.0\examples\.

Below samples automatically establish the LTE connection on start-up, using the Modem Control service. After that, it tries to fetch the current GMT time from public NTP server.

* MQTT client
  * Client project: \nRF5_SDK_15.3.0\examples\nrf5-nrf91-network-clients\mqtt_client
  * nRF91 Configuration: CONFIG_MQTT_LIB, CONFIG_THIN_MQTT_SERVICE, CONFIG_THIN_SNTP_SERVICE

Works with MQTT service. The same topic for publishing and subscribing is used to test message sending and receiving on nRF52 side. It is also possible to change QoS level by updating the code. The buttons on DK are defined as below: 
  Button#1: connect to the Broker server
  Button#2: disconnect from the Broker server
  Button#3: subscribe/unsubscribe a topic from the Broker
  Button#4: publish to the Broker

* MQTT client and GPS client
  * Client project: \nRF5_SDK_15.3.0\examples\nrf5-nrf91-network-clients\mqtt_client_gps
  * nRF91 Configuration: CONFIG_MQTT_LIB, CONFIG_THIN_MQTT_SERVICE, CONFIG_THIN_GPS_SERVICE

Works with MQTT and GPS services.  The buttons on DK are defined as below: 
  Button#1: connect to the Broker server
  Button#2: disconnect from the Broker server
  Button#3: enable/disable GPS position acquisition
  Button#4: Get GPS enabled/disabled and active/idle status

* LwM2M client
  * Client project: \nRF5_SDK_15.3.0\examples\nrf5-nrf91-network-clients\lwm2m_client
  * nRF91 Configuration: CONFIG_LWM2M, CONFIG_THIN_LWM2M_SERVICE, CONFIG_THIN_SNTP_SERVICE

Works with LwM2M service. The buttons on DK are defined as below:
  Button#1: connect/disconnect to the Bootstrap or the LwM2M server
  Button#2: can be used to test read/write integer/float/string, etc.
  Button#3: toggle LED1 and, if Observe is set, notify the LwM2M server
  Button#4: toggle LED2 and, if Observe is set, notify the LwM2M server

* TCP client
  * Client project: \nRF5_SDK_15.3.0\examples\nrf5-nrf91-network-clients\tcp_client
  * nRF91 Configuration: CONFIG_THIN_TCPIP_SERVICE, CONFIG_THIN_SNTP_SERVICE

Works with TCP service. The buttons on DK are defined as below: 
  Button#1: connect to remote TCP server by hostname
  Button#2: connect to remote TCP server by IP address
  Button#3: disconnect from the remote server
  Button#4: send data to remote server

TCP Receive is sent after TCP Send is acknowledged.

* AT command client and UDP client
  * Client project: \nRF5_SDK_15.3.0\examples\nrf5-nrf91-network-clients\tcp_client
  * nRF91 Configuration: CONFIG_THIN_TCPIP_SERVICE, CONFIG_THIN_SNTP_SERVICE

Works with UDP service. The buttons on DK are defined as below: 
  Button#1: send data to remote server by hostname
  Button#2: send data to remote server by IP address

UDP ReceiveFrom is sent after UDP SendTo is acknowledged.

About this project
------------------

This application is one of several applications that has been built by the support team at Nordic Semiconductor, as a demo of some particular feature or use case. It has not necessarily been thoroughly tested, so there might be unknown issues. It is hence provided as-is, without any warranty.

However, in the hope that it still may be useful also for others than the ones we initially wrote it for, we've chosen to distribute it here on GitHub.

The application is built to be used with the official nRF5 SDK, that can be downloaded from http://developer.nordicsemi.com/
