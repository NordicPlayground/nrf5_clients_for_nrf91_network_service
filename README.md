nRF52 Serial LTE Modem sample clients
=====================================
This repository contains nRF52 sample projects as serial-lte-modem clients.

More about serial-lte-modem project, please refer to below link first
https://github.com/NordicPlayground/fw-nrfconnect-nrf/tree/master/samples/nrf9160/serial-lte-modem

Environments
------------
To build nRF52 projects:
* DK: nRF52840/PCA10056 or nRF52832/PCA10040
* nRF52 SDK: SDK15.3.0
* nRF52 Toolchain: Keil

Instructions
------------
Please clone this repository into <local_path>\nRF5_SDK_15.3.0\examples\.

* BLE and AT commands test
  * Client project: \nRF5_SDK_15.3.0\examples\nrf5-serial-lte-modem-clients\ble_app_uart
  * nRF91 Configuration: none

* AT command client and MQTT client
  * Client project: \nRF5_SDK_15.3.0\examples\nrf5-serial-lte-modem-clients\mqtt_client
  * nRF91 Configuration: CONFIG_MQTT_SOCKET_LIB, CONFIG_THIN_MQTT_SERVICE

* AT command client and LwM2M client
  * Client project: \nRF5_SDK_15.3.0\examples\nrf5-serial-lte-modem-clients\lwm2m_client
  * nRF91 Configuration: CONFIG_LWM2M, CONFIG_THIN_LWM2M_SERVICE

About this project
------------------

This application is one of several applications that has been built by the support team at Nordic Semiconductor, as a demo of some particular feature or use case. It has not necessarily been thoroughly tested, so there might be unknown issues. It is hence provided as-is, without any warranty.

However, in the hope that it still may be useful also for others than the ones we initially wrote it for, we've chosen to distribute it here on GitHub.

The application is built to be used with the official nRF5 SDK, that can be downloaded from http://developer.nordicsemi.com/
