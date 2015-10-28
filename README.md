# DaVinciDriver

DaVinciDriver is an autonomous slotcar, scale 1:32

It is based on:
* Scalextric chassis W8835 Mercedes CLK (spare part for the Challenger)
* [Teensy 3.1](http://pjrc.com/teensy/teensy31.html)
* [Bosch BNO055 IMU](http://www.bosch-sensortec.com/en/homepage/products_3/sensor_hubs/iot_solutions/bno055_1/bno055_4), [Datasheet](http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf), on [Adafruit breakout board](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview)
* [Toshiba TB6612FNG](http://toshiba.semicon-storage.com/info/docget.jsp?did=10660&prodName=TB6612FNG) H-bridge motor driver on [Pololu breakout board](https://www.pololu.com/product/713)
* [NRF24L01+](https://www.nordicsemi.com/kor/content/download/2726/34069/file/nRF24L01P_Product_Specification_1_0.pdf) radio link for telemetry and remote control
* VS1833 Infrared remote control

Libraries used:
* [dc740's library](https://forum.pjrc.com/threads/24307-nRF24L01-working-in-Teensy-2-0-and-Teensy-3-0) for NRF24L01:  [download](https://forum.pjrc.com/attachment.php?attachmentid=1002&d=1381184909)
* Adafruit libraries for BNO055: [download](https://github.com/adafruit/Adafruit_BNO055/archive/master.zip) and [download](https://github.com/adafruit/Adafruit_Sensor/archive/master.zip) (you need both)
* Greiman's SdFat Library: [download](https://github.com/greiman/SdFat)

License: GPL v2.0
