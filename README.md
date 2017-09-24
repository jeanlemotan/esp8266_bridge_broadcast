# esp8266_bridge_broadcast
A SPI to ESP8266 broadcast (rfmon) firmware

This lib + firmware allows you to inject and receive packets using an esp8266 module.
It's meant for streaming data - like video - similarly to the wifibroadcast project, but instead of using of the shelf wifi dongles with patched firmwares, it uses the esp8266.

The advantages over standard wifi dongles (like the WN721N) are:
* Ability to control the rate (from 1Mbps to 56Mbps) and modulation (CCK with and w/o short preamble and ODFM) 
* Cheaper and readily available
* Very short stack so less points of failure. It doesn't have to go through the kernel, 802.11 stack, rate control, firmware etc
* Easy to add new features in the firmware
* Very good sensitivity and power: Up to -92dBi and 22.5dBm
* SPI connection so no USB issues on the Raspberry Pi

Disadvantages:
* More complicated connectivity. The module is connected through SPI to the host device which is a bit more complicated than just plugging a USB dongle
* Limited bandwidth. With PIGPIO, 12Mhz SPI speed and 10us delay you can get ~8Mbps throughput. Recommended settings are 10Mhz and 20us delay which results in 5-6Mbps


There are 2 helper classes in the project:
* A Phy which talks to the esp firmware. It supports:
  - Sending and receiving data packets up to 1376K. Data is sent through the SPI bus in packets of 64 bytes. When receiving you get the RSSI as well, per packet.
  - Changing the power settings, in dBm from 0 to 20.5
  - Changing the rate & modulation. These are the supported ones:
  	0:  802.11b 1Mbps, CCK modulation
		1:  802.11b 2Mbps, CCK modulation
		2:  802.11b 2Mbps, Short Preamble, CCK modulation
		3:  802.11b 5.5Mbps, CCK modulation
		4:  802.11b 5.5Mbps, Short Preamble, CCK modulation
		5:  802.11b 11Mbps, CCK modulation
		6:  802.11b 11Mbps, Short Preamble, CCK modulation
		7:  802.11g 6Mbps, ODFM modulation
		8:  802.11g 9Mbps, ODFM modulation
		9:  802.11g 12Mbps, ODFM modulation
		10: 802.11g 18Mbps, ODFM modulation
		11: 802.11g 24Mbps, ODFM modulation
		12: 802.11g 36Mbps, ODFM modulation
		13: 802.11g 48Mbps, ODFM modulation
		13: 802.11g 56Mbps, ODFM modulation
  - Changing the channel.*This is broken for now as the radio doesn't seem to react to this setting for some reason.
  - Getting stats from the esp module - like data transfered, packets dropped etc.

* A FEC_Encoder that does... fec encoding. It allows settings as the K & N parameters (up to 16 and 32 respectively), timeout parameters so in case of packet loss the decoder doesn't get stuck, blocking and non blocking operation.

Both classes can be used independently in other projects.

There is also a test app (esp8266_app) that uses them and sends whatever is presented in its stdin and outputs to stdout whatever it received. You can configure the fec params, the spi speeds and the phy rates/power/channel.



