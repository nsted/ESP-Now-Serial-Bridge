# ESP-Now-Serial-Bridge (optimized version)

ESP32 based serial bridge for transmitting high bitrate serial data between two boards over long ranges.

Optimized version based on https://github.com/yuri-rage/ESP-Now-Serial-Bridge

Tested with baud rate 921600 and video streaming bitrates of up to 400 kbps.
The primary purpose of this sketch was to enable a MAVLink serial connection.
There is no error correction or acknowledgement mechanism. If you require this, it has to be implemented on a higher protocol layer. MAVLink already provides its own mechanisms.

Range is easily better than regular WiFi, however an external antenna may be required for truly long range messaging, to combat obstacles/walls, and/or to achieve success in an area saturated with 2.4GHz traffic.

To find the MAC address of each board, uncomment the `#define DEBUG` line, and monitor serial output on boot.  Set the OPPOSITE board's IP address as the value for RECVR_MAC in the macros at the top of the sketch.

When uploading the sketch, be sure to define `BOARD1` or `BOARD2` for the OPPONENT board
as appropriate before compiling.

-- Yuri - Sep 2021

-- hhackbarth - March 2023

Based this example - https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

## Modifications/Optimizations

The original version was modified for significantly more efficient and buffered serial communication
and for an option to configure for higher radio transmission data rates (2 Mbps).
This allows for much higher throughput like low latency video streaming with 400 kbps.

Adaptions have been made to use it with the tiny XIAO ESP32C3 modules, which are extremely small
and lightweight and have a connector for external antennas.
<img src="https://live.staticflickr.com/65535/52613792130_c31a772c7c_b.jpg">

At the moment, the serial initialization is modified in order to communicate through the USB Serial interface.

By default, ESP NOW works with 1 Mbps data rate.
The option to increase radio transmission data rate to 2 Mbps is experimental.
It is compiling correctly and modules run with these modifications but it could
not yet be proved that the data rate and bandwidth is really doubled.
I encourage others make further performance tests for changes in throughput and range
with 2 Mbps and also with lower data rates (like 250 kbps)

## Further use cases

As mentioned before, the serial bridge also was successfully tested with UDP video
streams at bitrates up to 400 kbps. In order to convert between UDP traffic and serial
there are several options available like "socat" tool or a pppd (daemon for PPP protocol layer).

## License

This fork is based on the work of Yuri on ESP-Now-Serial-Bridge. All rights to him for his pieces of software. Part of the software was also based on the tutorial by randomnerdtutorials.com.

Permission for my modifications is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
