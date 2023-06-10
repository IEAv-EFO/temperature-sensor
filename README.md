# Read temperature sensor



- Temperature sensor: [MCP9808](https://octopart.com/datasheet/mcp9808-e%2Fms-microchip-21270223).
Resolution +0.5°C, +0.25°C, +0.125°C, +0.0625°C.
- Microprocessor model: Tiva C series Launchpad tm4c123.

i2c Pins:
- SCL   -> PA6
- SDA   -> PA7
- Vcc   -> 2.7 to 5.5 V
- Gnd

Note: When the i2c is working all the disconnected sensors return -0.06 celsius measurement.
When communication fails all values are 0.00, and when it fails stronger it shows the initialization value.