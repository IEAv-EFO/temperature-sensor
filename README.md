# Read temperature sensor



- Temperature sensor: [MCP9808](https://octopart.com/datasheet/mcp9808-e%2Fms-microchip-21270223).
Resolution +0.5°C, +0.25°C, +0.125°C, +0.0625°C.
- Microprocessor model: Tiva C series Launchpad tm4c123.

i2c Pins:
- SCL   -> PA6
- SDA   -> PA7
  
|     | SCL | SDA |
|-----|-----|-----|
|I2C0 | PB2 | PB3 |
|I2C1 | PA6 | PA7 |  
|I2C2 | PE4 | PE5 |  
|I2C3 | PD0 | PD1 |  

- Vcc   -> 2.7 to 5.5 V
- Gnd

![image](/post-processing/front_panel.png)


Note: When the i2c is working all the disconnected sensors return -0.06 celsius measurement.
When communication fails all values are 0.00, and when it fails stronger it shows the initialization value.
