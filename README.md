# Quail_Incubator
Built a quail incubator by using ESP32 to collect the temperature and humidity value and visualize it on Grafana through InfluxDB. The hatch rate is merely 70% when excluding unfertilised eggs.
- Includes automatic temperature control using relay on the light bulb and PID controlled fan.
- Applied Kalman filter to filter out any noises collected by the sensor.
