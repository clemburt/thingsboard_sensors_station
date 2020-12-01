# thingsboard_sensors_station
Arduino code for interfacing sensors with ThingsBoard

- System monitoring remote weather and topographic sensors, via different communication protocols (Bluetooth and Ethernet) and a laptop

Data from sensors are recovered by the electronic board which powers them, and sent to ThingsBoard, an open source cloud where data can be stored and displayed in a playful way and real time, thanks to dashboards (charts, gauges, curves…)

- Sturdy and intuitive device, Plug and Play type, for a non-professional audience

For example a teacher who wants to make a weather tracking with his students

To do so, the user connects the provided electronic assembly to a computer, fills in the Arduino code with the name and of the Internet network (Wi-Fi or Ethernet) and its password, and recovers and displays the data on his ThingsBoard account

- Communication between the electronic board and ThingsBoard requires a MQTT (Message Queuing Telemetry Transport) protocol

Lightweight messaging protocol, publish/subscribe type via a message broker, used for M2M (Machine To Machine) exchanges
