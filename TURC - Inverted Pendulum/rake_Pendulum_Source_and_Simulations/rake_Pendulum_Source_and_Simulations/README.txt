Nathanael Rake
njr602@utulsa.edu
01 May 2017

Contents
- simulations: this folder contains MatLab simulations that were used to model the system dynamics of the inverted pendulum and determine the full-state feedback gains required to stabilize the system.

- stateSpace: an Arduino sketch used to control the inverted pendulum using a state space model. This sketch is meant to be run on a single board and does not require a connection to a CAN network. 

- stateSpace_CAN_sensor: an Arduino sketch that works with another node (running the stateSpace_CAN_controller sketch) over a CAN network to control the inverted pendulum using the state space model. The node running this sketch is responsible for reading the system encoders and driving the actuator. This node also measures/computes the system state and transmits it over the CAN connection. The node also drives the actuator according to the value received over the CAN connection from a controller node.

- stateSpace_CAN_controller: an Arduino sketch that works with another node (running the stateSpace_CAN_sensor sketch) over a CAN network to control the inverted pendulum using the state space model. The node running this sketch is responsible for computing the analog signal used to drive the system actuator. The value of this signal is computed from the system state which is received on the CAN connection from the sensor/actuation node. The value of this signal is then transmitted over the CAN connection.