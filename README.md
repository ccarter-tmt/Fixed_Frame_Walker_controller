# Fixed_Frame_Walker_controller
Three-axis stepper-motor actuator controller for Fixed Frame positioning.
The code runs on an Arduino Uno. It reads step/direction switches, a speed (step-rate) potentiometer and generates control signals for three internal stepper-driver modules. These modules control three stepper-motor driven linear actuators.
Control over a serial interface is also possible. A very simple set of commands is available to set the step-counts for each actuator and to query and command moves.
