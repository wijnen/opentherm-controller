# Overview
The is firmware for an avr microcontroller that is connected to an OpenTherm+
heater, one or more radiator valves and zero or more extra pumps. To operate,
it needs to receive periodic updates about the temperature in each of the zones
it controls. It will open and close the valves, start and stop the extra pumps
and send and receive data to and from the heater. It also receives settings,
including the target temperature for any of the zones it controls.

# Installation
To use this firmware, you need:

  - an avr board, such as an Arduino (an Arduino Uno will do, but with a device that has two serial ports, such as the Arduino Mega, you can also get debugging output).
  - Hardware to connect the avr to the OpenTherm port of a heater. This can either be something that is controlled using two gpio pins, or an OpenTherm Gateway that uses the interface firmware.
  - a relay board for controlling the valve(s) and extra pump(s).
  - a (mini-)computer that passes temperature measurements and target temperatures (and possibly other requests) to the avr.

Once you have your hardware set up, you need to edit config.cpp.

After that, run *make upload* to compile it and upload it to the avr.

# Interface
The system is controlled using the serial port(s):

  - Serial port 0 is used as the command interface. This is described below.
  - Serial port 1 is used for debugging output (if it is available).
  - Serial port 2 is used to connect to the OpenTherm Gateway board, if it is enabled in config.cpp.

There are a few command types:

  - Set a value using: "S" `<code>` `<index>` "=" `<value>` "\n". Example: "SC2=3f\n" to set the current temperature (code = "C") for zone 0x2 to 0x3f.
  - Get a value using: "G" `<code>` `<index>` "\n". Example: "GC2\n" to get the current temperature (code = "C") for zone 2.
  - Write a custom OpenTherm Data-Id using: "W" `<id>` "=" `<value>` "\n".  Example: "W38=3700\n" to set the hot tap water temperature (id=0x38=56) to 55.00&deg;C (0x37 + 0x00 / 256).
  - Read a custom OpenTherm Data-Id using: "R" `<code>` "\n". Example "R38\n" to read the hot tap water temperature.

Valid codes for set/get are (with index type):

  - C (zone): current temperature. This needs to be periodically sent. Reading returns that last sent value.
  - T (zone): temperature setpoint.
  - P (valve): Proportional configuration value for PID control
  - I (valve): Integrating configuration value for PID control
  - D (valve): Differntiating configuration value for PID control
  - B (valve): Current buffer value for integrating component of PID control
  - Z (valve): Which zone this valve belongs to
  - F (valve): Which extra (floor) pump is connected to this valve; -1 means no pump.
  - O (valve): Whether the valve is normally closed (0) or normally open (1)
  - t (-): The PID period (given in 1/Hz seconds, so with Hz=125, a value of 125 means 1 second). The index for this command is ignored, but must be specified anyway.
  - S (valve): (Get only) The current state of the valve (closed(0) or open(1))
  - E (pump): (Get only) The current state of the extra pump (off(0) or on(1))
