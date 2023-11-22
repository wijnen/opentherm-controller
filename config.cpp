// Configuration of opentherm controller firmware.
// Values in this file should be edited by the user.

// Comment this out if you use a direct gpio connection for OpenTherm.
//#define USE_OTGW

// Pin definitions: these must match the hardware connections. {{{
// Not used when OpenTherm Gateway is used.
#ifndef USE_OTGW
// Opentherm pins.

// Input pin; pin change interrupt id.
#define opentherm_in_int 2
#define PCINT_VECT PCINT0_vect

// Ouput pin.
#define opentherm_out_pin Gpio::make_pin(PD, 3)

#endif

// Gpio pins where the external pumps are connected.
#define pump_pins { /* {{{ */ \
	Gpio::make_pin(PB, 0) \
} // }}}

// Gpio pins where all the valves are connected.
#define valve_pins { /* {{{ */ \
	Gpio::make_pin(PB, 1), \
	Gpio::make_pin(PB, 2), \
	Gpio::make_pin(PB, 3), \
	Gpio::make_pin(PB, 4), \
	Gpio::make_pin(PB, 5), \
	Gpio::make_pin(PB, 6), \
	Gpio::make_pin(PB, 7), \
	Gpio::make_pin(PD, 1), \
	Gpio::make_pin(PD, 2) \
} // }}}

// }}}

// Other settings: these can be set as desired. {{{

// Maximum number of zones.
// num_valves is defined later as the number of elements in valve_pins.
#define num_zones num_valves

// Frequency of clock which handles PID updates.
// Use 125 Hz because with a 16MHz clock,
// this gives OCR0A=16M/1024/125=125 for an exact result (with DIV=1024)
#define Hz 125

// Default period of PID updates.
// The uint32_t ensures there is no overflow in the expression.
#define default_pid_period (5 * 60 * uint32_t(Hz)) // PID and PWM period (default 5 minutes).

// }}}

// vim: set foldmethod=marker :
