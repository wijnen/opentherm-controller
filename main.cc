#include <math.h>
#include <stdint.h>
#include "config.cpp"

// Amat configuration {{{
#ifndef NDEBUG
#define DBG1_ENABLE
#endif

#define USART_TX0_SIZE 100
#define USART0_ENABLE_RX 100

#ifdef USE_OTGW
#define USART_TX2_SIZE 100
#define USART2_ENABLE_RX 100
#endif
// }}}
#include <amat.hh>

// Types. {{{
enum OpenthermMasterType {
	Read,
	Write,
	Invalid
};
enum OpenthermSlaveType {
	ReadAck,
	WriteAck,
	InvalidAck,
	Unknown
};
struct Valve { // {{{
	// Values that are set through the control connection.
	int zone;
	int pump;	// Which external pump should be activated for this valve; -1 for no pump.
	bool normally_open;
	float P, I, D;		// Settings for PID control.
	// Computed values.
	int next_valve;	// Valves in a zone are a linked list.
	float I_buffer;		// Internal buffer for PID control.
	float out;		// Output value of PID system.
	bool active;		// If true, boiler is working for this valve.
	float pwm;		// Duty cycle of pwm (0 < value <= 1).
}; // }}}

struct Zone { // {{{
	// Values that are set through the control connection.
	float temperature;	// Current temperature, as received from control connection (presumably by thermostat).
	float setpoint;		// Current setpoint, as received from control connection.
	// Computed values.
	int first_valve;	// Valves in a zone are a linked list.
	float last_temperature;	// Temperature of last iteration, for use with D component.
}; // }}}
// }}}

// Globals. {{{
// All non-const variables are marked volatile even if they don't need to be.
// This is because there is only a small cost to performance, but it saves very
// hard to debug issues.
static uint8_t const pump_pin[] = pump_pins;
static uint8_t const valve_pin[] = valve_pins;
static volatile uint32_t pid_period = default_pid_period;
// Globals that are computed from configuration.
static unsigned int const num_valves = sizeof(valve_pin) / sizeof(*valve_pin);
static unsigned int const num_pumps = sizeof(pump_pin) / sizeof(*pump_pin);
#ifndef USE_OTGW
static uint8_t const opentherm_in_pin = Pcint::get_pin(opentherm_in_int);	// Input pin as gpio.
static uint8_t const opentherm_in_group = opentherm_in_pin >> 3;
static volatile uint8_t recv_ovf;
#endif

// Uninitialized variables.
static volatile Valve valve[num_valves]; // Valves.
static volatile Zone zone[num_zones]; // Zones.
static volatile unsigned int pump_active[num_pumps];	// Number of valves that are requesting the pump to run.
static volatile uint8_t clock_phase;	// Count up to Hz for seconds clock.
static volatile uint32_t pid_timer;	// Count up to selected period for pid update (unit is clock ticks, so Hz per second).
static volatile float current_out = -INFINITY;	// Current output temperature for boiler.

// Current received opentherm command.
static volatile uint8_t opentherm_type;
static volatile uint8_t opentherm_id;
static volatile uint16_t opentherm_value;
static volatile bool opentherm_ready;	// Flag to notify that the above variables have been filled with a new value.
// }}}

// Declarations. {{{
static void opentherm_transaction(uint8_t type, uint8_t id, uint16_t value, bool block);
static uint8_t read_digit(uint8_t src);
// }}}

#include "controller.cpp"

// Documentation. {{{
// Terminology:
// - Zone: an area that is controlled as a unit by one or more heaters.
// - Valve: a component that is opened (or closed) in response to the output of a single I/O-pin; each valve controls a single heater.

// Design (hardware):
// - Serial port 0 connects to control interface
// - Serial port 1 connects to debug output.
// - Serial port 2 connects to opentherm interface
// - GPIO connects to relay for external pumps, which are enabled for e.g. floor heaters
// - GPIO array connects to relays for heater valves; one for each heater.
// - On the opentherm gateway, the interface firmware is used (http://otgw.tclcode.com/interface.html).
// - Watchdog is used to reset the system if it hangs. On reset, all zones are set to defrost (5 degrees).
// - Counter 0 is used to capture and send opentherm messages (if OTGW is not used).
// - Counter 1 is used to trigger periodic opentherm command sending and to run PID state updates and handle PID PWM.

// Design (firmware):
// - Every zone has a temperature setpoint.
// - Current temperature in each zone is reported periodically.
// - Every valve has PID settings.
// - Once per period (which is usually 5 minutes), every valve's PID control is updated. The result is an output control for the requested temperature to be sent to this valve.
// - The output control can be overruled by the controller. This can also be used for zones without a thermostat. (TODO: This is not currently implemented.)
// - For every valve where the output control is lower than the current zone temperature, the output control is disabled and the valve is closed.
// - If all output controls are disabled, the boiler is switched off; otherwise it is switched on as follows:
// - The control setpoint of the boiler is set to the highest value of any of the valves.
// - For every valve the fraction of that temperature is computed.
// - Valves are PWM'd using their zone's fraction. The PWM is fixed frequency with variable duty cycle, one on-pulse and one off-pulse during a single period (which is the same as the PID period, so usually 5 minutes).
// - While any of the floor heaters are on, the corresponding external pump is enabled.

// List of interrupts:
// - data received on control serial port.
// - data received on opentherm serial port (if using otgw).
// - opentherm packet start on pcint (if not using otgw).
// - timer for sending next opentherm command and computing next PID state.

// Configuration (set and retrieve current values through serial interface):
// - Current temperature for every zone.
// - Temperature setpoint for every zone.
// - P, I, D values for every valve.
// - PID I-buffer for every valve.
// - Zone assignment for every valve.
// - Which pump it links to (-1 for none), for every valve.
// - Whether the valve is normally open or normally closed, for every valve.
// - PID and PWM period (default 5 minutes).

// Status (read-only values which can be retrieved through serial interface):
// - Output value (valve open or closed) for every valve.
// - External pump output (on or off) for every pump.
// - Anything that the boiler unit reports.

// For reference, this is the program that the opentherm gateway page uses:
//
// MsgID 0: Master and Slave Status flags	(read)
// MsgID 25: Boiler flow water temperature	(read)
// MsgID 1: Control Setpoint			(write)
// UserDefined
// MsgID 17: Relative Modulation Level		(read)
// MsgID 27: Outside temperature		(read)
// MsgID 28: Return water temperature		(read)
// MsgID 14: Maximum relative modulation level setting	(write)
// UserDefined
// MsgID 0: Master and Slave Status flags	(read again)
// MsgID 25: Boiler flow water temperature	(read again)
// MsgID 1: Control Setpoint			(write again)
// MsgID 56: DHW Setpoint			(read or write)
// MsgID 57: Max CH water Setpoint		(read or write)
// UserDefined
// MsgID 18: Water pressure in CH circuit 	(read)

// The split between read at boot, read when requested and monitor is a
// suggestion for a host program; the firmware does not read any of these
// unless requested.

// Useful readable info (read at boot):
// 0, 3: status flags, slave configuration
// 15: max capacity/min modulation level
// 48: DHW setpoint bounds for adjustment
// 49: CH setpoint bounds for adjustment
// (50: OTC heat curve ratio bounds for adjustment)	OTC is "outside temperature correction", for adjusting target water temperature based on outside temperature.
// 56: DHW setpoint
// 57: max CH water setpoint
// 58: OTC heat curve ratio
// 125: Opentherm version implemented in slave

// Useful readable info (read when requested):
// 12, 13: fault history
// 20, 21, 22: date+time
// 116: Number of burner starts
// 117: Number of CH pump starts
// 118: Number of DHW pump/valve starts
// 119: Number of burner starts in DHW mode
// 120: Number of burning hours
// 121: Number of CH pump operation hours
// 122: Number of hours DHW pump has been running or DHW valve has been opened
// 123: Number of burning hours in DHW mode

// Useful readable info (monitor):
// 17: relative modulation level
// 18: CH pressure
// 19: DHW flow rate.
// 25: Boiler flow water temperature
// 26: DHW temperature
// 27: Outside temperature
// 28: Return water temperature
// 29: Solar storage temperature
// 30: Solar collector temperature
// 31: CH2 flow water temperature
// 32: DHW2 temperature
// 33: Exhaust temperature

// Useful writable info:
// 1: CH temperature setpoint.
// 14: maximum relative modulation level
// 16: room setpoint
// 23: room setpoint for CH2
// 24: room temperature
// 56: DHW setpoint
// 57: max CH water setpoint
// (58: OTC heat curve ratio)
// 124: opentherm version implemented in master
// }}}

// Compute parity of data.
static bool compute_parity(uint8_t src) { // {{{
	src ^= src >> 4;
	src ^= src >> 2;
	src ^= src >> 1;
	return src & 1;
} // }}}

// Parse a single hexadecimal digit.
static uint8_t read_digit(uint8_t src) { // {{{
	if (src >= '0' && src <= '9')
		return src - '0' + 0x0;
	if (src >= 'a' && src <= 'f')
		return src - 'a' + 0xa;
	if (src >= 'A' && src <= 'F')
		return src - 'A' + 0xA;
	dbg("received invalid digit #", src);
	return 0;
} // }}}

// Initialize everything.
static void setup() { // {{{
	dbg("system starts");

	// Set up gpio pins: set all output pins to output (low).
	for (unsigned i = 0; i < num_pumps; ++i)
		Gpio::write(pump_pin[i], false);
	for (unsigned i = 0; i < num_valves; ++i)
		Gpio::write(valve_pin[i], false);

	// Start system clock.
	Counter::set_ocr1a(F_CPU / 1024 / Hz);
	Counter::enable1(Counter::s1_div1024, Counter::m1_ctc_ocra);

#ifndef USE_OTGW
	Pcint::enable_group(opentherm_in_group);
#endif
} // }}}

// This clock triggers Hz times per second.
ISR(TIMER1_COMPA_vect) {
	// Check if valves need to be closed for pwm. {{{
	float now = ++pid_timer / pid_period;
	for (unsigned vi = 0; vi < num_valves; ++vi) {
		volatile Valve *v = &valve[vi];
		if (!v->active)
			continue;
		if (now > v->pwm) {
			Gpio::write(valve_pin[vi], v->normally_open);
			v->active = false;
			if (v->pump >= 0) {
				if (--pump_active[v->pump] == 0)
					Gpio::write(pump_pin[v->pump], false);
			}
		}
	}
	// }}}

	// Make and use 1s clock for sending next opentherm command. {{{
	if (++clock_phase < Hz)
		return;
	clock_phase = 0;

	// This code is running once per second.
	send_next_opentherm();
	// }}}

	// Run PID update for each valve. {{{
	if (pid_timer < pid_period)
		return;
	pid_timer = 0;

	// This code is running once per pid timer period.

	// Compute new pid state.
	float max_out = -INFINITY;
	for (unsigned i = 0; i < num_pumps; ++i)
		pump_active[i] = 0;
	for (unsigned vi = 0; vi < num_valves; ++vi) {
		// Ignore zone with no valves.
		volatile Valve *v = &valve[vi];
		if (v->zone < 0)
			continue;
		volatile Zone *z = &zone[v->zone];

		float error = z->temperature - z->setpoint;
		float dt = pid_period;
		float part_P = v->P * -error;
		float part_D = v->P * v->D * (z->temperature - z->last_temperature) / dt;
		v->out = part_P + v->I_buffer + part_D;
		v->active = v->out > z->temperature;
		if (v->active && v->pump >= 0)
			pump_active[v->pump]++;
		if (v->active && v->out > max_out)
			max_out = v->out;
		// Update I_buffer
		v->I_buffer += v->P / v->I * -error * dt;
		if (v->I_buffer < z->setpoint - 10)
			v->I_buffer = z->setpoint - 10;
		if (v->I_buffer > z->setpoint + 50)
			v->I_buffer = z->setpoint + 50;
	}
	// Set new pump state.
	for (unsigned i = 0; i < num_pumps; ++i)
		Gpio::write(pump_pin[i], pump_active[i] > 0);

	// Set last temperature.
	for (unsigned z = 0; z < num_zones; ++z)
		zone[z].last_temperature = zone[z].temperature;

	if (!isinf(max_out)) {
		// At least one valve is requesting a temperature.
		for (unsigned vi = 0; vi < num_valves; ++vi) {
			volatile Valve *v = &valve[vi];
			if (!v->active)
				continue;
			float sp = zone[v->zone].setpoint;
			v->pwm = (max_out - sp) / (v->out - sp);
			Gpio::write(valve_pin[vi], !v->normally_open);
		}
	}
	if (current_out != max_out) {
		// Temperature changed; immediately update.
		current_out = max_out;
		opentherm_transaction(Write, 1, current_out, true);
	}
	// }}}
}

// Receive command data.
static void usart_rx0(uint8_t data, uint8_t len) { // {{{
	if (data != '\n')
		return;
	// Copy command into buffer and handle it.
	uint8_t buffer[len];
	Usart::rx0_move(buffer, len);
	buffer[len - 1] = '\0';
	handle_command(buffer, len);
} // }}}

// Receive opentherm data.
static void recv_opentherm(uint8_t type, uint8_t id, uint16_t value) { // {{{
	bool parity = compute_parity(type) ^ compute_parity(id) ^ compute_parity((value >> 8) & 0xff) ^ compute_parity(value & 0xff);
	if (parity) {
		dbg("parity error: #, #, *", type, id, value);
		return;
	}
	if (type & 0xf) {
		dbg("spare bits are not 0: #", type);
		return;
	}
	type = (type >> 4) & 0x7;
	if (type & 0x4) {
		dbg("invalid type: #", type);
		return;
	}
	opentherm_type = type;
	opentherm_id = id;
	opentherm_value = value;
	opentherm_ready = true;
} // }}}

// Opentherm interface has 2 options: opentherm gateway and native. Both define send_opentherm(uint32_t data), which should only be called when the timing is right, and they call recv_opentherm(uint32_t data) when a response is received.
// Frame format: 1PTTT----IIIIIIIIVVVVVVVVVVVVVVVV1	(Start(1), Parity(1), Message Type(3), Spare(4), Data Id(8), Data Value(16), Stop(1))
#ifdef USE_OTGW // {{{
static void send_opentherm(uint8_t type, uint8_t id, uint16_t value) { // {{{
	if (type > 2)
		dbg("warning: sending invalid type #", type);
	bool parity = compute_parity(type) ^ compute_parity(id) ^ compute_parity(value & 0xff) ^ compute_parity((value >> 8) & 0xff);
	if (parity)
		type |= 0x8;
	Usart::tx2_write(Avr::digit(type));
	Usart::tx2_write('0');
	Usart::tx2_write(Avr::digit((id >> 4) & 0xff));
	Usart::tx2_write(Avr::digit(id & 0xff));
	Usart::tx2_write(Avr::digit((value >> 12) & 0xff));
	Usart::tx2_write(Avr::digit((value >> 8) & 0xff));
	Usart::tx2_write(Avr::digit((value >> 4) & 0xff));
	Usart::tx2_write(Avr::digit(value & 0xff));
	Usart::tx2_write('\n');
} // }}}

// Receive opentherm data.
static void usart_rx2(uint8_t data, uint8_t len) { // {{{
	if (data != '\n')
		return;
	if (len != 10) {
		dbg("received invalid packet from gateway (len #)", len);
		Usart::rx2_pop(len);
		return;
	}
	uint8_t buffer[10];
	Usart::rx2_move(buffer, 10);
	if (buffer[0] != 'B') {
		dbg("Invalid first byte #", buffer[0]);
		return;
	}
	uint8_t type = (read_digit(buffer[1]) << 4) | read_digit(buffer[2]);
	uint8_t id = (read_digit(buffer[3]) << 4) | read_digit(buffer[4]);
	uint8_t valueH = (read_digit(buffer[5]) << 4) | read_digit(buffer[6]);
	uint8_t valueL = (read_digit(buffer[7]) << 4) | read_digit(buffer[8]);
	uint16_t value = (valueH << 8) | valueL;
	recv_opentherm(type, id, value);
} // }}}
// }}}
#else // {{{
static void send_opentherm(uint8_t type, uint8_t id, uint16_t value) { // {{{
	if (type > 2)
		dbg("warning: sending invalid type #", type);
	bool parity = compute_parity(type) ^ compute_parity(id) ^ compute_parity(value & 0xff) ^ compute_parity((value >> 8) & 0xff);
	if (parity)
		type |= 0x8;
	type <<= 4;
	uint32_t raw = (uint32_t(type) << 24) | (uint32_t(id) << 16) | value;
	// With a 16MHz clock and a divider of 64, the clock runs at 250kHz, so 125 ticks per half millisecond (which is the OpenTherm pulse width).
	Counter::enable0(Counter::s0_div64, Counter::m0_ctc);
	Counter::set_ocr0a(F_CPU / 64 / 2000);	// 2 overflows per ms for sending command.
	Gpio::write(opentherm_out_pin, false);
	Counter::clear_ints0();
	// Start bit. {{{
	while (!Counter::has_ovf0()) {}
	Gpio::write(opentherm_out_pin, true);
	Counter::clear_ints0();
	while (!Counter::has_ovf0()) {}
	Gpio::write(opentherm_out_pin, false);
	Counter::clear_ints0();
	// }}}
	// Data bits. {{{
	for (int8_t bit = 31; bit >= 0; --bit) {
		bool b = (raw >> bit) & 1;
		while (!Counter::has_ovf0()) {}
		Gpio::write(opentherm_out_pin, b);
		Counter::clear_ints0();
		while (!Counter::has_ovf0()) {}
		Gpio::write(opentherm_out_pin, !b);
		Counter::clear_ints0();
	}
	// }}}
	// Stop bit. {{{
	while (!Counter::has_ovf0()) {}
	Gpio::write(opentherm_out_pin, true);
	Counter::clear_ints0();
	while (!Counter::has_ovf0()) {}
	Gpio::write(opentherm_out_pin, false);
	// }}}

	// Prepare counter for receiving reply.
	Counter::enable0(Counter::s0_div8, Counter::m0_ctc);
	Counter::set_ocr0a(F_CPU / 8 / 20000);	// Overflow after 50 μs; 20 overflows per ms. (Useful unit because of transition window is 900-1150 μs, so 18-23 overflows.)
	recv_ovf = -1;	// Prepare overflow counter as "not counting yet".
	Counter::clear_ints0();
	Counter::enable_ovf0();
	Pcint::clear_group(opentherm_in_group);
	Pcint::enable(opentherm_in_int);
} // }}}

// Receive opentherm data.
ISR(TIMER0_OVF_vect) { // {{{
	if (recv_ovf < 0) {
		// We are not receiving a packet, so ignore overflows.
		return;
	}
	if (++recv_ovf > 24) {
		dbg("Data error: no valid reply received");
		// Disable further reception.
		Counter::disable_ovf0();
		Pcint::disable(opentherm_in_int);
	}
} // }}}

ISR(PCINT_VECT) { // {{{
	static bool recv_flip;
	static uint8_t recv_bit;
	static uint32_t opentherm_raw;
	bool b = Gpio::read(opentherm_in_pin);
	if (recv_ovf < 0) {
		// We are waiting for the start bit. Start the timer (if this is a falling edge).
		if (!b) {
			// Reset counter for better timing.
			Counter::write0(0);
			// Start counting overflows.
			recv_ovf = 0;
			recv_flip = false;
			recv_bit = 0;
			opentherm_raw = 0;
		}
		return;
	}
	// Ignore (at most one) bit flip that is not in the window.
	if (++recv_ovf < 17) {
		if (recv_flip) {
			Counter::disable_ovf0();
			Pcint::disable(opentherm_in_int);
			dbg("noise found during opentherm reception");
		}
		recv_flip = true;
		return;
	}
	// Edge found.
	// Reset counter for better timing.
	Counter::write0(0);
	// Start counting overflows.
	recv_ovf = 0;
	recv_flip = false;
	// Check if this is the stop bit.
	if (++recv_bit > 31) {
		// Stop bit found; disable further events.
		Counter::disable_ovf0();
		Pcint::disable(opentherm_in_int);
		if (b) {
			dbg("incorrect stop bit received");
			return;
		}
		// Stop bit was correct; handle data.
		uint8_t type = (opentherm_raw >> 24) & 0xff;
		uint8_t id = (opentherm_raw >> 16) & 0xff;
		uint16_t value = opentherm_raw & 0xffff;
		recv_opentherm(type, id, value);
		return;
	}
	// Edge found; record it.
	opentherm_raw <<= 1;
	if (b)
		opentherm_raw |= 1;
} // }}}
#endif // }}}

// Send an opentherm command and wait for a reply.
static void opentherm_transaction(uint8_t type, uint8_t id, uint16_t value, bool block) { // {{{
	static volatile bool locked = false;
	// Claim lock. {{{
	while (true) {
		cli();	// Avoid race conditions by diabling interrupts.
		if (!locked) {
			locked = true;
			sei();
			break;
		}
		sei();
		// Lock was clamed. If not blocking, return. Otherwise wait for
		// lock to be released and then retry claiming it.
		if (!block)
			return;
		while (locked) {}
	}
	// }}}
	opentherm_ready = false;
	send_opentherm(type, id, value);
	while (!opentherm_ready) {}
	if (opentherm_id != id)
		dbg("invalid id # returned, should be #", opentherm_id, id);
	// Wait at least 100 ms.
	// Timer1 runs at 125 Hz.
	// 12.5 overflows is 0.1 s.
	// The first overflow may happen in 0 cycles, so wait for 14 overflows to be sure.
	for (uint8_t i = 0; i < 14; ++i) {
		Counter::clear_ints1();
		while (!Counter::has_ovf1()) {}
	}
	// Release lock.
	locked = false;
} // }}}

// vim: set foldmethod=marker :
