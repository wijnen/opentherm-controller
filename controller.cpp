// This file contains the high level parts of the controller.

// Send an opentherm command (to keep the line active).
static void send_next_opentherm() { // {{{
	opentherm_transaction(Write, 1, current_out, false);	// Repeatedly write setpoint, to avoid timeouts.
	dbg("status: #, #, *", opentherm_type, opentherm_id, opentherm_value);
} // }}}

// Helper functions for handle_command.
static uint8_t read_u8(volatile uint8_t *buffer) { // {{{
	return (read_digit(buffer[0]) << 4) | read_digit(buffer[1]);
} // }}}
static uint16_t read_u16(volatile uint8_t *buffer) { // {{{
	return (read_digit(buffer[0]) << 12) | (read_digit(buffer[1]) << 8) | (read_digit(buffer[2]) << 4) | read_digit(buffer[3]);
} // }}}

// Command received on serial port; handle it.
static void handle_command() { // {{{
	// Supported commands:
	// SC<zone>=<value> / GC<zone>			Set/get current temperature for a zone.
	// ST<zone>=<value> / GT<zone>			Set/get temperature setpoint for a zone.
	// S<P|I|D><valve>=<value> / G<P|I|D><valve>	Set/get P, I, D values for every valve.
	// SB<valve>=<value> / GB<valve>		Set/get PID I-buffer for every valve.
	// SZ<valve>=<zone> / GZ<valve>			Set/get Zone assignment for every valve.
	// SF<valve>=<pump> / GF<valve>			Set/get which (floor) pump this is connected to (-1 for none), for every valve.
	// SO<valve>=<0|1> / GO<valve>			Set/get Whether the valve is normally open or normally closed, for every valve.
	// St<id>=<period> / Gt<id>			Set/get PID and PWM period (default 5 minutes). The id is ignored.
	// GS<valve>					Get Output value (valve open or closed) for every valve.
	// GE<pump>					Get External pump output (on or off).

	// R<id>					Read from opentherm.
	// W<id>=<value>				Write to opentherm.

	// Parser options:
	// S<code><id>=<value>
	// G<code><id>
	// R<id>
	// W<id>=<value>
	// code: 1 character
	// id: 2 character hex
	// value: 4 character hex
	if (command_len == 0)
		return;
	uint8_t id;
	uint16_t value;
	uint8_t code;
	switch (command_buffer[0]) {
	case 'S':
		if (command_len != 9) {
			dbg("S command too short");
			return;
		}
		id = read_u8(&command_buffer[2]);
		value = read_u16(&command_buffer[5]);
		code = command_buffer[1];
		command_len = 0;	// Free buffer.
		sei();
		switch (code) {
		case 'C':	// Current temperature
			if (id >= num_zones)
				dbg("invalid zone");
			else
				zone[id].temperature = value / 256.;
			break;
		case 'T':	// Temperature setpoint
			if (id >= num_zones)
				dbg("invalid zone");
			else
				zone[id].setpoint = value / 256.;
			break;
		case 'P':	// Proportional part of PID
			if (id >= num_valves)
				dbg("invalid valve");
			else
				valve[id].P = value / 256.;
			break;
		case 'I':	// Integral part of PID
			if (id >= num_valves)
				dbg("invalid valve");
			else
				valve[id].I = value / 256.;
			break;
		case 'D':	// Derivative part of PID
			if (id >= num_valves)
				dbg("invalid valve");
			else
				valve[id].D = value / 256.;
			break;
		case 'B':	// I-buffer
			if (id >= num_valves)
				dbg("invalid valve");
			else
				valve[id].I_buffer = value / 256.;
			break;
		case 'Z':	// Zone
			if (id >= num_valves)
				dbg("invalid valve");
			else
				valve[id].zone = value;
			break;
		case 'F':	// Floor pump
			if (id >= num_valves)
				dbg("invalid valve");
			else
				valve[id].pump = value;
			break;
		case 'O':	// Normally open
			if (id >= num_valves)
				dbg("invalid valve");
			else
				valve[id].normally_open = value;
			break;
		case 't':	// PID period
			pid_period = value;	// id is ignored.
			break;
		default:
			dbg("invalid S command");
			return;
		}
		dbg("value has been set");
		break;
	case 'G':
		if (command_len != 4) {
			dbg("S command too short");
			return;
		}
		id = read_u8(&command_buffer[2]);
		code = command_buffer[1];
		command_len = 0;	// Free buffer.
		sei();
		// Report requested data to command port.
		switch (code) {
		case 'C':	// Current temperature
			if (id >= num_zones)
				dbg("invalid zone");
			else
				Usart::tx0_print("GC#:*", id, uint16_t(zone[id].temperature * 256));
			break;
		case 'T':	// Temperature setpoint
			if (id >= num_zones)
				dbg("invalid zone");
			else
				Usart::tx0_print("GC#:*", id, uint16_t(zone[id].setpoint * 256));
			break;
		case 'P':	// Proportional part of PID
			if (id >= num_valves)
				dbg("invalid valve");
			else
				Usart::tx0_print("GC#:*", id, uint16_t(valve[id].P * 256));
			break;
		case 'I':	// Integral part of PID
			if (id >= num_valves)
				dbg("invalid valve");
			else
				Usart::tx0_print("GC#:*", id, uint16_t(valve[id].I * 256));
			break;
		case 'D':	// Derivative part of PID
			if (id >= num_valves)
				dbg("invalid valve");
			else
				Usart::tx0_print("GC#:*", id, uint16_t(valve[id].D * 256));
			break;
		case 'B':	// I-buffer
			if (id >= num_valves)
				dbg("invalid valve");
			else
				Usart::tx0_print("GC#:*", id, uint16_t(valve[id].I_buffer * 256));
			break;
		case 'Z':	// Zone
			if (id >= num_valves)
				dbg("invalid valve");
			else
				Usart::tx0_print("GC#:*", id, valve[id].zone);
			break;
		case 'F':	// Floor pump
			if (id >= num_valves)
				dbg("invalid valve");
			else
				Usart::tx0_print("GC#:*", id, valve[id].pump);
			break;
		case 'O':	// Normally open
			if (id >= num_valves)
				dbg("invalid valve");
			else
				Usart::tx0_print("GC#:*", id, valve[id].normally_open);
			break;
		case 't':	// PID period
			Usart::tx0_print("GC#:*", id, pid_period);
			break;
		case 'S':	// Valve state
			if (id >= num_valves)
				dbg("invalid valve");
			else
				Usart::tx0_print("GC#:*", id, valve[id].active);
			break;
		case 'E':	// External pump state
			if (id >= num_pumps)
				dbg("invalid pump");
			else
				Usart::tx0_print("GC#:*", id, pump_active[id]);
			break;
		default:
			dbg("invalid G command");
			return;
		}
		dbg("get complete");
		break;
	case 'R':
		if (command_len != 3) {
			dbg("S command too short");
			return;
		}
		id = read_u8(&command_buffer[1]);
		command_len = 0;	// Free buffer.
		sei();
		opentherm_transaction(Read, id, 0, true);
		dbg("opentherm read transaction done: #, #, *", opentherm_type, opentherm_id, opentherm_value);
		// Reply is sent from opentherm_transaction.
		break;
	case 'W':
		if (command_len != 8) {
			dbg("S command too short");
			return;
		}
		id = read_u8(&command_buffer[1]);
		value = read_u16(&command_buffer[4]);
		command_len = 0;	// Free buffer.
		sei();
		opentherm_transaction(Write, id, value, true);
		dbg("opentherm write transaction done: #, #, *", opentherm_type, opentherm_id, opentherm_value);
		break;
	default:
		dbg("invalid command");
		command_len = 0;	// Free buffer.
		sei();
		return;
	}
} // }}}

// vim: set foldmethod=marker :
