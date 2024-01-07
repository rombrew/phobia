## Overview

This page introduces you to the Command Line Interface (CLI). We have a regular
CLI with autocompletion and command history.

## Key mapping

These are the basic special keys that are used in the CLI:

* `Return` - Run the command.
* `Backspace` or `Delete` - Erase last typed character.
* `Tab` or (@) - Automplete function.
* `Shift` + `Tab` - Automplete function reverse.
* `Ctrl` + `C` or `Ctrl` + `D` - Drop the content of the line or abort the command.
* `Ctrl` + `P` or `Up` or `*` - History function scroll up.
* `Ctrl` + `N` or `Down` or `!` - History function scroll down.

## Register file concept

Register is a scalar variable known by its name and having associated
attributes. All registers together are called the register file. This is a
convenient way to access all parameters using a single mechanism. There is a
`reg` command to work with registers from the CLI. There are several ways to
call this command:

* Without arguments it will list all registers and their values.
* You can specify a pattern by which registers will be filtered. A pattern can
  be any part of the register name.
* If only one register matches the specified pattern the second parameter
  specifies its new value.
* You can specify a register number instead of its name to refer the exactly
  one register.

Something like that:

	(pmc) reg <pattern> <value>
	(pmc) reg <ID> <value>

Almost all of the configuration process is to change the value of the
registers.

You can also export all of configuration registers in plain text using a
`config_reg` command. The output of this command can be fed back into the CLI
to restore the configuration.

	(pmc) config_reg

To save the values of the configuration registers in the flash storage there is
a `flash_prog` command. Register values from the flash are loaded automatically
at startup.

	(pmc) flash_prog

Note the different types of registers. There are registers intended for saving
as configuration. Other registers provide information to read only. Virtual
registers provide a different way to access other registers (usually this is
taking a value in different unit of measurement). There are also link registers
that are required to configure data transfer between different subsystems.

Keep in mind each register can have its own write and read handler that can do
a complex non-obvious actions during access to it.

## Linkage concept

As we have already mentioned there are link registers. If we start accessing
such a register we will be redirected to the link. For example the analog input
module writes the control signal to register `ap.knob_reg_ID` but the value
falls into `pm.i_setpoint_current_pc`. You can configure `ap.knob_reg_ID`
to link it to another register if you want control another parameter. We
provide many registers in different units of measurement. You are free to
choose what to control current in Amperes or percentage from full scale or
something else.

There are telemetry module with 10 link registers. Choose any registers you
need to be captured.

	(pmc) reg tlm.reg_ID0 pm.tvm_A
	(pmc) reg tlm.reg_ID1 pm.tvm_B
	(pmc) reg tlm.reg_ID2 ...

Telemetry grab into RAM and flush textual dump.

	(pmc) tlm_grab <freq>
	(pmc) tlm_flush_sync

Or live telemetry printout.

	(pmc) tlm_live_sync <freq>

Using CAN data pipes you are able to link register across CAN network. You can
easily control many machines from single input. Build a traction control by
exchange the speed signals across PMC instances.

## Examples

Show all raw feedback values that PMC uses in control loops.

	(pmc) reg pm.fb

Show all machine constants.

	(pmc) reg pm.const

Enable control from analog input.

	(pmc) reg ap.knob_ENABLED 1

Look for speed setpoint registers.

	(pmc) reg pm.s_setpoint

Assign the value of 700 to the register with ID 377.

	(pmc) reg 377 700

## Basic commands

There are some commands that you can use to get started. Basic informational
commands.

	(pmc) ap_task_info
	(pmc) ap_heap_info

Get firmware version info.

	(pmc) ap_version

Manual PWM DC control for testing.

	(pmc) hal_PWM_set_DC <DC>

Show instant analog knob input voltages.

	(pmc) reg ap.knob_in_ANG
	(pmc) reg ap.knob_in_BRK

Enable the HX711 helper task.

	(pmc) reg ap.task_HX711 1

