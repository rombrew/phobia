## Overview

This page introduces you to the Command Line Interface (CLI). We have a regular
CLI with autocompletion function and command history.

## Key mapping

These are the basic special keys that are used in the CLI:

- `Return` - Run the command from current line.
- `Backspace` or `Delete` - Erase last typed character.
- `Tab` or (@) - Automplete function.
- `Shift` + `Tab` - Automplete function reverse.
- `Ctrl` + `C` or `Ctrl` + `D` - Drop the content of the line or abort the command.
- `Ctrl` + `P` or `Up` or `*` - History function scroll up.
- `Ctrl` + `N` or `Down` or `!` - History function scroll down.

## Register file

A register is a scalar variable known by its name and having associated
attributes. All registers together are called the register file. This is a
convenient way to access all parameters in one way. There is a `reg` command to
work with registers from the CLI. There are several ways to call this
command.

- Without arguments it will list all registers and their current values.
- You can specify a pattern by which registers will be filtered. A pattern can
  be any part of the register name.
- If only one register matches the specified pattern the second parameter can
  specify its new value.
- You can specify a register number instead of its name to refer the exactly
  one register.

Something like that:

	(pmc) reg <pattern> <value>
	(pmc) reg <ID> <value>

Almost all of the configuration process is to review and change the value of
the registers.

You can also export all of configuration registers in plain text using a
`config_reg` command. The output of this command can be fed back into the CLI
to restore the configuration.

	(pmc) config_reg

To save the values of the configuration registers in the flash storage there is
a `flash_prog` command. Register values from the flash are loaded automatically
at startup.

	(pmc) flash_prog

If you need to cleanup the flash storage do not forget to reboot PMC after.

	(pmc) flash_wipe
	(pmc) ap_reboot

Note the different types of registers. There are registers intended to storage
of configuration. Other registers provide some read-only information. Virtual
registers provide a different way to access already known registers (usually
this is taking a value in different unit of measurement). There are also link
registers that are required to configure data transfer between different
subsystems.

Keep in mind each register can have its own write and read event handler that
can do any non-obvious actions during access to it.

## Linkage concept

As we have already mentioned there are link registers. If we start accessing
such a register we will be redirected to the link. For example the analog input
module writes the control signal to register `ap.knob_reg_ID` but the value
falls into `pm.i_setpoint_current_pc`. You can configure `ap.knob_reg_ID` to
link it to another register if you want control another parameter. We provide
many registers in different units of measurement. You are free to choose how to
control current in Amperes or percentage from full scale or something else.

There are telemetry module with 10 link registers. Choose any registers you
need to be captured.

	(pmc) reg tlm.reg_ID0 pm.tvm_A
	(pmc) reg tlm.reg_ID1 pm.tvm_B
	(pmc) reg tlm.reg_ID2 ...

Command to grab telemetry into RAM and flush textual dump.

	(pmc) tlm_grab <rate>
	(pmc) tlm_flush_sync

Run in endless loop of grabbing until PMC stops with error.

	(pmc) tlm_watch <rate>

Use a real-time telemetry printout.

	(pmc) tlm_live_sync <rate>

Using CAN data pipes you are able to link register across CAN network. You can
easily control many machines from single input. Build a traction control by
exchange the speed signals across PMC nodes.

## Textual transcription

Some of integer registers printed out accompanied by textual transcription that
describes register current value. Typically this is a configuration register
responsible for selecting one of several possible options. The textual
transcription corresponds to the enumeration in the source code of PMC.

	(pmc) reg pm.config_IFB
	1 [151] pm.config_IFB = 2 (PM_IFB_ABC_INLINE)

## Examples

Show all raw feedback values that PMC uses in control loops.

	(pmc) reg pm.fb

Show all machine constants.

	(pmc) reg pm.const

Enable control from analog input.

	(pmc) reg ap.knob_ENABLED 1

Show all speed setpoint registers.

	(pmc) reg pm.s_setpoint

Assign the value of 700 to the register with ID 377.

	(pmc) reg 377 700

## Basic commands

Basic informational commands.

	(pmc) ap_dbg_task
	(pmc) ap_dbg_heap

Get firmware version info.

	(pmc) ap_version

Manual PWM DC control for testing.

	(pmc) hal_PWM_set_DC <DC>

Show instant analog knob input voltages.

	(pmc) reg ap.knob_in_ANG
	(pmc) reg ap.knob_in_BRK

Enable the HX711 helper task.

	(pmc) reg ap.task_HX711 1

