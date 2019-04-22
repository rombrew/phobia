## Overview

This page introduces you to the Command Line Interface (CLI). We have a regular
CLI with autocompletion and command history.

## Register file concept

Register is a scalar variable known by its name and having associated
attributes. All registers together are called a register file. This is a
convenient way to access all parameters using a single mechanism. There is a
**reg** command to work with registers from the CLI. There are several ways to
call this command.

1. Without arguments it will list all registers and their values.
2. You can specify a pattern by which registers will be filtered. A pattern can
   be any part of the register name.
3. If only one register matches the specified pattern, the second parameter
   specifies its new value.
4. You can specify a reqister number instead of its name to refer to exactly
   one register.

Something like that.

	# reg <pattern> <value>
	# reg <ID> <value>

Almost all of the configuration is to change the value of the registers.

You can also export configuration registers values in plain form using a
**reg_export** command. The output of this command can be fed back into the CLI
to restore the configuration.

To save the values of the configuration registers in the flash there is a
**flash_write** command. Register values from the flash are loaded
automatically at startup.

Note the different types of registers. There are registers intended for saving
as configuration. Other registers provide information to read only. Virtual
registers provide a different way to access other registers (usually this is
taking a value in other units). There are also registers-pointers that are
required to configure data transfer between different subsystems.

Each register can have its own write and read handlers, thus performing complex
non-obvious actions during access to it.

## Basic commands

Basic informational commands.

	# rtos_uptime
	# rtos_cpu_usage

Manual PWM control for testing.

	# hal_PWM_set_DC <DC>
	# hal_PWM_set_Z <Z>

Telemetry grab (to fill the memory buffer) and flush.

	# tel_grab <freq>
	# tel_flush_sync

Live telemetry printout.

	# tel_live_sync

Start the HX711 helper application.

	# ap_hx711_startup

