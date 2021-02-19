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
3. If only one register matches the specified pattern the second parameter
   specifies its new value.
4. You can specify a reqister number instead of its name to refer the exactly
   one register.

Something like that.

	# reg <pattern> <value>
	# reg <ID> <value>

Almost all of the configuration is to change the value of the registers.

You can also export configuration registers values in plain text using a
**config_export** command. The output of this command can be fed back into the
CLI to restore the configuration.

	# config_export

To save the values of the configuration registers in the flash there is a
**flash_prog** command. Register values from the flash are loaded automatically
at startup.

	# flash_prog

Note the different types of registers. There are registers intended for saving
as configuration. Other registers provide information to read only. Virtual
registers provide a different way to access other registers (usually this is
taking a value in other units). There are also link registers that are required
to configure data transfer between different subsystems.

Each register can have its own write and read handler that can do a complex non
obvious actions during access to it.

## Linkage concept

As we have already mentioned there are link registers. If we start accessing
such a register we will be redirected to the link. For example the analog input
module writes the control signal to register **ap.analog_reg_ID** but the value
falls into **pm.i_setpoint_torque_pc**. You can configure **ap.analog_reg_ID**
to link it to another register if you want control another parameter. We
provide many registers in different units of measurement. You are free to
choose what to control current in Amperes or percentage from full scale.

There are telemetry module with 10 link registers. Choose any registers you
need to be captured.

	# reg tlm.reg_ID_0 pm.tvm_A
	# reg tlm.reg_ID_1 pm.tvm_B
	# reg tlm.reg_ID_* ...

Telemetry grab (to fill the memory buffer) and flush textual dump.

	# tlm_grab <freq>
	# tlm_flush_sync

Or live telemetry printout.

	# tlm_live_sync <freq>

Using CAN data pipes you are able to link register across CAN network. You can
easily control many motors from single input. Build a traction control by
exchange the speed signals across motors.

## Examples

Show all raw feedback values that PMC uses in control loops.

	# reg pm.fb

Show all motor constants.

	# reg pm.const

Enable control from analog input.

	# reg ap.analog_ENABLED 1

Look for speed setpoint registers.

	# reg pm.s_setpoint

Access to the register by its number.

	# reg 377 700

## Basic commands

There are some commands that you can use to get started. Basic informational
commands.

	# rtos_uptime
	# rtos_cpu_usage

Get firmware size and crc32.

	# rtos_firmware

Manual PWM control for testing.

	# hal_PWM_set_DC <DC>
	# hal_PWM_set_Z <Z>

Show instant analog input values.

	# hal_ADC_get_analog_ANG
	# hal_ADC_get_analog_BRK

Start the HX711 helper application.

	# hx711_startup

