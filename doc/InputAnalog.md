## Overview

This page describes the configuration of analog input interface.

## Hardware

The analog signal (from 0 to 5v) is fed to ANG pin.

	                                +------------+
	            +---+---+           |            |
	            |   |   |          | | (analog)  |
	(brake)     | [=/   |          | |<------+   |
	            |   |   |          |_|       |   |
	(reverse) [=/   |   |           |        |   |
		    |   |   |           +----+   |   |
	            |   |   |                |   |   |
	+-----------|---|---|----------------|---|---|---+
	|          PPM SDA GND              +5v ANG GND  |

## Configuration


