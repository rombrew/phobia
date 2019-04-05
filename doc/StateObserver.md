## Overview

This page describes some details of sensorless algorithm used in PMC.

## Problem statement

	            Timeline diagram of one PWM cycle
	
	         |<--------------------- dT ---------------------->|
	         |                                                 |
	         |  TIM update     +----+---+----+                 |
	PWM      | /               |    |   |    |                 |
	waveform |/                |    |   |    |                 |
	       --*--*--*-----------------------------+-------------*--
	         |  |  |                             |             |
	ADC      iA uS uB          preload DC        |
	samples  iB uA uC           to hw timer ->  TIM
	                 \                          /
	MCU               pm_feedback()            /
	routines                p_set_DC(xA, xB, xC)

## Basics

