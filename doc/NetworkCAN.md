## Overview

This page describes CAN network architecture adopted in PMC. An overview of the
networking functions.

## Hardware

TODO

conventional two wire bus
uses a linear bus terminated at each end

	       +-------------------------*------------------~~~
	       |                         |
	       |      +------------------|------*-----------~~~
	       |      |                  |      |
	+------+------+------+    +------+------+------+
	|    CAN_H  CAN_L    |    |    CAN_H  CAN_L    |    ...
	|                    |    |                    |
	|    PMC (node 1)    |    |    PMC (node 2)    |

## Protocol

We use base frame format with 11-bit identifier.

	+---------------+-------------+------------------------+
	|  ID (11-bit)  |  N (4-bit)  |    payload (64-bit)    |
	+---------------+-------------+------------------------+

## Network survey

## IO forwarding

## Flexible data pipes

TODO

