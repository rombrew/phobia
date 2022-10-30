#!/usr/bin/python

import os, re

def mkdefs(hw):

    f = open('hal/hw/' + hw + '.h', 'r')
    g = open('hal/mk/' + hw + '.d', 'w')

    for s in f:
        if   'HW_MCU_STM32F405' in s:
            g.write('HWMCU = STM32F405\n')
        elif 'HW_MCU_STM32F722' in s:
            g.write('HWMCU = STM32F722\n')

        elif 'HW_HAVE_PART_DRV8303' in s:
            g.write('INCLUDE_HAL_DRV = hal/drv.o\n')
        elif 'HW_HAVE_PART_DRV8305' in s:
            g.write('INCLUDE_HAL_DRV = hal/drv.o\n')

        elif 'HW_HAVE_USB_OTG_FS' in s:
            g.write('INCLUDE_HAL_USB = hal/usb.o\n');
            g.write('INCLUDE_CHERRY = cherry/usb_dc_dwc2.o \\\n');
            g.write('                 cherry/usbd_cdc.o \\\n');
            g.write('                 cherry/usbd_core.o\n');

        elif 'HW_HAVE_NETWORK_EPCAN' in s:
            g.write('INCLUDE_HAL_CAN = hal/can.o\n')
            g.write('INCLUDE_EPCAN = epcan.o\n')

        elif 'HW_HAVE_NETWORK_DRONECAN' in s:
            g.write('INCLUDE_HAL_CAN = hal/can.o\n')
            g.write('INCLUDE_LIBCANARD = libcanard/canard.o\n')
            g.write('INCLUDE_CAN = dronecan.o\n')

    f.close()
    g.close()

def mkbuild():

    for file in os.listdir('hal/hw'):
        if file.endswith(".h"):
            mkdefs(file[0:-2])

def checkmacro(s, m):
    m = re.search('^\s*' + m + '\(.+\)', s)
    return True if m != None else False

def shdefs(file):

    f = open(file, 'r')
    g = open('shdefs.h', 'a')

    if file.endswith('epcan.c'):
        g.write('#ifdef HW_HAVE_NETWORK_EPCAN\n')

    for s in f:
        if checkmacro(s, 'SH_DEF'):
            m = re.search('\(\w+\)', s).group(0)
            s = re.sub('[\s\(\)]', '', m);
            g.write('SH_DEF(' + s + ')\n')

    if file.endswith('epcan.c'):
        g.write('#endif /* HW_HAVE_NETWORK_EPCAN */\n')

    f.close()
    g.close()

def shbuild():

    g = open('shdefs.h', 'w')
    g.close();

    for path in ['./', 'apps/', 'hal/']:
        for file in os.listdir(path):
            if file.endswith(".c"):
                shdefs(path + file)

def regdefs():

    distance = 10

    f = open('regfile.c', 'r')
    g = open('regdefs.h', 'w')

    for s in f:
        if s[0] == '#':
            if distance < 4:
                g.write(s)
        elif checkmacro(s, 'REG_DEF'):
            s = re.search('\([\w\.]+?,\s*[\w\.]*?,', s).group(0)
            s = re.sub('[\(\,\s]', '', s).replace('.', '_');
            g.write('ID_' + s.upper() + ',\n')
            distance = 0

        distance += 1

    f.close()
    g.close()

mkbuild()
shbuild()
regdefs()

