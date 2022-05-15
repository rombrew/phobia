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
        elif 'HW_HAVE_NETWORK_CAN' in s:
            g.write('INCLUDE_HAL_CAN = hal/can.o\n')
            g.write('INCLUDE_IFCAN = ifcan.o\n')

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

    if file.endswith('ifcan.c'):
        g.write('#ifdef HW_HAVE_NETWORK_CAN\n')

    for s in f:
        if checkmacro(s, 'SH_DEF'):
            m = re.search('\(\w+\)', s).group(0)
            s = re.sub('[\s\(\)]', '', m);
            g.write('SH_DEF(' + s + ')\n')

    if file.endswith('ifcan.c'):
        g.write('#endif /* HW_HAVE_NETWORK_CAN */\n')

    f.close()
    g.close()

def shbuild():

    g = open('shdefs.h', 'w')
    g.close();

    for path in ['./', 'apps/']:
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

