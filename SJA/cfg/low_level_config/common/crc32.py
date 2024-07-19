#! /usr/bin/env python
# -*- coding: iso-8859-1 -*-
########################################################################
# File name:       crc32.py
# Author:          Petr Grillinger
# Revision:        $Revision: 1.10 $
# Last changed by: $Author: nxp30460 $
# Last chanded on: $Date: Fri Jul  1 11:30:45 2016 $
# Description:     CRC-32 computation (Ethernet CRC compatible)
########################################################################

from array import array

ETH_POLY         = 0x04C11DB7  # 0000 0100 1100 0001 0001 1101 1011 0111
#reflected poly  # 0xEDB88320  # 1110 1101 1011 1000 1000 0011 0010 0000
CRC_CHECK_RESULT = 0x2144df1c  # expected result after the checksum is appended to the data stream
crctab_poly      = None


def reflect(num, width=32):
    ret = 0L
    for i in range(width):
        ret   = (ret << 1) | (num & 1)
        num >>= 1
    return ret
### reflect


# create a CRC lookup table used by table algorithms
def make_crc_table(poly=ETH_POLY):
    global crctab, crctab_poly
    crctab_poly = poly
    ref_poly    = reflect(poly, 32)
    crctab      = array("L")

    for i in range(256):
        crc = long(i)
        for j in range(8):
            if crc & 1: 
                crc = ref_poly ^ (crc >> 1)
            else:
                crc = crc >> 1
        crctab.append(crc)
### def make_crc_table


def crc32_byte(crc, byte):
    i = (crc ^ byte) & 0xFF
    return crctab[i] ^ (crc >> 8)
### def crc32_byte


def crc32(data, init=0xFFFFFFFF, poly=ETH_POLY):
    if crctab_poly != poly:
        make_crc_table(poly)

    crc = init
    for byte in data:
        i   = (crc ^ byte) & 0xFF
        crc = crctab[i] ^ (crc >> 8)

    return crc ^ 0xFFFFFFFF
### def crc32

