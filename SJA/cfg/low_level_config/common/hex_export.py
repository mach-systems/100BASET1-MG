#! /usr/bin/env python
# -*- coding: iso-8859-1 -*-
########################################################################
# File name:       hex_export.py
# Author:          Petr Grillinger
# Revision:        $Revision: 1.10 $
# Last changed by: $Author: nxp30460 $
# Last chanded on: $Date: Fri Jul  1 11:30:45 2016 $
# Description:     Configuration file export for multiple objects
########################################################################

import sys
from array import array
from crc32 import crc32,CRC_CHECK_RESULT
from intelhex import IntelHex

CRC_INIT = 0xFFFFFFFF

def debug(msg):
    pass

def error(msg):
    sys.stderr.write(msg + "\n")

def warning(msg):
    sys.stderr.write(msg + "\n")

# Return a 32-bit integer separated into individual bytes
def to_list(i):
    return [i&0xFF, (i>>8)&0xFF, (i>>16)&0xFF, (i>>24)&0xFF]


# Store one or more configurations into a single Intel Hex-file
# configs is a list of objects that must support following functions:
#    get_tables(self)
#    format_for_output(self)
#    format_for_update(self)
def store_hex_file(device_id,                # 8-bit number
                   revision,                 # 24-bit number
                   configs,                  # list of configuration object
                   file_name="config.hex"):  # HEX file name + path

    # create intel-hex container
    ihex = IntelHex()
   
    # store the global header (device ID and revision)
    g_header = to_list((device_id << 24) | revision)
    ihex.extend(g_header)

    # store all configuration objects
    for config in configs:
        # switch to output format
        config.format_for_output()
        # store individual tables into ihex
        for map in config.get_tables():
            # get the table content as list of bytes
            blist  = map.store()

            # skip empty table
            if len(blist) == 0:
                continue

            # prepare header and CRC
            header = to_list((map.table.offset & 0xFF) << 24) + \
                     to_list(len(blist) / 4)
            h_crc  = to_list(crc32(header, CRC_INIT))
            d_crc  = to_list(crc32(blist, CRC_INIT))

            # store into hex
            ihex.extend(header)
            ihex.extend(h_crc)
            ihex.extend(blist)
            ihex.extend(d_crc)

        # switch to edit-format
        config.format_for_update()

    # append the termination record
    ihex.extend(to_list(0))   # 0 table ID
    ihex.extend(to_list(0))   # 0 table length
    
    # append the global CRC
    g_crc = crc32(ihex.tobinarray(), CRC_INIT)
    ihex.extend(to_list(g_crc))

    ihex.writefile("%s" % file_name, line_size=4)
    debug("Creating %s" % file_name)

# def store_hex_file


# Store one or more configurations into a single C-file
# configs is a list of objects that must support following functions:
#    get_tables(self)
#    format_for_output(self)
#    format_for_update(self)
def store_c_file(device_id,                # 8-bit number
                 revision,                 # 24-bit number
                 configs,                  # list of configuration object
                 file_name="config.c"):    # HEX file name + path
    data = []

    # store the global header (device ID and revision)
    g_header = to_list((device_id << 24) | revision)
    data.extend(g_header)

    # store all configuration objects
    for config in configs:
        # switch to output format
        config.format_for_output()
        # store individual tables into data
        for map in config.get_tables():
            # get the table content as list of bytes
            blist  = map.store()

            # skip empty table
            if len(blist) == 0:
                continue

            # prepare header and CRC
            header = to_list((map.table.offset & 0xFF) << 24) + \
                     to_list(len(blist) / 4)
            h_crc  = to_list(crc32(header, CRC_INIT))
            d_crc  = to_list(crc32(blist, CRC_INIT))

            # store into list
            data.extend(header)
            data.extend(h_crc)
            data.extend(blist)
            data.extend(d_crc)

        # switch to edit-format
        config.format_for_update()

    # append the termination record
    data.extend(to_list(0))   # 0 table ID
    data.extend(to_list(0))   # 0 table length
    
    # append the global CRC
    g_crc = crc32(data, CRC_INIT)
    data.extend(to_list(g_crc))

    debug("Creating %s" % file_name)
    fw = open(file_name, "w")
    fw.write("static const uint32_t CONFIG[] = {\n")
    for i in range(len(data)/4):
        val = data[4*i] + 0x100*data[4*i+1] + 0x10000*data[4*i+2] + 0x1000000*data[4*i+3]
        fw.write("    0x%08x,\n" % val)
    fw.write("};\n")
    fw.close()

# def store_c_file

