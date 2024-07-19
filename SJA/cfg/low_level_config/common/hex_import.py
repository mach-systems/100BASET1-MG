#! /usr/bin/env python
# -*- coding: iso-8859-1 -*-
########################################################################
# File name:       hex_import.py
# Author:          Petr Grillinger
# Revision:        $Revision: 1.10 $
# Last changed by: $Author: nxp30460 $
# Last chanded on: $Date: Fri Jul  1 11:30:45 2016 $
# Description:     Configuration file loading for multiple objects
########################################################################

import sys
from array import array
from crc32 import crc32,CRC_CHECK_RESULT
from intelhex import IntelHex

CRC_INIT = 0xFFFFFFFF

def debug(msg):
    pass

def warning(msg):
    sys.stderr.write(msg + "\n")

def error(msg):
    sys.stderr.write(msg + "\n")


def read32(ba, i):
    return long(ba[i]) | long(ba[i+1])<<8 | long(ba[i+2])<<16 | long(ba[i+3])<<24


# Load one or more configurations from an Intel Hex-file
# configs is a list of objects that must support following functions:
#    load_data_block(self, block_id, block_size, data)
#    clear(self)
#    finish_loading(self)
def load_hex_file(file_name, configs):
    # Load the HEX file into a byte array
    ba   = IntelHex(file_name, altera=altera).tobinarray()
    i    = 0

    # load the configuration header
    dev_id   = read32(ba, i) >> 24
    revision = read32(ba, i) & 0xFFFFFF
    i       += 4
    debug("Configuration device ID: %d, revision %d" % (dev_id, revision))
    
    # initialize the configurations
    for config in configs:
        config.clear()

    # parse the complete file, stop at termination record (length 0)
    while i < len(ba):
        # load block header
        block_id  = read32(ba, i) >> 24
        block_len = read32(ba, i+4) * 4
        
        # 0-length data indicates a termination record
        if block_len == 0:
            # check global CRC
            if crc32(ba[0:i+12], CRC_INIT) != CRC_CHECK_RESULT:
                warning("Global CRC mismatch at byte %u" % i)
            break

        # check header CRC
        if crc32(ba[i:i+12], CRC_INIT) != CRC_CHECK_RESULT:
            warning("CRC mismatch in header at byte %u" % i)

        # move to data start (skip header and header CRC)
        i += 12

        # verify data CRC
        if crc32(ba[i:i+block_len+4], CRC_INIT) != CRC_CHECK_RESULT:
            error("CRC mismatch in data at byte %u" % i)

        # pass to all configuration containers
        for config in configs:
            config.load_data_block(block_id=block_id, block_len=block_len, data=ba[i:i+block_len])

        # move to next header (skip data CRC)
        i += block_len + 4

    # notify the configuration containers that we are done
    for config in configs:
        config.finish_loading()

    # return the Device ID and revision number
    return {"device id" : device_id, "revision" : revision}
# def load_hex_file

