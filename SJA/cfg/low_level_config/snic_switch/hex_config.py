#! /usr/bin/env python
# -*- coding: iso-8859-1 -*-
########################################################################
# File name:       hex_config.py
# Author:          Petr Grillinger
# Revision:        $Revision: 1.10 $
# Last changed by: $Author: nxp30460 $
# Last chanded on: $Date: Fri Jul  1 11:30:45 2016 $
# Description:     Configuration file generation for TTGbE
########################################################################

# add the search path for parent package
import snic_switch
if len(snic_switch.__path__) < 2:
    snic_switch.__path__.append(snic_switch.__path__[0] + '\\..')

#from types import *
from array import array
import sys
from common.crc32 import crc32,CRC_CHECK_RESULT
from common.intelhex import IntelHex
import common.hex_config

CRC_INIT = 0xFFFFFFFF

def debug(msg):
    pass

def error(msg):
    sys.stderr.write(msg + "\n")

warning = error


def read32(ba, i):
    return long(ba[i]) | long(ba[i+1])<<8 | long(ba[i+2])<<16 | long(ba[i+3])<<24

# Return a 32-bit integer separated into individual bytes
def to_list(i):
    return [i&0xFF, (i>>8)&0xFF, (i>>16)&0xFF, (i>>24)&0xFF]


# Support for a configuration stored in a HEX file as
# defined in the TTGbE specification (separate tables with
# variable length and a CRC check)
class Hex_Config(common.hex_config.Hex_Config):

    # Create an empty configuration
    # You have to fill the parameters manualy
    def __init__(self):
        common.hex_config.Hex_Config.__init__(self)
    # def __init__

    # Load the contents of a HEX file into local configuration dictionaries
    # according to the SNIC specification. The individual tables are parsed
    # with functions defined in self.table_map
    def load_nvm_file(self, file_name, altera=False, start_addr=None):
        # Load the HEX file into array
        ba   = IntelHex(file_name, altera=altera).tobinarray()
        i    = 0

        # load and check the global header
        dev_id   = read32(ba, i) >> 24
        revision = read32(ba, i) & 0xFFFFFF
        i       += 4

        if self.gp.DEVICE_ID != dev_id:
            error("Device ID mismatch: file says 0x%02x, expected 0x%02x\n" % (dev_id, self.gp.DEVICE_ID))
            return False

        if self.gp.CFG_FORMAT_REV != revision:
            warning("Revision mismatch: file says %u, expected %u\n" % (revision, self.gp.CFG_FORMAT_REV))


        # parse the complete file, stop at termination record (length 0)
        while i < len(ba):
            # load block header
            table_i    = read32(ba, i) >> 24
            data_len   = read32(ba, i+4) * 4

            # 0-length termination record
            if data_len == 0:
                # check global CRC
                if crc32(ba[0:i+12], CRC_INIT) != CRC_CHECK_RESULT:
                    warning("Global CRC mismatch at byte %u" % i)
                break

            # check header CRC
            if crc32(ba[i:i+12], CRC_INIT) != CRC_CHECK_RESULT:
                error("CRC mismatch in header at byte %u" % i)

            # find table with matching offset / index
            map       = filter(lambda x: (x.table.offset&0xFF) == table_i, self.table_map)
            if len(map) != 1:
                error("Table ID %u is invalid; Found %u possible matches.\n" % (table_i, len(map)))
                return False
            map       = map[0]

            # move to data start (skip header and header CRC)
            i += 12

            # verify data CRC
            if crc32(ba[i:i+data_len+4], CRC_INIT) != CRC_CHECK_RESULT:
                error("CRC mismatch in data at byte %u" % i)

            # decode the block using the table-specific function
            debug("Loading table %s, ID %d, %d bytes" % (map.table.name, table_i, data_len))
            if not map.load(blist = ba, start=i, bsize=data_len):
                error("Loading of table '%s' failed" % map.table.name)
                return False

            # move to next header (skip data CRC)
            i += data_len + 4

        self.finish_loading()

        return True
    # def load_nvm_file


    # Store the complete contents into a HEX file
    def create_nvm_file(self,
                        file_name="ecfg_nvm.hex", # HEX file name + path
                        b_device_id_no_tt=False,  # device_id_no_tt flag
                        n_device_id_no_tt=0,      # device_id_no_tt value
                        altera=False,             # Create an Altera-compatible HEX file (not Intel-HEX)
                        write_file=True,
                        block_ids_crc_err=[],     # list of block IDs for which the CRC will be generated wrong
                        block_ids_alter=[]):      # list of block IDs to be altered

        self.format_for_output()

        # Store the data into a HEX file
        if not hasattr (self, "ihex") :
            self.ihex = IntelHex(altera=altera)
       
        # store the global header (device ID and revision)
        if b_device_id_no_tt :
            g_header = to_list((n_device_id_no_tt << 24) | (self.gp.AHB_INTERFACE_REV << 8) | self.gp.CFG_FORMAT_REV)
            print "g_header %08x" % ((n_device_id_no_tt << 24) | (self.gp.AHB_INTERFACE_REV << 8) | self.gp.CFG_FORMAT_REV)
        else :
            g_header = to_list((self.gp.DEVICE_ID << 24) | (self.gp.AHB_INTERFACE_REV << 8) | self.gp.CFG_FORMAT_REV)
            print "g_header %08x" % ((self.gp.DEVICE_ID << 24) | (self.gp.AHB_INTERFACE_REV << 8) | self.gp.CFG_FORMAT_REV)
        self.ihex.extend(g_header)

        # Prepare the complete configuration as an exact configuration area image
        for map in self.table_map:
            # get the table content as list of bytes
            blist  = map.store()

            # skip empty table
            if len(blist) == 0:
                continue
    
            # prepare header and CRC
            if map.table.offset in block_ids_alter :
                header = to_list(((map.table.offset+128+64) & 0xFF) << 24) + \
                           to_list(len(blist) / 4)
                print "Altered block ID %d to %d" % (map.table.offset, map.table.offset + 128 + 64)
            else :
                header = to_list((map.table.offset & 0xFF) << 24) + \
                        to_list(len(blist) / 4)
            h_crc  = to_list(crc32(header, CRC_INIT))
            if map.table.offset in block_ids_crc_err :
                d_crc  = to_list(0x55555555)
                print "Altered block ID %d CRC" % map.table.offset
            else :
                d_crc  = to_list(crc32(blist, CRC_INIT))

            # store into hex
            self.ihex.extend(header)
            self.ihex.extend(h_crc)
            self.ihex.extend(blist)
            self.ihex.extend(d_crc)

        # store a termination record + global CRC
        self.ihex.extend(to_list(0))   # 0 table ID
        self.ihex.extend(to_list(0))   # 0 table length
        g_crc = crc32(self.ihex.tobinarray(), CRC_INIT)
        self.ihex.extend(to_list(g_crc))

        if write_file : # only write to HEX file when `write_file` is True
            self.ihex.writefile("%s" % file_name, line_size=4)
            delattr (self, "ihex")
            debug("Creating %s" % file_name)

        self.format_for_update()
    # def create_nvm_file

# class Hex_Config
