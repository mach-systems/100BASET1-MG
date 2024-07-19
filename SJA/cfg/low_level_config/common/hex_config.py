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

import sys
from types import *


def debug(msg):
    pass

def error(msg):
    sys.stderr.write(msg + "\n")

warning = error


# find the best block size and run-length encoding
# return (block_size, repeat_count)
def find_run_length_block(ba, block_size=16, index=0):
    # find best encoding for repeated patterns
    max_len     = 0
    saved_len   = -1
    max_gran    = 0
    for gran in range(4, block_size+4, 4):
        pattern = ba[0:gran]
        i       = gran
        while ba[i:i+gran] == pattern:
            i  += gran
        if i - gran > saved_len:
            max_len   = i
            max_gran  = gran
            saved_len = i - gran

    # block count > 1
    if max_len / max_gran > 1:
        return (max_gran, max_len / max_gran)

    # no compression, use larger blocks
    return (min(block_size, len(ba)), 1)


# Convert a dictionary to string
def dict_to_str(d, indent="", fmt="%-44s : %s"):
    s = ""
    for key in sorted(d.iterkeys()):
        val = d[key]
        s += indent
        s += fmt % (key, any_to_str(val, indent, "\n"))
        s += "\n"
    return s

# Convert a list or a tuple to string
def list_to_str(l, indent="", fmt="%04d : %s"):
    s = ""
    for i, val in enumerate(l):
        s += indent
        s += fmt % (i, any_to_str(val, indent, "\n"))
        s += "\n"
    return s

# Convert any hierarchy of list and dictionaries into a string
def any_to_str(item, indent="", eol=""):
    try:
        if type(item) == DictType:
            return eol+dict_to_str(item, indent+"    ")
        elif type(item) in (ListType,TupleType):
            return eol+list_to_str(item, indent+"    ")
        else:
            return str(item)
    except:
        return str(type(item))


# Use this class to specify which configuration tables are part
# of the configuration file and how to store/load them
class Hex_Table(object):
    def __init__(self, table, load_fn=None, store_fn=None, data=None, file_name=None):
        object.__init__(self)
        self.table     = table
        self.layout    = table.layout
        self.file_name = file_name
        if data != None:
            self.data  = data
        else:
            self.data  = self
        if load_fn:
            self.load  = load_fn
        if store_fn:
            self.store = store_fn

    def finish_loading(self, config):
        return True
### class Hex_Table



# Support for a configuration stored in a HEX file as
# defined in the TTGbE specification (separate tables with
# variable length and a CRC check)
class Hex_Config(object):

    # Create an empty configuration
    # You have to fill the parameters manualy
    def __init__(self):
        object.__init__(self)
        # this should be a list of Hex_Table instances
        self.table_map = []
    # def __init__

    # if any of the parameters must by computed internally before
    # the output, override this function
    def compute_derived_params(self):
        pass
    # def compute_derived_params

    # make any changes necessary to be able to update the configuration
    # via direct access to the tables
    def format_for_update(self):
        pass
    # def format_for_update


    # make any changes necessary to be able to store the configuration
    # in a configuration file
    def format_for_output(self):
        pass
    # def format_for_output


    # called when all configuration tables have been loaded from a HEX file
    # to do final post-processing
    def finish_loading(self):
        self.format_for_update()
    # def finish_loading


    # Load the contents of a HEX file into local configuration dictionaries
    def load_nvm_file(self, file_name, altera=False, start_addr=True):
        return True  # success
    # def load_nvm_file


    # Store the complete contents into a single HEX file
    # the HEX file object should remain stored in self.ihex
    def create_nvm_file(self,
                        file_name="ecfg_nvm.hex", # HEX file name + path
                        altera=False,             # Create an Altera-compatible HEX file (not Intel-HEX)
                        block_crc_error=None,     # Compute a wrong block CRC value for configuration block n (0 = block 0, 1 = block 1, etc)
                        global_crc_error=False,   # Compute a wrong global CRC value
                        start_record=True,        # Add a start record for the boot loader
                        start_ofs=0x800,          # Specify configuration start address (byte offset)
                        add_query_table=False,
                        write_file=True):         # If False the configs are accumulated internally and only written to file when this param is True  
        pass
    # def create_nvm_file


    # Export the configuration tables into individual HEX files
    def create_sim_files(self, altera=False):
        pass
    # def create_sim_files


    # Store the complete contents into a C file as a data array
    def create_c_file(self, file_name="config.c", var_name="CONFIG", compression=False):

        ofile = open(file_name, "w")
        ofile.write("static const unsigned char %s[] = {\n" % var_name)
        
        # create HEX file internally
        self.create_nvm_file(write_file=False)

        ba = self.ihex.tobinarray(padding=False)

        # run-length encoded data structure
        if compression:
            ofile.write("   /* Sequence of blocks formed by a 32-bit header and a data part. The header is \n"
                        "   in the form 'NNNNCCCC', where NNNN is the number of bytes in the data part and\n"
                        "   CCCC is the number of repetions of the data part in the original data stream. */\n")
            base = 0
            while base < len(ba):
                (blen,bcount) = find_run_length_block(ba[base:], index=base)
                ofile.write("   0x%02x, 0x%02x, 0x%02x, 0x%02x, /* block length %u, count %u */\n" % (blen&0xFF, blen>>8, bcount&0xFF, bcount>>8, blen, bcount))
                for i in range(base, base+blen, 4):
                    ofile.write("   0x%02x, 0x%02x, 0x%02x, 0x%02x,\n" % tuple(ba[i:i+4]))
                base += bcount * blen
        else:
            # simple C data structure
            for i in range(len(ba) / 4):
                ofile.write("   0x%02x, 0x%02x, 0x%02x, 0x%02x,\n" % tuple(ba[i*4:i*4+4]))

        ofile.write("};\n");
        ofile.close()
    # def create_c_file


    # Return a string representation of the internal data
    def __str__(self):
        s = 60*"-" + "\n"
        for tm in self.table_map:
            s += tm.table.name + "\n"
            s += any_to_str(tm.data)
            s += 60*"-" + "\n"
        return s
    # def __str__

# class Hex_Config
