#! /usr/bin/env python
# -*- coding: iso-8859-1 -*-
########################################################################
# File name:       bit_table.py
# Author:          Petr Grillinger
# Revision:        $Revision: 1.10 $
# Last changed by: $Author: nxp30460 $
# Last chanded on: $Date: Fri Jul  1 11:30:45 2016 $
# Description:     Classes that support configuration table description
########################################################################

from copy import copy
from types import *
from debug import debug,error,warning


# Container for a single layout element
class Layout_Entry (object):
    # Choices can by:
    #  - dictionary: key is the binary form, value is the string representation
    #  - list/tuple of strings: binary representation is the index of the string in the tuple
    #  - any number except 0: numerical base (2, 8, 10, 16)
    #  - anything that evaluates for 'False': direct binary representation
    def __init__(self, name=None, bit_len=32, count=1, comment=None, const=None, choices=None, default=None, align=1):
        self.name     = name
        self.bit_len  = bit_len
        self.count    = count
        self.comment  = comment  # optional description
        self.const    = const
        self.bit_ofs  = None     # used by Layout
        self.default  = default
        self.align    = align    # bit alignment
        if choices == None and bit_len == 1:
            self.choices = ("No", "Yes")
        else:
            self.choices = choices

    def __str__ (self) :
        return "%s %d %d" %(self.name,self.bit_ofs,self.bit_len)

    @property
    def end_ofs (self):
        return self.bit_ofs + self.bit_len - 1

# end class Layout_Entry


# Create a layout element that is aligned to an even 32-bit address
def Layout_Entry32(name=None, bit_len=32, count=1, comment=None, const=None, choices=None, default=None):
    return Layout_Entry(name, bit_len, count, comment, const, choices, default, align=32)


# A sequence of layout elements that allows conversion to binary format
class Layout (object):

    # Create a layout container from a list of Layout_Entry instances
    def __init__(self, layout, name="None", align=32, pad_low=False):
        self.layout  = layout
        self.name    = name
        self.align   = align
        self.pad_low = pad_low
        # build a dictionary for faster name lookup
        # also compute the start offsets
        self.str_lookup = {}
        self.num_lookup = []
        self.bit_len    = 0
        for le in layout:
            for i in range(le.count):
                # make sure the element is properly aligned
                rem = self.bit_len % le.align
                if rem != 0:
                    self.bit_len += le.align - rem
                # store its start offset
                le.bit_ofs           = self.bit_len;
                self.bit_len        += le.bit_len
                # store an entry in the lookup table
                if le.name != None:
                    if le.count == 1:
                        le.full_name = le.name
                        lookup_link  = le
                    else:
                        lookup_link = copy(le)
                        lookup_link.full_name = le.name + "[%d]"%i
                    self.str_lookup[lookup_link.full_name] = lookup_link
                    self.num_lookup.append(lookup_link)
        # compute the total size in 8/32-bit words
        self.pad_bits = (align - (self.bit_len % align)) % align
        self.byte_len = (self.bit_len + self.pad_bits + 7 - 1) / 8
        # insert padding from lower bits by shifting all elements up
        if pad_low:
            for le in self.num_lookup:
                le.bit_ofs += self.pad_bits
    # def __init__


    def add_padding(self, bits):
        # insert padding from lower bits by shifting all elements up
        if self.pad_low:
            for le in self.num_lookup:
                le.bit_ofs += bits
        self.pad_bits += bits
        self.byte_len = (self.bit_len + self.pad_bits + 7 - 1) / 8
    # def add_padding


    def __str__(self):
        return self.name


    def __contains__(self, name):
        return name in self.str_lookup


    def __getitem__(self, key):
        if type(key) == StringType:
            return self.str_lookup[key]
        elif type(key) == IntType:
            return self.num_lookup[key]
        else:
            assert False


    def __len__(self):
        return len(self.num_lookup)


    # translate a single layout element into the integer array
    # le is the layout element (Layout_Entry)
    # val is an integer value
    # ret is the integer array that will contain the result
    def tr(self, le, val, ret):
        if val >= 2**le.bit_len:
            warning("Value %u is too large for '%s.%s'" % (val, self.name, le.name))
        # store the first byte (might not start at bit 0)
        bits_left  = le.bit_len
        w          = le.bit_ofs / 8
        bs         = le.bit_ofs % 8
        blen       = min(8-bs, bits_left) # do not cross the byte boundary
        mask       = 2**blen - 1
        ret[w]    |= (val & mask) << bs
        val      >>= blen
        bits_left -= blen
        # check if there are more words to write
        while bits_left > 0:
            w         += 1
            blen       = min(8, bits_left)  # do not cross the byte boundary
            mask       = 2**blen - 1
            ret[w]    |= val & mask
            val      >>= blen
            bits_left -= blen
    # def tr


    # extract and return a dictionary of "name": value pairs from
    # the provided list of bytes (or a byte array)
    def to_dict(self, blist):
        # convert the byte list into a single long integer
        long_val = 0L
        for b in reversed(blist):
            long_val = (long_val << 8) | b
        # parse the layout and create dictionary entries
        ret     = {}
        bit_pos = 0
        for _le in self.layout:
            for i in range(_le.count):
                # get the proper layout element in case there is a vector
                if _le.name == None:
                    continue
                elif _le.count > 1:
                    name = "%s[%d]" % (_le.name, i)
                    le   = self.str_lookup[name]
                else:
                    name = _le.name
                    le   = _le

                # drop any bits below this field
                if bit_pos < le.bit_ofs:
                    long_val >>= le.bit_ofs - bit_pos
                    bit_pos    = le.bit_ofs
                # extract the field value
                mask = 2**le.bit_len - 1
                val  = long_val & mask
                # store the result
                ret[name] = val
                # check if a constant field is valid
                if le.const != None and le.const != val:
                    warning("Field '%s' should be %d but is %d" % (le.name, le.const, val))
        ret["_type"] = self
        return ret
    # def extract


    # convert data in a dictionary form into a string
    def to_string(self, data):
        s = ""
        for key, val in data.iteritems():
            if key in self.str_lookup and key[0] != "_" and self.str_lookup[key].const == None:
                s += ", %s=%s" % (key, val)
        return s[2:]
    # def to_string


    # convert data in a dictionary form into an XML format
    def to_xml(self, data, indent=""):
         s  = indent + "<record>\n"
         s += indent + indent + "<layout>%s</layout>\n" % (self.name,)
         for key, val in data.iteritems():
            if key in self.str_lookup and key[0] != "_": #and self.str_lookup[key].const == None:
               s += indent + indent + "<item>\n"
               s += indent + indent + indent + "<name>%s</name>\n" % (key,)
               s += indent + indent + indent + "<value>%s</value>\n" % (val,)
               s += indent + indent + "</item>\n"
         for key in self.str_lookup:
            if key not in data:
               value = self.str_lookup[key].default
               if value == None:
                  value = 0
               #s += indent + "  <key name=%s value=%s comment=default></key>\n" % (key, value)
               s += indent + indent + "<item>\n"
               s += indent + indent + indent + "<name>%s</name>\n" % (key,)
               s += indent + indent + indent + "<value>%s</value>\n" % (value,)
               s += indent + indent + "</item>\n"
         s = s + indent + "</record>\n"
         return s
    # def to_xml


    # produce a sequence of bytes from a data dictionary
    # lower bit positions are placed in lower bytes
    # data must be a dictionary {"name": value}
    def to_bin(self, data):
        # prepare an empty container
        ret = self.byte_len * [0]
        # add constant fields
        for le in filter(lambda le: le.const != None, self.layout):
            debug("C %28s = %u" % (le.name,le.const))
            self.tr(le, int(le.const), ret)
        # add default fields that are not defined explicitly
        for le in filter(lambda le: le.const == None and le.default != None, self.layout):
            if le.count == 1 and le.name not in data:
                debug("D %28s = %u" % (le.name,le.default))
                self.tr(le, int(le.default), ret)
            elif le.count > 1:
                for i in range(le.count):
                    le_name = le.name + "[%d]"%i
                    if le_name not in data:
                        debug("D %28s = %u" % (le_name, le.default))
                        self.tr(self.str_lookup[le_name], int(le.default), ret)
        for name, val in data.items():
            # find the data element in the layout
            le = self.str_lookup.get(name, None)
            if not le:
                if name[0] != "_":
                    warning("%s does not exist in layout" % name)
                continue
            try:
                self.tr(le, int(val), ret)
                debug("E %28s = %s, bit ofs %d" % (name,val,le.bit_ofs))
            except:
                error("cannot convert %s with value %s" % (name,val))
        return ret
    # def compile

# class Layout


# A single configuration table in the ES/Switch
# A configuration table can have multiple entries
# The entries of a table can have different layouts but must have the same size
class Config_Table (object):

    # Arguments:
    #   name:      configuration table name
    #   offset:    byte offset in the device address space
    #   size:      number of entries in the table
    #   layouts:   dictionary of allowed layouts {"name": Layout}
    #   layout_fn: layout detection function (from data)
    def __init__(self, name, offset, size, layouts={}, layout_fn=None, default_layout=None, first_index=0, parts=1):
        self.name        = name
        self.offset      = offset
        self.size        = size
        self.layouts     = layouts
        self.layout      = self.layouts.values()[0]
        if layout_fn:
            self.get_layout = layout_fn
        else:
            self.get_layout = self._get_layout
        if default_layout:
            self.default_layout = default_layout
        else:
            self.default_layout = self.layout
        self.first_index = first_index
        self.parts       = parts

        # make sure that multiple layouts have the same length
        byte_lens = set(map(lambda x: x.byte_len, self.layouts.values()))
        if len(byte_lens) > 1:
            max_len = max(byte_lens)
            for l in self.layouts.values():
                diff = (max_len - l.byte_len) * 8
                l.add_padding(diff)
            

    def _get_layout(self, data):
        return self.layout

