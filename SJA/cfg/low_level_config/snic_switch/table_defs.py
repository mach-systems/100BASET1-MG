#! /usr/bin/env python
# -*- coding: iso-8859-1 -*-
########################################################################
# File name:       es_table_defs.py
# Author:          Petr Grillinger
# Revision:        $Revision: 1.10 $
# Last changed by: $Author: nxp30460 $
# Last chanded on: $Date: Fri Jul  1 11:30:45 2016 $
# Description:     EndSystem table definitions
########################################################################

# add the search path for parent package
import snic_switch
if len(snic_switch.__path__) < 2:
    snic_switch.__path__.append(snic_switch.__path__[0] + '\\..')

from common.hex_config import Hex_Table
from common.debug import debug,warning,error


class Generic_Table(Hex_Table, list):
    def __init__(self, table, file_name=None):
        Hex_Table.__init__(self,
            table     = table,
            file_name = file_name)
        list.__init__(self)

    def clear(self):
        del self[:]

    def store(self):
        if len(self) > self.table.size:
            error("%s is too large (%d/%d)" % (self.table.name, len(self), self.table.size))
        ret = []
        for params in self:
            layout = params.get("_type", self.layout)
            ret.extend(layout.to_bin(params))
        return ret

    def load(self, blist, start, bsize):
        rec_len = self.layout.byte_len
        if bsize / rec_len > self.table.size:
            error("%s table is too large: %d/%d" % (self.table.name,bsize/rec_len,self.table.size))
            return False
        tmp_list = []
        for i in range(start, start+bsize, rec_len):
            blk    = blist[i:i+rec_len]
            layout = self.table.get_layout(blk)
            tmp_list.append(layout.to_dict(blk))
        self[:] = tmp_list
        return True
### class Generic_Table

class Generic_Parameters(Hex_Table, dict):
    def __init__(self, table, file_name=None):
        Hex_Table.__init__(self,
            table     = table,
            file_name = file_name)
        dict.__init__(self)

    def getdef(self, key):
        default = self.layout.str_lookup[key].default
        if default == None:
            default = 0
        return self.get(key, default)

    def store(self):
        if self:
            return self.layout.to_bin(self)
        else:
            return []

    def load(self, blist, start, bsize):
        if bsize != self.layout.byte_len:
            warning("%s table has an invalid size: %d/%d" % (self.table.name, bsize, self.layout.byte_len))
        self.clear()
        blk = blist[start:start+bsize]
        self.update(self.layout.to_dict(blk))
        return True
### class Generic_Table


class Schedule_Table(Generic_Table):
    def __init__(self, layouts):
        Generic_Table.__init__(self, layouts.sch_table)

class VL_Lookup_Table(Generic_Table):
    def __init__(self, layouts):
        Generic_Table.__init__(self, layouts.ivl_table)

class VL_Policing_Table(Generic_Table):
    def __init__(self, layouts):
        Generic_Table.__init__(self, layouts.ivp_table)

class VL_Routing_Table(Generic_Table):
    def __init__(self, layouts):
        Generic_Table.__init__(self, layouts.ivr_table)

class COTS_Lookup_Table(Generic_Table):
    def __init__(self, layouts):
        Generic_Table.__init__(self, layouts.icl_table)

class COTS_Routing_Table(Generic_Table):
    def __init__(self, layouts):
        Generic_Table.__init__(self, layouts.icr_table)

class COTS_VLAN_Lookup_Table(Generic_Table):
    def __init__(self, layouts):
        Generic_Table.__init__(self, layouts.iql_table)

class COTS_Lookup_Parameters(Generic_Parameters):
    def __init__(self, layouts):
        Generic_Parameters.__init__(self, layouts.icl_parameters)

class COTS_Policing_Table(Generic_Table):
    def __init__(self, layouts):
        Generic_Table.__init__(self, layouts.icp_table)

class Schedule_Entry_Points_Table(Generic_Table):
    def __init__(self, layouts):
        Generic_Table.__init__(self, layouts.sd_table)

class MAC_Configuration_Table(Generic_Table):
    def __init__(self, layouts):
        Generic_Table.__init__(self, layouts.mac_table)

class Retagging_Table(Generic_Table):
    def __init__(self, layouts):
        Generic_Table.__init__(self, layouts.lbp_table)

class Schedule_Parameters(Generic_Parameters):
    def __init__(self, layouts):
        Generic_Parameters.__init__(self, layouts.sch_parameters)

class Schedule_Entry_Parameters(Generic_Parameters):
    def __init__(self, layouts):
        Generic_Parameters.__init__(self, layouts.sd_parameters)

class Clock_Synchronization_Parameters(Generic_Parameters):
    def __init__(self, layouts):
        Generic_Parameters.__init__(self, layouts.css_parameters)

class VL_Routing_Parameters(Generic_Parameters):
    def __init__(self, layouts):
        Generic_Parameters.__init__(self, layouts.ivr_parameters)

class COTS_Routing_Parameters(Generic_Parameters):
    def __init__(self, layouts):
        Generic_Parameters.__init__(self, layouts.icr_parameters)

class AVB_Parameters(Generic_Parameters):
    def __init__(self, layouts):
        Generic_Parameters.__init__(self, layouts.avb_parameters)

class MII_Mode_Parameters(Generic_Parameters):
    def __init__(self, layouts):
        Generic_Parameters.__init__(self, layouts.mii_mode_parameters)

class General_Parameters(Generic_Parameters):
    def __init__(self, layouts):
        Generic_Parameters.__init__(self, layouts.cc_parameters)

# class External_Blocks_Not_Used(Generic_Parameters):
#     def __init__(self, layouts):
#         Generic_Parameters.__init__(self, layouts.external_blocks_not_used)

class CBS_Table(Generic_Table):
    def __init__(self, layouts):
        Generic_Table.__init__(self, layouts.cbs_table)

class CGU_Parameters(Generic_Parameters):
    def __init__(self, layouts):
        Generic_Parameters.__init__(self, layouts.cgu_parameters)

class RGU_Parameters(Generic_Parameters):
    def __init__(self, layouts):
        Generic_Parameters.__init__(self, layouts.rgu_parameters)
                                       
class ACU_Parameters(Generic_Parameters):
    def __init__(self, layouts):
        Generic_Parameters.__init__(self, layouts.acu_parameters)


class SGMII_Parameters(Generic_Parameters):
    def __init__(self, layouts):
        Generic_Parameters.__init__(self, layouts.sgmii_parameters)


