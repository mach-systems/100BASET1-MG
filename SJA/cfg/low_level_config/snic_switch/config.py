#! /usr/bin/env python
# -*- coding: iso-8859-1 -*-
########################################################################
# File name:       config.py
# Author:          Petr Grillinger
# Revision:        $Revision: 1.10 $
# Last changed by: $Author: nxp30460 $
# Last chanded on: $Date: Fri Jul  1 11:30:45 2016 $
# Description:     Configuration file generation for the TTGbE ES
########################################################################

# add the search path for parent package
import snic_switch
if len(snic_switch.__path__) < 2:
    snic_switch.__path__.append(snic_switch.__path__[0] + '\\..')

import sys
import table_defs  # snic_switch
import tables      # snic_switch
import hex_config  # snic_switch

def debug(msg):
    pass
def warning(msg):
    sys.stderr.write(msg + "\n")
def error(msg):
    sys.stderr.write(msg + "\n")



# Container for the complete configuration of an ES
# The container supports export into following formats:
#   HEX file suitable for the non-volatile memory initialization
#   Multiple HEX files for VHDL simulation (one per table)
# The container supports import from the NVM hex file
class Switch_Config(hex_config.Hex_Config):

    # Create an empty configuration
    # You have to fill the parameters manualy
    def __init__(self, layouts=tables.switch_layouts, table_defs=table_defs):
        hex_config.Hex_Config.__init__(self)

        self.td                  = table_defs
        self.layouts             = layouts
        self.gp                  = layouts.gp
        self.derived_params_done = False
        self.format              = "update"

        # create a list of hex tables (ordered the same as HEX file)
        self.table_map = [
            self.td.Schedule_Table(layouts),
            self.td.Schedule_Entry_Points_Table(layouts),
            self.td.VL_Lookup_Table(layouts),
            self.td.VL_Policing_Table(layouts),
            self.td.VL_Routing_Table(layouts),
            self.td.COTS_Lookup_Table(layouts),
            self.td.COTS_Policing_Table(layouts),
            self.td.COTS_VLAN_Lookup_Table(layouts),
            self.td.COTS_Routing_Table(layouts),
            self.td.MAC_Configuration_Table(layouts),
            self.td.Schedule_Parameters(layouts),
            self.td.Schedule_Entry_Parameters(layouts),
            self.td.VL_Routing_Parameters(layouts),
            self.td.COTS_Lookup_Parameters(layouts),
            self.td.COTS_Routing_Parameters(layouts),
            self.td.Clock_Synchronization_Parameters(layouts),
            self.td.General_Parameters(layouts),
            self.td.AVB_Parameters(layouts),
            self.td.Retagging_Table(layouts),
            self.td.CBS_Table(layouts),
            self.td.MII_Mode_Parameters(layouts),
            # self.td.External_Blocks_Not_Used(layouts),
            self.td.CGU_Parameters(layouts),
            self.td.ACU_Parameters(layouts),
            self.td.SGMII_Parameters(layouts),
        ]

        # set-up direct access to data
        self.sch_table                         = self.table_map[0]
        self.sd_table                          = self.table_map[1]
        self.ivl_table                         = self.table_map[2]
        self.ivp_table                         = self.table_map[3]
        self.ivr_table                         = self.table_map[4]
        self.icl_table                         = self.table_map[5]
        self.icp_table                         = self.table_map[6]
        self.iql_table                         = self.table_map[7]
        self.icr_table                         = self.table_map[8]
        self.mac_table                         = self.table_map[9]
        self.sch_parameters                    = self.table_map[10]
        self.sd_parameters                     = self.table_map[11]
        self.ivr_parameters                    = self.table_map[12]
        self.icl_parameters                    = self.table_map[13]
        self.icr_parameters                    = self.table_map[14]
        self.css_parameters                    = self.table_map[15]
        self.cc_parameters                     = self.table_map[16]
        self.avb_parameters                    = self.table_map[17]
        self.lbp_table                         = self.table_map[18]
        self.cbs_table                         = self.table_map[19]
        self.mii_mode_parameters               = self.table_map[20]
        self.cgu_parameters                    = self.table_map[21]
        self.acu_parameters                    = self.table_map[22]
        self.sgmii_parameters                  = self.table_map[23]
    # def __init__
    # def __init__

    def get_tables(self):
        return self.table_map

    # make sure that the table references are not replaced by using '=' operator
    # the object must stay the same, only the contents is replaced
    def __setattr__(self, name, value):
        # attribute does not exist
        if name not in self.__dict__:
            object.__setattr__(self, name, value)
        # list-table attribute
        elif isinstance(self.__dict__[name], table_defs.Generic_Table):
            self.__dict__[name][:] = value
        # dict-table attribute
        elif isinstance(self.__dict__[name], table_defs.Generic_Parameters):
            self.__dict__[name].clear
            self.__dict__[name].update(value)
        # default attribute
        else:
            object.__setattr__(self, name, value)
    # def __setattr__


    # delete all data from all tables
    def clear(self):
        for tab in self.table_map:
            tab.clear()
        self.derived_params_done = False
        self.format              = "update"
    # def clear


    # put into format for configuration file output
    def format_for_output(self):
        if self.format == "output":
            return
        self.compute_derived_params()
        self.format = "output"
    # def format_for_output


    def format_for_update(self):
        if self.format == "update":
            return
        self.format = "update"
    # def format_for_update


    # only for parameters that are hand-coded. Not used when loading HEX from file
    def compute_derived_params(self):
        if self.derived_params_done:
            return

        self.derived_params_done = True
    # def compute_derived_params


    # called when all configuration tables have been loaded from a HEX file
    # to do final post-processing
    def finish_loading(self):
        ret = True
        for tab in self.table_map:
            ret = ret and tab.finish_loading(self)

        # We do not want to modify loaded parameters
        self.derived_params_done = True

        self.format = "output"
        self.format_for_update()
        return ret
    # def finish_loading


    def validate(self):
        if not self.derived_params_done:
            self.compute_derived_params()
    # def validate

# class Switch_Config


if __name__ == "__main__":
    if len(sys.argv) > 1:
        esc = Switch_Config()
        esc.load_nvm_file(sys.argv[1])
        print str(esc).replace(",", "\n").replace("}", "\n").replace("L","")
    else:
        print "Usage: %s <cfg_file.hex>" % sys.argv[0]

