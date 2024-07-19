#! /usr/bin/env python
# -*- coding: iso-8859-1 -*-
########################################################################
# File name:       xml_export.py
# Author:          Lukas Kellner, Petr Grillinger
# Revision:        $Revision: 1.10 $
# Last changed by: $Author: nxp30460 $
# Last chanded on: $Date: Fri Jul  1 11:30:45 2016 $
# Description:     Export configuration to XML file 
########################################################################

import sys
from array import array

def debug(msg):
    pass

def error(msg):
    sys.stderr.write(msg + "\n")

def warning(msg):
    sys.stderr.write(msg + "\n")

# Store one or more configurations into a single XML file
# configs is a list of objects that must support following functions:
#    get_tables(self)
#    format_for_output(self)
#    format_for_update(self)
def store_xml_file(device_id,                # 8-bit number
                   revision,                 # 24-bit number
                   configs,                  # list of configuration object
                   file_name="config.xml"):  # HEX file name + path

    debug("Running XML export ... (%s)" % file_name)

    # create intel-hex container
    indent = "   "
    f = file(file_name, "w")
    f.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
     
    f.write("<config>\n")  
    f.write(indent + "<version>\n")
    f.write(indent + indent + "<item>\n")
    f.write(indent + indent + indent + "<name>deviceId</name>\n")
    f.write(indent + indent + indent + "<value>%s</value>\n" % (device_id,))   
    f.write(indent + indent + "</item>\n" )
    f.write(indent + indent + "<item>\n")
    f.write(indent + indent + indent + "<name>revision</name>\n")
    f.write(indent + indent + indent + "<value>%s</value>\n" % (device_id,))   
    f.write(indent + indent + "</item>\n" )    
    f.write(indent + "</version>\n")    

    # store all configuration objects
    for config in configs:
       # switch to output format
       config.format_for_output()
       # store individual tables into ihex
       for table in config.get_tables():
            tab_def = table.table
            # get the table content as list of bytes
            f.write("<table>\n")
            f.write(indent + "<name>%s</name>\n" % tab_def.name)            
            #f.write(indent + "<size>%d</size>\n" % tab_def.size)
            if isinstance(table, list):   # iterable
               for data in table:
                  layout  = data.get("_type", tab_def.layout)
                  #f.write(layout.to_xml(data, indent="   "))
                  f.write(layout.to_xml(data, indent))
            else: # non-iterable
               data    = table
               layout  = data.get("_type", tab_def.layout)
               f.write(layout.to_xml(data, indent))
                  
            f.write("</table>\n")

       # switch to edit-format
       config.format_for_update()

    f.write("</config>\n")
    debug("Creating %s" % file_name)



