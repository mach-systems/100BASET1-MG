#! /usr/bin/env python
# -*- coding: iso-8859-1 -*-
########################################################################
# File name:       debug.py
# Author:          Petr Grillinger
# Revision:        $Revision: 1.10 $
# Last changed by: $Author: nxp30460 $
# Last chanded on: $Date: Fri Jul  1 11:30:45 2016 $
# Description:     Debugging & logging
########################################################################

from sys import stderr,stdout

enable_debug    = False
enable_warning  = True
enable_error    = True
stop_on_warning = False

def debug(msg):
    if enable_debug:
        stdout.write(msg + "\n")

def warning(msg):
    if enable_warning:
        if stop_on_warning:
            raise Exception(msg)
        else:
            stderr.write(msg + "\n")

def error(msg):
    if enable_error:
        raise Exception(msg)

