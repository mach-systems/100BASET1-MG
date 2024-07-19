############################################################################
# SJA1105PQ example configuration
#
# This is an example showcasing many of the configuration possibilities.
# This includes
# * Example CBS configuration on port 0
# * An example policer setup with policing on port 1 and 2
# * Port speed and MII mode parameters
# * MAC filtering configuration for gPTP
# * Examples of ARL and VLAN table configuration
#
############################################################################

import sys
import os.path
import shutil
import copy
import code
local_dir = os.path.dirname(sys.argv[0])
sys.path.append(os.path.join(local_dir, "low_level_config"))

from snic_switch.tables import *
from snic_switch.config import Switch_Config

# Create an empty configuration with the modified configuration
swc = Switch_Config()

#############################################################################
# General Parameters
#############################################################################

swc.cc_parameters["VLLUPFORMAT"]   = 0
swc.cc_parameters["MIRR_PTACU"]    = 1 # Dynamic change of Mirror Port is enabled
swc.cc_parameters["SWITCHID"]      = 0
swc.cc_parameters["HOSTPRIO"]      = 5
swc.cc_parameters["MAC_FLTRES[0]"] = 0xFFFFFFFFFFFF  # PTP Multicast Address  0x0180C200000E
swc.cc_parameters["MAC_FLTRES[1]"] = 0x0180C2000003  # EAPOL Multicast Address (for 802.1X)
swc.cc_parameters["MAC_FLT[0]"]    = 0x000000000000
swc.cc_parameters["MAC_FLT[1]"]    = 0xFFFFFF0000FF
swc.cc_parameters["INCL_SRCPT[0]"] = 1
swc.cc_parameters["INCL_SRCPT[1]"] = 1
swc.cc_parameters["SEND_META[0]"]  = 1
swc.cc_parameters["SEND_META[1]"]  = 0
swc.cc_parameters["CASC_PORT"]     = 6
swc.cc_parameters["MIRR_PORT"]     = 6 # No default mirror port. Set through reconfiguration
swc.cc_parameters["HOST_PORT"]     = 2
swc.cc_parameters["VIMARKER"]      = 0xFFFFFFFF
swc.cc_parameters["VIMASK"]        = 0xFFFFFFFF
swc.cc_parameters["TPID"]          = 0x88A8
swc.cc_parameters["IGNORE2STF"]    = 0
swc.cc_parameters["TPID2"]         = 0x8100
swc.cc_parameters["QUEUE_TS"]      = 0
swc.cc_parameters["EGRMIRRVID"]    = 0
swc.cc_parameters["EGRMIRRPCP"]    = 0
swc.cc_parameters["EGRMIRRDEI"]    = 0
swc.cc_parameters["REPLAY_PORT"]   = 7

#############################################################################
# MAC Configuration Table
#############################################################################

SPEED_HOST    = 0  # speed set by host during run-time
SPEED_1GBPS   = 1
SPEED_100MBPS = 2
SPEED_10MBPS  = 3

speed = [SPEED_100MBPS, SPEED_100MBPS, SPEED_100MBPS, SPEED_100MBPS, SPEED_HOST]
speed_Mbps  = [10**(4-x) for x in speed]
speed_Mbps[4] = 10**(4-1) #if speed is HOST but can reach 1G

default_vlan = 555  # Default VLAN ID on all ports for untagged frames is 555

queue_enable = [1,1,1,1,1,1,1,1]
prio_queue0  = [0,63]
prio_queue1  = [64,127]
prio_queue2  = [128,191]
prio_queue3  = [192,255]
prio_queue4  = [256,319]
prio_queue5  = [320,383]
prio_queue6  = [384,447]
prio_queue7  = [448,511]

for i in range(swc.gp.NO_ETH_PORTS):
	swc.mac_table.append({
		"INGMIRRDEI" : 0,
		"INGMIRRPCP" : 0,
		"INGMIRRVID" : 0,
		"MIRRCETAG"  : 0,
		"MIRRCIE"    : 0,
        "INGRESS"    : 1,
        "EGRESS"     : 1,
        "DYN_LEARN"  : 1,
		"DRPNONA664" : 0,
		"EGR_MIRR"   : 0, 
		"ING_MIRR"   : 0,
		"VLANID"     : default_vlan,
		"VLANPRIO"   : 0, 
		"MAXAGE"     : 255,
		"TP_DELOUT"  : 0,
		"TP_DELIN"   : 0,
		"SPEED"      : speed[i], 
		"IFG"        : 0,
		"ENABLED[0]" : queue_enable[0], # enable the queue
		"BASE[0]"    : prio_queue0[0], # start
		"TOP[0]"     : prio_queue0[1], # set the size of the queue to maximum size
		"ENABLED[1]" : queue_enable[1],
		"BASE[1]"    : prio_queue1[0],
		"TOP[1]"     : prio_queue1[1],
		"ENABLED[2]" : queue_enable[2],
		"BASE[2]"    : prio_queue2[0],
		"TOP[2]"     : prio_queue2[1],
		"ENABLED[3]" : queue_enable[3],
		"BASE[3]"    : prio_queue3[0],
		"TOP[3]"     : prio_queue3[1],
		"ENABLED[4]" : queue_enable[4],
		"BASE[4]"    : prio_queue4[0],
		"TOP[4]"     : prio_queue4[1],
		"ENABLED[5]" : queue_enable[5],
		"BASE[5]"    : prio_queue5[0],
		"TOP[5]"     : prio_queue5[1],
		"ENABLED[6]" : queue_enable[6],
		"BASE[6]"    : prio_queue6[0],
		"TOP[6]"     : prio_queue6[1],
		"ENABLED[7]" : queue_enable[7],
		"BASE[7]"    : prio_queue7[0],
		"TOP[7]"     : prio_queue7[1]})

#############################################################################
# Credit-Based Shaping Table
#############################################################################

# bandwidth given in Mbit/s
# Example configuration for 4 shapers in port 0
shapers = [
	{"bandwidth": 0.5, "prio": 7, "port": 1},
	{"bandwidth": 5,   "prio": 6, "port": 1},
	{"bandwidth": 10,  "prio": 5, "port": 1},
	{"bandwidth": 20,  "prio": 4, "port": 1}]

assert len(shapers) <= swc.gp.NO_CBS_BLOCKS, "No of shapers configured exceeds " + str(swc.gp.NO_CBS_BLOCKS)

for shaper in shapers:
	idle_slope = shaper["bandwidth"] / 8.0 * 10**6  # conversion from Mbit/s to byte/s
	rate = speed_Mbps[shaper["port"]] / 8.0 * 10**6  # rate at the port in byte/s
	send_slope = rate - idle_slope
	swc.cbs_table.append({
		"IDLE_SLOPE" : idle_slope,
		"SEND_SLOPE" : send_slope,
		"CREDIT_HI"  : 0x7FFFFFFF,
		"CREDIT_LO"  : 0x7FFFFFFF,
		"CBS_PRIO"   : shaper["prio"],
		"CBS_PORT"   : shaper["port"]})
	print(idle_slope)


#############################################################################
# VLAN Lookup Table
#############################################################################

# Default VLAN
swc.iql_table.append({
	"VING_MIRR"  : 0,
	"VEGR_MIRR"  : 0,
	"VMEMB_PORT" : 0x1F, # All ports are member of the VLAN
	"VLAN_BC"    : 0x1F, # Broadcast domain for the VLAN
	"TAG_PORT"   : 0x00, # Egress frames are untagged
	"VLANID"     : default_vlan})

# Enable VLANs 0 to 15
for i in range(16):
	swc.iql_table.append({
		"VING_MIRR"  : 0,
		"VEGR_MIRR"  : 0,
		"VMEMB_PORT" : 0x1F, # all ports are member
		"VLAN_BC"    : 0x1F, # Broadcast domain
		"TAG_PORT"   : 0x1F, # Egress frames are tagged
		"VLANID"     : i})

#############################################################################
# L2 Lookup Parameters Table
#############################################################################

swc.icl_parameters["LEARN_ONCE"]     = 0
swc.icl_parameters["OWR_DYN"]        = 0
swc.icl_parameters["USE_STATIC"]     = 0
swc.icl_parameters["NO_MGMT_LEARN"]  = 1
swc.icl_parameters["NO_ENF_HOSTPRT"] = 0 
swc.icl_parameters["DRPNOLEARN"]     = 0
swc.icl_parameters["START_DYNSPC"]   = 0
swc.icl_parameters["MAXAGE"]         = 0x2EDF   # 2 minutes
swc.icl_parameters["MAXADDRP[0]"]    = 1024
swc.icl_parameters["MAXADDRP[1]"]    = 1024
swc.icl_parameters["MAXADDRP[2]"]    = 1024   
swc.icl_parameters["MAXADDRP[3]"]    = 1024   
swc.icl_parameters["MAXADDRP[4]"]    = 1024   
swc.icl_parameters["DRPUNI"]         = 0
swc.icl_parameters["DRPMC"]          = 0
swc.icl_parameters["DRPBC"]          = 0

#############################################################################
# L2 Address Lookup Table
#############################################################################

swc.icl_table.append({
	"INDEX"     	: 0,
	"ENFPORT"   	: 0,
	"DESTPORTS"		: 1 << 0,
	"MACADDR"   	: 0x001094000001,
	"VLANID"    	: default_vlan,
	"IOTAG"     	: 0,
	"MASK_MACADDR" 	: 0xFFFFFFFFFFFF,
	"MASK_VLANID" 	: 0xFFF,
	"MASK_IOTAG"	: 0x1,
	"RETAG"     	: 0,
	"MIRR"      	: 0,
	"TAKETS"    	: 0,
	"MIRRVLAN"  	: 0,
	"TSREG"     	: 0})

#############################################################################
# L2 Policing Table
#############################################################################

# In this example, every port/priority is assigned a dedicated policer.
# By use of SHARINDX, multiple ports/priorities can be mapped to a single policer.

# defines the ratio of available bandwidth that is admitted for a prio/port combination in %
# a value of 100 means that no policing is performed
ratio = [[100, 100, 100, 100, 100, 100, 100, 100],  # Port 0 - no policing
         [100, 100, 100, 100, 100, 100, 100, 100], # Port 1 - 10% for at each prio, except prio 0
         [100, 100, 100, 100, 100, 100, 100, 100],  # Port 2 - only lowest prio allowed
         [100, 100, 100, 100, 100, 100, 100, 100],  # Port 3 - no policing
         [100, 100, 100, 100, 100, 100, 100, 100]]  # Port 4 - no policing

for port in range(swc.gp.NO_ETH_PORTS):
	for prio in range(swc.gp.NO_PRIORITIES):
		swc.icp_table.append({
			"SHARINDX"  : port * swc.gp.NO_PRIORITIES + prio, # individual policing block for each priority
			"SMAX"      : 10 * 1526, # Maximum burst as 10 maximum sized, double tagged frames
			"RATE"      : speed_Mbps[port] * ratio[port][prio]/100.0 * 1000/15.625,  # Unit: [15.625 kbps]
			"MAXLEN"    : 1526,
			"PARTITION" : prio})  # memory is partitioned towards the priorities


# Broadcast Storm Prevention
# Defines the ratio of available bandwidth that is admitted for a port
# ratio = [port 0, port 1, ..., port 4]
ratio = [100, 100, 100, 100, 100] # All ports admit up to 100% broadcast traffic
for port in range(5):			
	swc.icp_table.append({
		"SHARINDX"  : 40 + port, # individual policing block for each priority
		"SMAX"      : 10 * 1526, # Maximum burst as 10 maximum sized, double tagged frames
		"RATE"      : speed_Mbps[port] * ratio[port]/100.0 * 1000/15.625,  # Unit: [15.625 kbps]
		"MAXLEN"    : 1526,
		"PARTITION" : 0})  # memory is partitioned towards the priorities
			
############################################################################
# L2 Forwarding Table
#############################################################################

# retain the priority of the frames at ingress
for i in range(swc.gp.NO_ETH_PORTS):
	if (i == swc.cc_parameters["HOST_PORT"]):
		reachable_ports	= 0x1F  # host port is reachable by itself, needed for hybrid AVB implementation with endpoint and bridge stack on a single host
	else:
		reachable_ports	= 0x1F & ~(1 << i)
	broadcast_domain = 0x1F & ~(1 << i)
	default_route    = 0x1F & ~(1 << i)

	# Priority regeneration
	priority_map     = [0,1,2,3,4,5,6,7]  # No PCP modification	

	swc.icr_table.append({
		"FL_DOMAIN"    : default_route,
		"VLAN_BC"      : broadcast_domain,
		"REACH_PORT"   : reachable_ports,
		"VLAN_PMAP[0]" : priority_map[0],
		"VLAN_PMAP[1]" : priority_map[1],
		"VLAN_PMAP[2]" : priority_map[2],
		"VLAN_PMAP[3]" : priority_map[3],
		"VLAN_PMAP[4]" : priority_map[4],
		"VLAN_PMAP[5]" : priority_map[5],
		"VLAN_PMAP[6]" : priority_map[6],
		"VLAN_PMAP[7]" : priority_map[7]})

# Output PCP to queue mapping
# map priority i to queue i on all ports
for i in range(swc.gp.NO_PRIORITIES):
	swc.icr_table.append({
		"VLAN_PMAP[0]" : i,
		"VLAN_PMAP[1]" : i,
		"VLAN_PMAP[2]" : i,
		"VLAN_PMAP[3]" : i,
		"VLAN_PMAP[4]" : i})

#############################################################################
# L2 Forwarding Parameters Table
#############################################################################

swc.icr_parameters["MAX_DYNP"]    = 0
swc.icr_parameters["PART_SPC[7]"] = 100
swc.icr_parameters["PART_SPC[6]"] = 100
swc.icr_parameters["PART_SPC[5]"] = 100
swc.icr_parameters["PART_SPC[4]"] = 100
swc.icr_parameters["PART_SPC[3]"] = 100
swc.icr_parameters["PART_SPC[2]"] = 100
swc.icr_parameters["PART_SPC[1]"] = 100
swc.icr_parameters["PART_SPC[0]"] = 210

assert sum([swc.icr_parameters["PART_SPC[%d]" % i] for i in range(8)]) <= 910, 'sum of paritions must not exceed 910 (if retagging used)'

#############################################################################
# AVB Parameters
#############################################################################

swc.avb_parameters["SRCMETA"]    = 0x026037C0FFEE
swc.avb_parameters["DESTMETA"]   = 0x026037DECADE
swc.avb_parameters["CAS_MASTER"] = 1
swc.avb_parameters["L2CBS"]      = 0

#############################################################################
# MII Mode Control Parameters
#############################################################################

MII    = 0
RMII   = 1
RGMII  = 2
UNUSED = 3
SGMII  = 3

PHY_MODE = 1
MAC_MODE = 0

swc.mii_mode_parameters["xMII_MODE[0]"] = RMII
swc.mii_mode_parameters["PHY_MAC[0]"]   = PHY_MODE
swc.mii_mode_parameters["xMII_MODE[1]"] = RMII
swc.mii_mode_parameters["PHY_MAC[1]"]   = MAC_MODE
swc.mii_mode_parameters["xMII_MODE[2]"] = RMII
swc.mii_mode_parameters["PHY_MAC[2]"]   = MAC_MODE
swc.mii_mode_parameters["xMII_MODE[3]"] = RMII
swc.mii_mode_parameters["PHY_MAC[3]"]   = MAC_MODE
swc.mii_mode_parameters["xMII_MODE[4]"] = RGMII
swc.mii_mode_parameters["PHY_MAC[4]"]   = 0  # not applicable for RGMII

#############################################################################
# Main configuration done, start creating output
#############################################################################

SJA1105_PR = 0xAF  # SJA1105P and SJA1105R (no TSN features)
SJA1105_QS = 0xAE  # SJA1105Q and SJA1105S (TSN features)

filename = os.path.basename(__file__)
filename   = filename.replace(".py", "_SJA1105QS.hex")

swc.create_nvm_file(os.path.join(local_dir, filename), True, SJA1105_QS)
