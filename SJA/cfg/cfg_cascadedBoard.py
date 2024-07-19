############################################################################
# Reference Design Board Configuration
############################################################################

import sys
import os.path
import shutil
import copy
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
swc.cc_parameters["MAC_FLTRES[0]"] = 0x0180C200000E  # PTP Multicast Address
swc.cc_parameters["MAC_FLTRES[1]"] = 0xFFFFFFFFFFFF
swc.cc_parameters["MAC_FLT[0]"]    = 0xFFFFFF0000FF
swc.cc_parameters["MAC_FLT[1]"]    = 0xFFFFFFFFFFFF
swc.cc_parameters["INCL_SRCPT[0]"] = 0
swc.cc_parameters["INCL_SRCPT[1]"] = 0
swc.cc_parameters["SEND_META[0]"]  = 1
swc.cc_parameters["SEND_META[1]"]  = 0
swc.cc_parameters["CASC_PORT"]     = 4
swc.cc_parameters["MIRR_PORT"]     = 6 # No default mirror port. Set through reconfiguration
swc.cc_parameters["HOST_PORT"]     = 0
swc.cc_parameters["VIMARKER"]      = 0xFFFFFFFF
swc.cc_parameters["VIMASK"]        = 0xFFFFFFFF
swc.cc_parameters["TPID"]          = 0x88A8
swc.cc_parameters["IGNORE2STF"]    = 1
swc.cc_parameters["TPID2"]         = 0x8100
swc.cc_parameters["QUEUE_TS"]      = 0
swc.cc_parameters["EGRMIRRVID"]    = 0
swc.cc_parameters["EGRMIRRPCP"]    = 0
swc.cc_parameters["EGRMIRRDEI"]    = 0
swc.cc_parameters["REPLAY_PORT"]   = 7

#############################################################################
# Credit-Based Shaping Table
#############################################################################

# No shapers configured

#############################################################################
# MAC Configuration Table
#############################################################################

SPEED_HOST    = 0  # speed set by host during run-time
SPEED_1GBPS   = 1
SPEED_100MBPS = 2
SPEED_10MBPS  = 3

queue_enable = [1,1,1,1,1,1,1,1]
prio_queue0  = [0,63]
prio_queue1  = [64,127]
prio_queue2  = [128,191]
prio_queue3  = [192,255]
prio_queue4  = [256,319]
prio_queue5  = [320,383]
prio_queue6  = [384,447]
prio_queue7  = [448,511]

speed = [SPEED_100MBPS, SPEED_100MBPS, SPEED_1GBPS, SPEED_1GBPS, SPEED_1GBPS]

default_vlan = 555  #Default VLAN ID on all ports for untagged frames is 555

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
swc.icl_parameters["NO_MGMT_LEARN"]  = 0
swc.icl_parameters["NO_ENF_HOSTPRT"] = 0 
swc.icl_parameters["DRPNOLEARN"]     = 0
swc.icl_parameters["START_DYNSPC"]   = 0
swc.icl_parameters["MAXAGE"]         = 0
swc.icl_parameters["MAXADDRP[0]"]    = 1024
swc.icl_parameters["MAXADDRP[1]"]    = 1024
swc.icl_parameters["MAXADDRP[2]"]    = 1024   
swc.icl_parameters["MAXADDRP[3]"]    = 1024   
swc.icl_parameters["MAXADDRP[4]"]    = 1024   
swc.icl_parameters["DRPUNI"]         = 0
swc.icl_parameters["DRPMC"]          = 0
swc.icl_parameters["DRPBC"]          = 0

#############################################################################
# L2 Policing Table
#############################################################################

for i in range(swc.gp.NO_ETH_PORTS* swc.gp.NO_PRIORITIES):
	 swc.icp_table.append({
			"SHARINDX"  : i/swc.gp.NO_PRIORITIES, #setting SHARINDX for a given port to a single policing block for all prios
			"SMAX"      : 2**16-1,
			"RATE"      : 2**16-1, ##1000*2**6,
			"MAXLEN"    : 1518,
			"PARTITION" : 0})
		
############################################################################
# L2 Forwarding Table
#############################################################################

# retain the priority of the frames at ingress
for i in range(swc.gp.NO_ETH_PORTS):
	if (i == swc.cc_parameters["HOST_PORT"]):
		reachable_ports	= 0x1F  # host port is reachable by itself
	else:
		reachable_ports	= 0x1F & ~(1 << i)
	broadcast_domain = 0x1F & ~(1 << i)
	default_route    = (0x1F & ~(1 << i)) & ~(1 << swc.cc_parameters["HOST_PORT"])  # not to itself or to the host port
	print(default_route)
	priority_map     = [0,1,2,3,4,5,6,7]
	# print "Append to ICR table: port %d, reachable_ports h'%x, broadcast_domain h'%x, default_route h'%x." % (i, reachable_ports, broadcast_domain, default_route)   
		
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

# map priority i to queue i on all ports
for i in range(8):
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
swc.icr_parameters["PART_SPC[7]"] = 0
swc.icr_parameters["PART_SPC[6]"] = 0
swc.icr_parameters["PART_SPC[5]"] = 0
swc.icr_parameters["PART_SPC[4]"] = 0
swc.icr_parameters["PART_SPC[3]"] = 0
swc.icr_parameters["PART_SPC[2]"] = 0
swc.icr_parameters["PART_SPC[1]"] = 0
swc.icr_parameters["PART_SPC[0]"] = 910 # three scratch-pad buffers per port

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

MII   = 0
RMII  = 1
RGMII = 2
GMII  = 3
SGMII = 3

PHY_MODE = 1
MAC_MODE = 0

swc.mii_mode_parameters["xMII_MODE[0]"] = MII
swc.mii_mode_parameters["PHY_MAC[0]"]   = PHY_MODE
swc.mii_mode_parameters["xMII_MODE[1]"] = RMII
swc.mii_mode_parameters["PHY_MAC[1]"]   = MAC_MODE
swc.mii_mode_parameters["xMII_MODE[2]"] = RGMII
swc.mii_mode_parameters["PHY_MAC[2]"]   = MAC_MODE
swc.mii_mode_parameters["xMII_MODE[3]"] = RGMII
swc.mii_mode_parameters["PHY_MAC[3]"]   = MAC_MODE
swc.mii_mode_parameters["xMII_MODE[4]"] = SGMII
swc.mii_mode_parameters["PHY_MAC[4]"]   = MAC_MODE 

#############################################################################
# Main configuration done, start creating output
#############################################################################

filename = os.path.basename(__file__)
suffix0 = "_switch0.hex"
suffix1 = "_switch1.hex"

filename   = filename.replace(".py", "")

suffix = suffix0

# Switch 0
swc.create_nvm_file(os.path.join(local_dir, filename + suffix), False, 0xAF)
print(os.path.join(local_dir, filename + suffix) + " created")

#############################################################################
# Make incremental changes for second (cascaded) switch
#############################################################################

speed = [SPEED_100MBPS, SPEED_100MBPS, SPEED_100MBPS, SPEED_100MBPS, SPEED_1GBPS]  # 1 = 1G, 2 = 100M, 3 = 10M

swc.avb_parameters["CAS_MASTER"] = 0

swc.mii_mode_parameters["xMII_MODE[0]"] = RMII
swc.mii_mode_parameters["PHY_MAC[0]"]   = MAC_MODE
swc.mii_mode_parameters["xMII_MODE[1]"] = RMII
swc.mii_mode_parameters["PHY_MAC[1]"]   = MAC_MODE
swc.mii_mode_parameters["xMII_MODE[2]"] = RMII
swc.mii_mode_parameters["PHY_MAC[2]"]   = MAC_MODE
swc.mii_mode_parameters["xMII_MODE[3]"] = RMII
swc.mii_mode_parameters["PHY_MAC[3]"]   = MAC_MODE
swc.mii_mode_parameters["xMII_MODE[4]"] = SGMII
swc.mii_mode_parameters["PHY_MAC[4]"]   = MAC_MODE

swc.cc_parameters["SWITCHID"]  = 1
swc.cc_parameters["HOST_PORT"] = 4
swc.cc_parameters["CASC_PORT"] = 6

for i in range(swc.gp.NO_ETH_PORTS):
	swc.mac_table[i]["SPEED"]      = speed[i]
	reachable_ports	               = 0x1F & ~(1 << i)
	broadcast_domain               = 0x1F & ~(1 << i)
	default_route                  = 0x1F & ~(1 << i)
	swc.icr_table[i]["FL_DOMAIN"]  = default_route
	swc.icr_table[i]["VLAN_BC"]    = broadcast_domain
	swc.icr_table[i]["REACH_PORT"] = reachable_ports

#############################################################################
# Create rest of the output
#############################################################################

suffix = suffix1

# Switch 1
# generate same config for SJA1105S
swc.create_nvm_file(os.path.join(local_dir, filename + suffix), False, 0xAF)
print(os.path.join(local_dir, filename + suffix) + " created")
