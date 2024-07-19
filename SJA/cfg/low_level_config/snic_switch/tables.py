#! /usr/bin/env python
# -*- coding: iso-8859-1 -*-
########################################################################
# File name:       tables.py
# Author:          Petr Grillinger
# Revision:        $Revision: 1.10 $
# Last changed by: $Author: nxp30460 $
# Last chanded on: $Date: Fri Jul  1 11:30:46 2016 $
# Description:     Definitions of all switch configuration tables
########################################################################

# add the search path for parent package
import snic_switch
from Switch_Global_Parameters import Switch_Global_Parameters

if len(snic_switch.__path__) < 2:
    snic_switch.__path__.append(snic_switch.__path__[0] + '\\..')

import common.bit_table

def log2(x):
    ret = 0
    while 2**ret < x:
        ret += 1
    return ret

########################################################################

# Collection of all end-system configuration tables
class Switch_Layouts (object):

    def __init__(self, gp, bit_table=common.bit_table):
        self.bt = bit_table
        self.update(gp)
    # def __init__

    def _get_ivp_layout(self, data):
        ofs      = self.ivp_tt_layout.byte_len - 1
        if data[ofs] >> 7:
            return self.ivp_tt_layout
        else:
            return self.ivp_rc_layout

    # gp is instance of Global_Parameters
    def update(self, gp):
        self.gp = gp

        ########################################################################
        name = "Schedule Table"
        self.sch_table_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("DELTA", log2( gp.MAX_DELTA+1 ), comment="Defines the offset of the trigger event with respect to the next trigger of the same sub-schedule."),
            self.bt.Layout_Entry("VLINDEX", log2( gp.NO_IN_VLS ), comment="Defines the index of the IVR table the trigger refers to. It is used only if TXEN is set."),
            self.bt.Layout_Entry("RESMEDIA", gp.NO_PRIORITIES+1, comment="Indicates that the DESTPORTS should (also - in case the TXEN flag is set, too) be used for media reservation (media reservation maintains an individual state per each sub-schedule; the actual reservation state is a bit-wise-or operation on the states of all sub-schedule reservations)."),
            self.bt.Layout_Entry("TXEN", 1, comment="Indicates that the abPortVector should (also - in case the bReserveMedia flag is set, too) be used for dispatching a frame."),
            self.bt.Layout_Entry("SETVALID", 1, comment="Indicates that the bValid flag of the respective IVR entry shall be set."),
            self.bt.Layout_Entry("DESTPORTS", gp.NO_ETH_PORTS, comment="Defines the ports (one-hot encoding) the respective trigger event applies to."),
            self.bt.Layout_Entry("WINST", 1, comment="Indicates that the bWindowOpen flag of the IVR entry indexed by bWindowStartIdx shall be set."),
            self.bt.Layout_Entry("WINEND", 1, comment="Indicates that the bWindowOpen flag of the IVR entry indexed by bVlIdx shall be cleared."),
            self.bt.Layout_Entry("WINSTINDEX", log2( gp.NO_IN_VLS ), comment="Defines the index of the IVR table the window control part of the trigger refers to. It is used only if the WINST flag is set."),
        ])
        self.sch_table = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = gp.SCHEDULE_ENTRIES,
            layouts = {name: self.sch_table_layout},
        )


        ########################################################################
        name = "Schedule Entry Points Table"
        self.sd_table_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("ADDRESS", log2( gp.SCHEDULE_ENTRIES ), comment="Defines the address within the Schedule Table to start execution with."),
            self.bt.Layout_Entry("DELTA", log2( gp.MAX_DELTA+1 ), comment="Defines the initial value of the delta value."),
            self.bt.Layout_Entry("SUBSCHINDX", log2( gp.NO_SUBSCHEDULES ), comment="Defines the index of the sub-schedule the respective entry is assigned to."),
        ])
        self.sd_table = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = (2**gp.PCF_INTEGRATION_CYCLE_WIDTH) * gp.NO_SUBSCHEDULES,
            layouts = {name: self.sd_table_layout},
        )


        ########################################################################
        name = "VL Lookup Table"
        self.ivl_table_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("VLANPRIO", 3, comment="Priority codepoint identifying this virtual link."),
            self.bt.Layout_Entry("PORT", log2( gp.NO_ETH_PORTS+1 ), comment="Eligible source port of the VL.", choices=range(gp.NO_ETH_PORTS+1)),
            self.bt.Layout_Entry("VLANID", gp.VLAN_ID_SIZE, comment="VL ID that is attached to the respective index."),
            self.bt.Layout_Entry("VLID", gp.VL_ID_SIZE, comment="VL ID that is attached to the respective index."),
            self.bt.Layout_Entry("VIMARKER", gp.VL_MARKER_SIZE, comment="Upper 32 bits of MAC address (for advanced mode use only)."),
            self.bt.Layout_Entry("ISCRITICAL", 1, comment="Set if this link should be treated as critical and VLLUPFORMAT = 0."),
            self.bt.Layout_Entry("DESTPORTS", gp.NO_ETH_PORTS, comment="Destination port vector if this entry is not critical, i.e., BE."),
        ])
        self.ivl_table = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = gp.NO_IN_VLS,
            layouts = {name: self.ivl_table_layout},
        )


        ########################################################################
        name = "VL Policing Table"
        # The layout of a VL Policing Table for TT VLs
        self.ivp_tt_layout = self.bt.Layout(name = "TT VL", pad_low = True, layout = (
            self.bt.Layout_Entry("SHARINDX", log2( gp.NO_IN_VLS ), comment="Contains the index to be used by IVR module."),
            self.bt.Layout_Entry("MAXLEN", log2( gp.MAX_FRAME_LENGTH+1 ), comment="Max. length in bytes allowed for this VL.", choices=range(1,gp.MAX_FRAME_LENGTH+1)),
            self.bt.Layout_Entry("1", 1, comment="Idendifies a TT VL", const=1),
        ))

        # The layout of a VL Policing Table for RC VLs
        self.ivp_rc_layout = self.bt.Layout(name = "RC VL", pad_low = True, layout = (
            self.bt.Layout_Entry("JITTER", gp.JITTER_WIDTH, comment="10us granularity. Used only if any entry's nBagIdx field points to this entry."),
            self.bt.Layout_Entry("BAG", gp.BAG_WIDTH, comment="100us granularity. Used only if any entry's nBagIdx field points to this entry."),
            self.bt.Layout_Entry("SHARINDX", log2( gp.NO_IN_VLS ), comment="Points to the entry containing the policing parameters for frames of this VL."),
            self.bt.Layout_Entry("MAXLEN", log2( gp.MAX_FRAME_LENGTH+1 ), comment="Max. length allowed for this VL.", choices=range(1,gp.MAX_FRAME_LENGTH+1)),
            self.bt.Layout_Entry("0", 1, comment="Idendifies a RC VL", const=0),
        ))

        # The VL Policing Table
        self.ivp_table = self.bt.Config_Table(
            name           = name,
            offset         = gp.config_offsets[name],
            size           = gp.NO_IN_VLS,
            layouts        = {"TT": self.ivp_tt_layout,
                              "RC": self.ivp_rc_layout},
            layout_fn      = self._get_ivp_layout,
        )


        ########################################################################
        name = "VL Forwarding Table"
        self.ivr_table_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("DESTPORTS", gp.NO_ETH_PORTS, comment="Vector of ports frames of this VL will flow to."),
            self.bt.Layout_Entry("PARTITION", log2( gp.NO_VL_PARTITIONS ), comment="Memory partition of the VL."),
            self.bt.Layout_Entry("PRIORITY", log2( gp.NO_PRIORITIES ), comment="Priority of the VL at output port."),
            self.bt.Layout_Entry("TYPE", 1, comment="Indicates that the VL flows out TT when set to 1, RC when set to 0."),
        ])

        self.ivr_table = self.bt.Config_Table(
            name           = name,
            offset         = gp.config_offsets[name],
            size           = gp.NO_IN_VLS,
            layouts        = {name: self.ivr_table_layout},
        )


        ########################################################################
        name = "L2 Policing Table"
        self.icp_table_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("PARTITION", log2( gp.NO_COTS_PARTITIONS ), comment="Memory partition of the VL."),
            self.bt.Layout_Entry("MAXLEN", log2( gp.MAX_FRAME_LENGTH+1 ), comment="Max. length in bytes allowed for frames sourced at this port.", choices=range(1,gp.MAX_FRAME_LENGTH+1)),
            self.bt.Layout_Entry("RATE", gp.COTS_RATE_INT_WIDTH + gp.COTS_RATE_FRAC_WIDTH, comment="Transmission rate in Mbps with %d bits fractional part."%gp.COTS_RATE_FRAC_WIDTH),
            self.bt.Layout_Entry("SMAX", gp.COTS_SMAX_WIDTH, comment="Maximum and initial value for bandwidth policing in bytes."),
            self.bt.Layout_Entry("SHARINDX", log2(gp.NO_ICP_ENTRIES+1), comment="Pointer to active rate limiter block for this entry."),
        ])

        self.icp_table = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = gp.NO_ETH_PORTS * (gp.NO_PRIORITIES+1),
            layouts = {name: self.icp_table_layout},
        )


        ########################################################################
        name = "L2 Address Lookup Table"
        self.icl_table_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("INDEX", log2( gp.NO_IN_COTS ), comment="Address within the table to store this entry to."),
            self.bt.Layout_Entry("ENFPORT", 1, comment="Used to enforce the MAC address on PORT (to prevent masquerading)."),
            self.bt.Layout_Entry("DESTPORTS", gp.NO_ETH_PORTS, comment="Destination ports vector for this MAC destination address."),
            self.bt.Layout_Entry("MACADDR", 48, choices=16, comment="Destination MAC address for which the entry will fire."),
            self.bt.Layout_Entry("VLANID", 12, comment="VLAN ID associated with this entry."),
            self.bt.Layout_Entry("IOTAG", 1, comment="."),
            self.bt.Layout_Entry("MASK_MACADDR", 48, comment=".", default=0xFFFFFFFFFFFF),
            self.bt.Layout_Entry("MASK_VLANID", 12, comment=".", default=0xFFF),
            self.bt.Layout_Entry("MASK_IOTAG", 1, comment=".", default=0x1),
            self.bt.Layout_Entry("RETAG", 1, comment="."),
            self.bt.Layout_Entry("MIRR", 1, comment="."),
            self.bt.Layout_Entry("TAKETS", 1, comment="."),
            self.bt.Layout_Entry("MIRRVLAN", 12, comment="."),
            self.bt.Layout_Entry("TSREG", 1, comment="."),
        ])
        self.icl_table = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = gp.NO_IN_COTS,
            layouts = {name: self.icl_table_layout},
        )

        ########################################################################
        name = "L2 Forwarding Table"
        self.icr_table_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("VLAN_PMAP", 3, count=8, comment="Mapping of logical<->logical VLAN priority for first NO_ETH_PORTS entries and logical<->physical queue for last 8 entries"),
            self.bt.Layout_Entry("FL_DOMAIN", gp.NO_ETH_PORTS, comment="Destination vector for non-broadcast traffic not producing a hit in the ICL table."),
            self.bt.Layout_Entry("REACH_PORT", gp.NO_ETH_PORTS, comment="Port reachability vector."),
            self.bt.Layout_Entry("VLAN_BC", gp.NO_ETH_PORTS, comment="Destination vector for broadcast traffic."),
        ])
        self.icr_table = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = gp.NO_ETH_PORTS+8,
            layouts = {name: self.icr_table_layout},
        )

        ########################################################################
        name = "VLAN Lookup Table"
        self.iql_table_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("VLANID", log2( gp.MAX_VLAN_ID+1 ), comment="VLAN ID of the received frame"),
            self.bt.Layout_Entry("TAG_PORT", gp.NO_ETH_PORTS, comment="Defines the set of ports on which the respective VLAN gets untagged."),
            self.bt.Layout_Entry("VLAN_BC", gp.NO_ETH_PORTS, comment="Defines the reachable ports inside this VLAN"),
            self.bt.Layout_Entry("VMEMB_PORT", gp.NO_ETH_PORTS, comment="Defines the set of ports on which frames entering this VLAN may be received"),
            self.bt.Layout_Entry("VEGR_MIRR", gp.NO_ETH_PORTS, comment="Defines if traffic of this VLAN will be routed to the mirror port if routed to any port having its flag set in this vector."),
            self.bt.Layout_Entry("VING_MIRR", gp.NO_ETH_PORTS, comment="Defines if traffic of this VLAN will be routed to the mirror port if received on any port having its flag set in this vector."),
         ])
        self.iql_table = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = gp.NO_IN_VLANS,
            layouts = {name: self.iql_table_layout},
        )

        ########################################################################
        name = "MAC Configuration Table"
        nSpeed_choices = ("Auto", "1Gbit/s", "100Mbit/s", "10Mbit/s")

        layout = [
            self.bt.Layout_Entry("INGMIRRDEI", 1, comment=".", default=0),
            self.bt.Layout_Entry("INGMIRRPCP", 3, comment=".", default=0),
            self.bt.Layout_Entry("INGMIRRVID", 12, comment=".", default=0),
            self.bt.Layout_Entry("MIRRCETAG", 1, comment=".", default=0),
            self.bt.Layout_Entry("MIRRCIE", 1, comment=".", default=0),
            # self.bt.Layout_Entry("abPortState", 8, comment="Port state (0 = input enable, 1 = output enable, 2 = learning enable, 3 = re-tag prio-tagged input, 4 = tagged traffic only, 5 = no single-inner-tagged traffic, 6 = no single-outer-tagged traffic, 7 = no double-tagged traffic)", default=7),
            self.bt.Layout_Entry("INGRESS", 1, comment=".", default=1),
            self.bt.Layout_Entry("EGRESS", 1, comment=".", default=1),
            self.bt.Layout_Entry("DYN_LEARN", 1, comment=".", default=1),
            self.bt.Layout_Entry("RETAG", 1, comment=".", default=0),
            self.bt.Layout_Entry("DRPUNTAG", 1, comment=".", default=0),
            self.bt.Layout_Entry("DRPSITAG", 1, comment=".", default=0),
            self.bt.Layout_Entry("DRPSOTAG", 1, comment=".", default=0),
            self.bt.Layout_Entry("DRPDTAG", 1, comment=".", default=0),
            self.bt.Layout_Entry("DRPNONA664", 1, comment="If set to 1, frames carrying an EtherType other than 0x800 or VLAN-tagged frames will be dropped."),
            self.bt.Layout_Entry("EGR_MIRR", 1, comment="If set to 1, frames forwarded to this port will be mirrored."),
            self.bt.Layout_Entry("ING_MIRR", 1, comment="If set to 1, frames received on this port will be mirrored."),
            self.bt.Layout_Entry("VLANID", log2( gp.MAX_VLAN_ID + 1 ), comment="Deafult VLAN ID of this port."),
            self.bt.Layout_Entry("VLANPRIO", log2( gp.MAX_VLAN_PRIO + 1 ), comment="Default VLAN priority of this port."),
            self.bt.Layout_Entry("MAXAGE", gp.DMM_AGE_TIME_WIDTH, comment="Max. allowed age of frame at transmission start in multiples of 10ms."),
            self.bt.Layout_Entry("TP_DELOUT", log2( gp.MAX_OUT_DELAY+1 ), comment="Transparent clock update at input."),
            self.bt.Layout_Entry("TP_DELIN", log2( gp.MAX_IN_DELAY+1 ), comment="Transparent clock update at output."),
            self.bt.Layout_Entry("SPEED", 2, choices=nSpeed_choices, comment="Speed of the port."),
            self.bt.Layout_Entry("IFG", log2( gp.IFG_MAX+1 ), comment="Additional extension of IFG."),
        ]
        for j in range(gp.NO_PRIORITIES):
            layout.extend([
                self.bt.Layout_Entry("ENABLED[%d]"%j, 1, comment="Indicates if the respective priority has any space at this port."),
                self.bt.Layout_Entry("BASE[%d]"%j, log2( gp.MAX_FRAMES_PER_PORT ), comment="Base pointer of this priority for this port."),
                self.bt.Layout_Entry("TOP[%d]"%j, log2( gp.MAX_FRAMES_PER_PORT ), comment="Max pointer of this priority for this port."),
            ])

        self.mac_table_layout = self.bt.Layout(name = name, pad_low = True, layout = layout)
        self.mac_table = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = gp.NO_ETH_PORTS,
            layouts = {name: self.mac_table_layout},
        )

        ########################################################################
        name = "Retagging Table"
        self.lbp_table_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("DESTPORTS", gp.NO_ETH_PORTS, comment="Dedicated route for frames matching VLAN_EGR if USE_DEST_PORTS is asserted."),
            self.bt.Layout_Entry("USE_DEST_PORTS", 1, comment="Enables dedicated routing for frames matching VLAN_EGR."),
            self.bt.Layout_Entry("DO_NOT_LEARN", 1, comment="Disables address learning for frames carrying VLAN_EGR."),
            self.bt.Layout_Entry("VLAN_EGR", log2( gp.MAX_VLAN_ID+1 ), comment="VLAN ID of the re-tagged frame."),
            self.bt.Layout_Entry("VLAN_ING", log2( gp.MAX_VLAN_ID+1 ), comment="VLAN ID of the original frame."),
            self.bt.Layout_Entry("ING_PORT", gp.NO_ETH_PORTS, comment="Defines the ingress ports for which the respective VLAN frame will get re-tagged."),
            self.bt.Layout_Entry("EGR_PORT", gp.NO_ETH_PORTS, comment="Defines the egress ports for which the respective VLAN frame will get re-tagged."),
         ])
        self.lbp_table = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = gp.NO_RETAG_ENTRIES,
            layouts = {name: self.lbp_table_layout},
        )

        ########################################################################
        name = "L2 Lookup Parameters Table"
        layout = [
            self.bt.Layout_Entry("LEARN_ONCE", 1, comment=".", default=0),
            self.bt.Layout_Entry("OWR_DYN", 1, comment=".", default=1),
            self.bt.Layout_Entry("USE_STATIC", 1, comment=".", default=0),
            self.bt.Layout_Entry("NO_MGMT_LEARN", 1, comment="If asserted, address learning will be de-activated for management frames received at the host port.", default=0),
            self.bt.Layout_Entry("NO_ENF_HOSTPRT", 1, comment="If asserted, port enforcement will be de-activated for management frames received at the host port.", default=0),
            self.bt.Layout_Entry("SHARED_LEARN", 1, comment="If the bit is cleared, the VLAN ID is included in the address learning hash computation.", default=0),
            self.bt.Layout_Entry("DRPNOLEARN", gp.NO_ETH_PORTS, comment=".", default=0),
            self.bt.Layout_Entry("START_DYNSPC", log2( gp.NO_IN_COTS ), comment=".", default=0),
            self.bt.Layout_Entry("MAXAGE", gp.COTS_AGE_MAX_WIDTH, comment="If a lookup entry hits this age, it will be forgotten.", default=0),
        ]
        for j in range(gp.NO_ETH_PORTS):
            layout.extend([
                self.bt.Layout_Entry("MAXADDRP[%d]"%j, 11, comment=".", default=1024),
            ])
        layout.extend([
            self.bt.Layout_Entry("DRPUNI", gp.NO_ETH_PORTS, comment=".", default=0),
            self.bt.Layout_Entry("DRPMC", gp.NO_ETH_PORTS, comment=".", default=0),
            self.bt.Layout_Entry("DRPBC", gp.NO_ETH_PORTS, comment=".", default=0),
        ])

        self.icl_parameters_layout = self.bt.Layout(name = name, pad_low = True, layout = layout)
        self.icl_parameters = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = 1,
            layouts = {name: self.icl_parameters_layout},
        )


        ########################################################################
        name = "Schedule Parameters"
        self.sch_parameters_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("SUBSCHEIND", log2( gp.SCHEDULE_ENTRIES ), count=gp.NO_SUBSCHEDULES, comment="Index of last sub-schedule entry."),
        ])
        self.sch_parameters = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = 1,
            layouts = {name: self.sch_parameters_layout},
        )


        ########################################################################
        name = "Schedule Entry Points Parameters"
        self.sd_parameters_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("ACTSUBSCH", log2( gp.NO_SUBSCHEDULES ), comment="Number of active sub-schedules reduced by 1"),
            self.bt.Layout_Entry("CLKSRC", 2, comment="Sync source type."),
        ])
        self.sd_parameters = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = 1,
            layouts = {name: self.sd_parameters_layout},
        )


        ########################################################################
        name = "VL Forwarding Parameters Table"
        self.ivr_parameters_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("DEBUGEN", 1, comment="Set this flag if you like to have mirring and re-tagging enabled for critical traffic."),
            self.bt.Layout_Entry("PART_SPC", log2( gp.NO_FMM_BUFFERS ), count=gp.NO_VL_PARTITIONS, comment="Number of buffers assigned to this partition."),
        ])
        self.ivr_parameters = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = 1,
            layouts = {name: self.ivr_parameters_layout},
        )


        ########################################################################
        name = "L2 Forwarding Parameters Table"
        self.icr_parameters_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("PART_SPC", log2( gp.NO_FMM_BUFFERS ), count=gp.NO_COTS_PARTITIONS, comment="Number of buffers assigned to this partition."),
            self.bt.Layout_Entry("MAX_DYNP", log2( gp.NO_PRIORITIES ), comment="Maximum allowed priority to be assigned by dynamic reconfiguration."),
        ])
        self.icr_parameters = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = 1,
            layouts = {name: self.icr_parameters_layout},
        )

        ########################################################################
        name = "AVB Parameters"
        self.avb_parameters_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("SRCMETA", 48, comment="Source MAC address of META data frames generated by the switch."),
            self.bt.Layout_Entry("DESTMETA", 48, comment="Destination MAC address of META data frames generated by the switch."),
            self.bt.Layout_Entry("CAS_MASTER", 1, comment="."),
            self.bt.Layout_Entry("L2CBS", 1, comment="."),
        ])
        self.avb_parameters = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = 1,
            layouts = {name: self.avb_parameters_layout},
        )


        ########################################################################
        name = "MII Mode Parameters"
        layout = []
        for j in range(5):
            layout.extend([
                self.bt.Layout_Entry("xMII_MODE[%d]"%j, 2, 1, comment="PHY multiplexor value. 00 = MII, 01 = RMII, 10 = RGMII, 11 = GMII."),
                self.bt.Layout_Entry("PHY_MAC[%d]"%j, 1, 1, comment="0 = MAC mode, 1 = PHY mode."),
                ])
        self.mii_mode_parameters_layout = self.bt.Layout(name = name, pad_low = True, layout = layout)
        self.mii_mode_parameters = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = 1,
            layouts = {name: self.mii_mode_parameters_layout},
        )


        ########################################################################
        name = "CGU Configuration"
        layout = [
            self.bt.Layout_Entry("RFRQ", 32, comment=".", default=0),
            self.bt.Layout_Entry("PLL_0_C", 32, comment=".", default=0),
            self.bt.Layout_Entry("PLL_1_C", 32, comment=".", default=0x0A000083),
        ]
        for j in range(gp.NO_ETH_PORTS):
            layout.extend([
                self.bt.Layout_Entry("IDIV_%d_C"%j, 32, comment=".", default=0x0A000000),
            ])

        layout.extend([
                self.bt.Layout_Entry("BASE_SAFE_CLK_C", 32, comment=".", default=0),
                self.bt.Layout_Entry("BASE_SWITCH_CLK_C", 32, comment=".", default=0),
                self.bt.Layout_Entry("BASE_PERPH_CLK_C", 32, comment=".", default=0),
            ])
        for j in range(gp.NO_ETH_PORTS):
            defaul_val = j * (1 << 24) + 0x11000000
            layout.extend([
                self.bt.Layout_Entry("MII%d_MII_TX_CLK"%j, 32, comment=".", default=defaul_val),
                self.bt.Layout_Entry("MII%d_MII_RX_CLK"%j, 32, comment=".", default=defaul_val),
                self.bt.Layout_Entry("MII%d_RMII_REF_CLK"%j, 32, comment=".", default=defaul_val),
                self.bt.Layout_Entry("MII%d_RGMII_TX_CLK"%j, 32, comment=".", default=defaul_val),
                self.bt.Layout_Entry("MII%d_EXT_TX_CLK"%j, 32, comment=".", default=defaul_val),
                self.bt.Layout_Entry("MII%d_EXT_RX_CLK"%j, 32, comment=".", default=defaul_val)
            ])
        layout.extend([
                self.bt.Layout_Entry("BASE_ISO_CLK_C", 32, comment=".", default=0),
                self.bt.Layout_Entry("BASE_TPR_CLK_C", 32, comment=".", default=0),
                self.bt.Layout_Entry("RBUS", 32, comment=".", default=0),
            ])
        self.cgu_parameters_layout = self.bt.Layout(name = name, pad_low = True, layout = layout)
        self.cgu_parameters = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = 1,
            layouts = {name: self.cgu_parameters_layout},
        )

        ########################################################################

        name = "RGU Configuration"
        layout = []

        layout.extend([
                self.bt.Layout_Entry("DISABLE_IF", 32, comment=".", default=0x0)
            ])
        self.rgu_parameters_layout = self.bt.Layout(name = name, pad_low = True, layout = layout)
        self.rgu_parameters = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = 1,
            layouts = {name: self.rgu_parameters_layout},
        )

        ########################################################################

        name = "ACU Configuration"
        layout = []
        for j in range(gp.NO_ETH_PORTS):
            layout.extend([
                self.bt.Layout_Entry("CFG_PAD_MII%d_TX"%j, 32, comment=".", default=0x12121212),
                self.bt.Layout_Entry("CFG_PAD_MII%d_RX"%j, 32, comment=".", default=0x02020212),
            ])

        for j in range(gp.NO_ETH_PORTS):
            layout.extend([
                self.bt.Layout_Entry("CFG_PAD_MII%d_ID"%j, 32, comment=".", default=0x00002323),
            ])

        layout.extend([
                self.bt.Layout_Entry("CFG_PAD_MISC", 32, comment=".", default=0x00120412),
                self.bt.Layout_Entry("CFG_PAD_SPI", 32, comment=".", default=0x12040407),
                self.bt.Layout_Entry("CFG_PAD_JTAG", 32, comment=".", default=0x02000000),
                self.bt.Layout_Entry("TS_CONFIG", 32, comment=".", default=0x00000065),
                self.bt.Layout_Entry("RGMII_MEAS_SETUP", 32, comment=".", default=0),
                self.bt.Layout_Entry("DISABLE_IF", 32, comment=".", default=0),
            ])

        self.acu_parameters_layout = self.bt.Layout(name = name, pad_low = True, layout = layout)
        self.acu_parameters = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = 1,
            layouts = {name: self.acu_parameters_layout},
        )


        ########################################################################

        name = "SGMII Configuration"
        layout = []

        layout.extend([
            self.bt.Layout_Entry("SR_VSMMD_DEV_ID1", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("SR_VSMMD_DEV_ID2", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("SR_VSMMD_PCS_ID1", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("SR_VSMMD_PCS_ID2", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("SR_VSMMD_CTRL", 32, comment=".",default=0x4),
            self.bt.Layout_Entry("BASIC_CONTROL", 32, comment=".",default=0x1140),
            self.bt.Layout_Entry("AUTONEG_ADV", 32, comment=".",default=0x20),
            self.bt.Layout_Entry("DIGITAL_CONTROL_1", 32, comment=".",default=0x2400),
            self.bt.Layout_Entry("AUTONEG_CONTROL", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("AUTONEG_INTR_STATUS", 32, comment=".",default=0xA),
            self.bt.Layout_Entry("TEST_CONTROL", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("DEBUG_CONTROL", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("VR_MII_EEE_MCTRL", 32, comment=".",default=0x899C),
            self.bt.Layout_Entry("VR_MII_EEE_TXTIMER", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("VR_MII_EEE_RXTIMER", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("VR_MII_LINK_TIMER_CTRL", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("VR_MII_GPIO", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("VR_MII_Gen1_TX_BSTCTRL", 32, comment=".",default=0xA),
            self.bt.Layout_Entry("VR_MII_Gen1_TX_ATTN_CTRL", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("VR_MII_Gen1_TX_GENCTRL", 32, comment=".",default=0x1),
            self.bt.Layout_Entry("VR_MII_Gen1_TX_EDGRT_CTRL", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("VR_MII_Gen1_RXGCTRL", 32, comment=".",default=0x101),
            self.bt.Layout_Entry("VR_MII_Gen1_RXEQ_CTRL", 32, comment=".",default=0x5),
            self.bt.Layout_Entry("VR_MII_Gen1_DPLL_MCTRL", 32, comment=".",default=0x1),
            self.bt.Layout_Entry("VR_MII_Gen1_RDPLL_RST", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("VR_MII_Gen1_RLOS_CTRL", 32, comment=".",default=0x3),
            self.bt.Layout_Entry("VR_MII_Gen1_MPLL_CTRL0", 32, comment=".",default=0x1),
            self.bt.Layout_Entry("VR_MII_Gen1_MPLL_CTRL1", 32, comment=".",default=0x1C22),
            self.bt.Layout_Entry("VR_MII_Gen1_MPLL_CTRL2", 32, comment=".",default=0xA),
            self.bt.Layout_Entry("VR_MII_Gen1_LVL_CTRL", 32, comment=".",default=0x23F),
            self.bt.Layout_Entry("VR_MII_Gen1_MISC_CTRL", 32, comment=".",default=0x100),
            self.bt.Layout_Entry("VR_MII_SNPS_CR_CTRL", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("VR_MII_SNPS_CR_ADDR", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("VR_MII_SNPS_CR_DATA", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("DIGITAL_CONTROL_2", 32, comment=".",default=0x0),
            self.bt.Layout_Entry("DIGITAL_ERROR_CNT", 32, comment=".",default=0x0),
        ])

        self.sgmii_parameters_layout = self.bt.Layout(name = name, pad_low = True, layout = layout)
        self.sgmii_parameters = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = 1,
            layouts = {name: self.sgmii_parameters_layout},
        )

        ########################################################################
        # name = "External Blocks Not Used"
        # self.external_blocks_not_used_layout = self.bt.Layout(name = name, pad_low = True, layout = [
        #     self.bt.Layout_Entry("nMiiMode", 3, count=gp.NO_ETH_PORTS, comment="PHY multiplexor value. -00 = MII, -01 = RMII, -10 = RGMII, -11 = GMII, 0-- = MAC mode, 1-- = PHY mode."),
        # ])
        # self.external_blocks_not_used = self.bt.Config_Table(
        #     name    = name,
        #     offset  = gp.config_offsets[name],
        #     size    = 1,
        #     layouts = {name: self.external_blocks_not_used_layout},
        # )


        ########################################################################
        name = "Clock Synchronization Parameters"
        layout = [
            self.bt.Layout_Entry("OUTPRTMASTER", 1, count=gp.NO_ETH_PORTS, comment=""),
            self.bt.Layout_Entry("OUTPRTSLAVE", 1, count=gp.NO_ETH_PORTS, comment=""),
            self.bt.Layout_Entry("SRCPORT", log2( gp.NO_ETH_PORTS+1 ), count=gp.MAX_CLOCK_MASTERS, comment=""),
            self.bt.Layout_Entry("FULLCBG", 1, comment=""),
            self.bt.Layout_Entry("SWMASTER", 1, comment=""),
            self.bt.Layout_Entry("STABASYEN", 1, comment=""),
            self.bt.Layout_Entry("IPCFRAMESY", 1, comment=""),
            self.bt.Layout_Entry("SYASYEN", 1, comment=""),
            self.bt.Layout_Entry("SYSYEN", 1, comment=""),
            self.bt.Layout_Entry("SYRELEN", 1, comment=""),
            self.bt.Layout_Entry("SYTOSTBEN", 1, comment=""),
            self.bt.Layout_Entry("ASYTENSYEN", 1, comment=""),
            self.bt.Layout_Entry("TENTSYRELEN", 1, comment=""),
            self.bt.Layout_Entry("VLIDSELECT", 1, comment=""),
            self.bt.Layout_Entry("ACCDEVWIN", gp.SW_ACC_WND_HALF_WIDTH, comment=""),
            self.bt.Layout_Entry("CAENTMOUT", gp.SW_CA_ENABLED_TIMEOUT_WIDTH, comment=""),
            self.bt.Layout_Entry("VLIDINMAX", gp.VL_ID_SIZE, comment=""),
            self.bt.Layout_Entry("VLIDIMNMIN", gp.VL_ID_SIZE, comment=""),
        ]
        for j in range(gp.NO_ETH_PORTS):
            layout.extend([
                self.bt.Layout_Entry("VLIDOUT[%d]"%j, gp.VL_ID_SIZE, comment=".", default=0),
            ])
        layout.extend([
            self.bt.Layout_Entry("INTTOSYNCTH", gp.SW_MS_THRESHOLD_WIDTH, comment=""),
            self.bt.Layout_Entry("INTTOTENTTH", gp.SW_MS_THRESHOLD_WIDTH, comment=""),
            self.bt.Layout_Entry("INTCYDUR", gp.SW_INTEGRATION_CYCLE_DURATION_WIDTH, comment=""),
            self.bt.Layout_Entry("LISTENTMOUT", gp.SW_LISTEN_TIMEOUT_WIDTH, comment=""),
            self.bt.Layout_Entry("MAXINTEGCY", gp.PCF_INTEGRATION_CYCLE_WIDTH, comment=""),
            self.bt.Layout_Entry("MAXTRANSPCLK", gp.PCF_TRANSPARENT_CLOCK_WIDTH, comment=""),
            self.bt.Layout_Entry("NUMSTBCY", gp.SW_NUM_STABLE_CYCLES_WIDTH, comment=""),
            self.bt.Layout_Entry("NUMUNSTBCY", gp.SW_NUM_UNSTABLE_CYCLES_WIDTH, comment=""),
            self.bt.Layout_Entry("OBVWINSZ", gp.SW_OBSERVATION_WINDOW_SIZE_WIDTH, comment=""),
            self.bt.Layout_Entry("PCFPRIORITY", log2( gp.NO_PRIORITIES+1 ), comment="", default=gp.NO_PRIORITIES+1),
            self.bt.Layout_Entry("PCFSZE", log2( gp.MAX_FRAME_LENGTH+1 ), comment="", default=64),
            self.bt.Layout_Entry("STTOINTTH", gp.SW_MS_THRESHOLD_WIDTH, comment=""),
            self.bt.Layout_Entry("STTH", gp.SW_MS_THRESHOLD_WIDTH, comment=""),
            self.bt.Layout_Entry("SYDOMAIN", gp.PCF_SYNC_DOMAIN_WIDTH, comment=""),
            self.bt.Layout_Entry("SYPRIORITY", gp.PCF_SYNC_PRIORITY_WIDTH, comment=""),
            self.bt.Layout_Entry("SYTOUSYTH", gp.SW_MS_THRESHOLD_WIDTH, comment=""),
            self.bt.Layout_Entry("SYTH", gp.SW_MS_THRESHOLD_WIDTH, comment=""),
            self.bt.Layout_Entry("TSYTOUSYTH", gp.SW_MS_THRESHOLD_WIDTH, comment=""),
            self.bt.Layout_Entry("TSYTH", gp.SW_MS_THRESHOLD_WIDTH, comment=""),
            self.bt.Layout_Entry("TSYTOSYTH", gp.SW_MS_THRESHOLD_WIDTH, comment=""),
            self.bt.Layout_Entry("UNSYTOSYTH", gp.SW_MS_THRESHOLD_WIDTH, comment=""),
            self.bt.Layout_Entry("UNSYTOTSYTH", gp.SW_MS_THRESHOLD_WIDTH, comment=""),
            self.bt.Layout_Entry("WFINTMOUT", gp.SW_WAIT_4_IN_0_TIMEOUT_WIDTH, comment=""),
            self.bt.Layout_Entry("WAITTHSYNC", gp.SW_MS_THRESHOLD_WIDTH, comment=""),
            self.bt.Layout_Entry("ETSSRCPCF", gp.ETH_ADDR_WIDTH, comment=""),
        ])
        self.css_parameters_layout = self.bt.Layout(name = name, pad_low = True, layout = layout)
        self.css_parameters = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = 1,
            layouts = {name: self.css_parameters_layout},
        )


        ########################################################################
        name = "General Parameters"
        self.cc_parameters_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("REPLAY_PORT", log2( gp.NO_ETH_PORTS+1 ), comment=".", default=7),
            self.bt.Layout_Entry("EGRMIRRDEI", 1, comment=".", default=0),
            self.bt.Layout_Entry("EGRMIRRPCP", 3, comment=".", default=0),
            self.bt.Layout_Entry("EGRMIRRVID", 12, comment=".", default=0),
            self.bt.Layout_Entry("QUEUE_TS", 1, comment=".", default=0),
            self.bt.Layout_Entry("TPID2", 16, comment="Ethernet type identifying VLAN double-tagged packets (VLAN inner tag).", default=0x8100),
            self.bt.Layout_Entry("IGNORE2STF", 1, comment="Set this to true to run legacy mode (i.e. all 1588 V2 event messages will receive a transparent clock update.", default=1),
            self.bt.Layout_Entry("TPID", 16, comment="Ethernet type identifying VLAN tagged packets (VLAN outer tag).", default=0x9100),
            self.bt.Layout_Entry("VIMASK", gp.VL_MARKER_SIZE, comment="Filter the bits that will be compared against VIMARKER.", default=2**gp.VL_MARKER_SIZE-1),
            self.bt.Layout_Entry("VIMARKER", gp.VL_MARKER_SIZE, comment="Upper 32 bits of VL MAC destination address in network byte order."),
            self.bt.Layout_Entry("MIRR_PORT", log2( gp.NO_ETH_PORTS+1 ), comment="If this is a valid port number, mirroring is enabled.", default=gp.NO_ETH_PORTS),
            self.bt.Layout_Entry("HOST_PORT", log2( gp.NO_ETH_PORTS+1 ), comment="If this is a valid port number, bridge traffic will be routed to this port (will be dropped otherwise).", default=gp.NO_ETH_PORTS),
            self.bt.Layout_Entry("CASC_PORT", log2( gp.NO_ETH_PORTS+1 ), comment="If this is a valid port number, groupcast frames received on this port get directly forwarded to the host.", default=gp.NO_ETH_PORTS),
            self.bt.Layout_Entry("SEND_META", 1, count=gp.NO_GROUPCAST_FILTER, comment="If this bit is set, a follow up frame containing the TS will be sent for frames matching uvMacFilter", default=0),
            self.bt.Layout_Entry("INCL_SRCPT", 1, count=gp.NO_GROUPCAST_FILTER, comment="If this bit is set, the source port ID will be included in the frame destination mac", default=0),
            self.bt.Layout_Entry("MAC_FLT", 48, count=gp.NO_GROUPCAST_FILTER, comment="Contains the filter mask for groupcast addresses", default=0xFFFFFF000000),
            self.bt.Layout_Entry("MAC_FLTRES", 48, count=gp.NO_GROUPCAST_FILTER, comment="MAC & MAC_FLT(i) = MAC_FLTRES(i) identifies a frame to be groupcast", default=0x0180C2000000),
            self.bt.Layout_Entry("HOSTPRIO", log2( gp.NO_PRIORITIES ), comment="CoS queue used for groupcast frames."),
            self.bt.Layout_Entry("SWITCHID", log2( gp.NO_CASCADED_DEV ), comment="ID of the device in case of cascading."),
            self.bt.Layout_Entry("MIRR_PTACU", 1, comment="When asserted, the host can dynamically change the value of MIRR_PORT at run-time."),
            self.bt.Layout_Entry("VLLUPFORMAT", 1, comment="Specifying if the ARINC664 scheme shall be used for VL identification. If so, the lower 16 bits of uvDstMacAddr needs to contain the VL ID.", default=1),
        ])
        self.cc_parameters = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = 1,
            layouts = {name: self.cc_parameters_layout},
        )

        ########################################################################
        name = "Credit-Based Shaping Table"
        self.cbs_table_layout = self.bt.Layout(name = name, pad_low = True, layout = [
            self.bt.Layout_Entry("IDLE_SLOPE", 32, comment="."),
            self.bt.Layout_Entry("SEND_SLOPE", 32, comment="."),
            self.bt.Layout_Entry("CREDIT_HI", 32, comment=".", default=0x7FFFFFFF),
            self.bt.Layout_Entry("CREDIT_LO", 32, comment=".", default=0x7FFFFFFF),
            self.bt.Layout_Entry("CBS_PRIO", 3, comment=".", default=0),
            self.bt.Layout_Entry("CBS_PORT", 3, comment=".", default=0),
        ])
        self.cbs_table = self.bt.Config_Table(
            name    = name,
            offset  = gp.config_offsets[name],
            size    = gp.NO_CBS_BLOCKS,
            layouts = {name: self.cbs_table_layout},
        )


        ########################################################################

        # A list of all configuration tables in the ES
        self.switch_table_list = (
            self.sch_table,
            self.sd_table,
            self.ivl_table,
            self.ivp_table,
            self.ivr_table,
            self.mac_table,
            self.lbp_table,
            self.sch_parameters,
            self.sd_parameters,
            self.ivr_parameters,
            self.css_parameters,
            self.cc_parameters,
            self.icl_table,
            self.iql_table,
            self.icl_parameters,
            self.icp_table,
            self.icr_parameters,
            self.avb_parameters,
            self.mii_mode_parameters,
            # The following tables are not in the hex by default
            #self.cgu_parameters,
            #self.rgu_parameters,
            #self.acu_parameters,
            #self.sgmii_parameters,
            #self.cbs_table,
        )
    # def update

# class Switch_Layouts

########################################################################

# create the default parameter set
switch_gp       = Switch_Global_Parameters ()
switch_layouts  = Switch_Layouts (bit_table=common.bit_table, gp=switch_gp)
