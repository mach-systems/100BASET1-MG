class Switch_Global_Parameters:
    def __init__(self):
        self.BAG_WIDTH                              = 14
        self.COTS_AGE_MAX_WIDTH                     = 15
        self.COTS_RATE_INT_WIDTH                    = 10
        self.COTS_RATE_FRAC_WIDTH                   = 6
        self.COTS_SMAX_WIDTH                        = 16
        self.DEVICE_ID                              = 174
        self.DMM_AGE_TIME_WIDTH                     = 8
        self.ETH_ADDR_WIDTH                         = 48
        self.IFG_MAX                                = 31
        self.JITTER_WIDTH                           = 10
        self.MAX_CLOCK_MASTERS                      = 8
        self.MAX_DELTA                              = 262143
        self.MAX_FRAME_LENGTH                       = 2047
        self.MAX_FRAMES_PER_PORT                    = 512
        self.MAX_IN_DELAY                           = 65535
        self.MAX_OUT_DELAY                          = 65535
        self.NO_BUFFERS                             = 1024
        self.NO_COTS_PARTITIONS                     = 8
        self.NO_FMM_BUFFERS                         = 1024
        self.NO_ETH_PORTS                           = 5
        self.NO_CASCADED_DEV                           = 8
        self.NO_ICP_ENTRIES                         = 45
        self.NO_IN_COTS                             = 1024
        self.NO_IN_COTS_ROWS                        = 256
        self.NO_IN_COTS_WAYS                        = 4
        self.NO_IN_VLS                              = 1024
        self.NO_IN_VLANS                            = 4096
        self.NO_PRIORITIES                          = 8
        self.NO_RETAG_ENTRIES                       = 32
        self.NO_SUBSCHEDULES                        = 8
        self.NO_VL_PARTITIONS                       = 8
        self.NO_GROUPCAST_FILTER                    = 2
        self.PCF_INTEGRATION_CYCLE_WIDTH            = 8
        self.PCF_SYNC_DOMAIN_WIDTH                  = 4
        self.PCF_SYNC_PRIORITY_WIDTH                = 2
        self.PCF_TRANSPARENT_CLOCK_WIDTH            = 27
        self.AHB_INTERFACE_REV                      = 3
        self.CFG_FORMAT_REV                         = 14
        self.SCHEDULE_ENTRIES                       = 1024
        self.SW_ACC_WND_HALF_WIDTH                  = 15
        self.SW_CA_ENABLED_TIMEOUT_WIDTH            = 20
        self.SW_INTEGRATION_CYCLE_DURATION_WIDTH    = 27
        self.SW_LISTEN_TIMEOUT_WIDTH                = 30
        self.SW_MS_THRESHOLD_WIDTH                  = 4
        self.SW_NUM_STABLE_CYCLES_WIDTH             = 7
        self.SW_NUM_UNSTABLE_CYCLES_WIDTH           = 7
        self.SW_OBSERVATION_WINDOW_SIZE_WIDTH       = 15
        self.SW_WAIT_4_IN_0_TIMEOUT_WIDTH           = 20
        self.VL_ID_SIZE                             = 16
        self.VLAN_ID_SIZE                           = 12
        self.VL_MARKER_SIZE                         = 32
        self.MAX_VLAN_ID                            = 4095
        self.MAX_VLAN_PRIO                          = 7
        self.NO_CBS_BLOCKS                          = 16
 
        # Block IDs of the configuration tables of the switch 
        self.config_offsets = {
            "Schedule Table"                    : 0,
            "Schedule Entry Points Table"       : 1,
            "VL Lookup Table"                   : 2,
            "VL Policing Table"                 : 3,
            "VL Forwarding Table"               : 4,
            "L2 Address Lookup Table"           : 5,
            "L2 Policing Table"                 : 6,
            "VLAN Lookup Table"                 : 7,
            "L2 Forwarding Table"               : 8,
            "MAC Configuration Table"           : 9,
            "Retagging Table"                   : 18,
            "Schedule Parameters"               : 10,
            "Schedule Entry Points Parameters"  : 11,
            "VL Forwarding Parameters Table"    : 12,
            "L2 Lookup Parameters Table"        : 13,
            "L2 Forwarding Parameters Table"    : 14,
            "AVB Parameters"                    : 16,
            "Clock Synchronization Parameters"  : 15,
            "General Parameters"                : 17,
            "Credit-Based Shaping Table"        : 19,
            "MII Mode Parameters"               : 78,
            "CGU Configuration"                 : 128,
            "RGU Configuration"                 : 129,
            "ACU Configuration"                 : 130,
            "SGMII Configuration"               : 200,
            # "External Blocks Not Used"        : 148,
            None                                : 201,
        }
    # def __init__
# class Switch_Global_Parameters
