############################################################################
#Test configuration where priority map is retained but all prios are put in a single
#egress queue

############################################################################

import sys
import os.path
import shutil
import pprint
local_dir = os.path.dirname(sys.argv[0])
sys.path.append(os.path.join(local_dir, "low_level_config"))

from snic_switch.tables import *
from snic_switch.config import Switch_Config

from common import gui_base

from copy import copy

# Create an empty configuration with the modified configuration
swc = Switch_Config()
hex_config = 'cfg_applicationBoard_SJA1105QS.hex'

swc.load_nvm_file(hex_config)


print (swc)

app = gui_base.Hex_Gui_App( "SJA110PQRS Hex Diagnosis", swc)
app.OnInit()
app.MainLoop()
