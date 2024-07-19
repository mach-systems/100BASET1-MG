import wx
from types import *

# The window will contain a single horizontal sizer with a record type
# choice to the left. The remaining controls in the window will be 
# created/updated depending on the record type choice. 
class Multi_Layout_Window(wx.Window):

    # Arguments:
    #  table - instance of Config_Table that defines the allowed layouts
    #  data - dictionary of input/output data {"name": value}
    def __init__(self, table, data, parent, id=wx.ID_ANY, 
                 pos=wx.DefaultPosition, size=wx.DefaultSize, 
                 style=wx.BORDER_NONE):
        wx.Window.__init__(self, parent, id, pos, size, style)
        self.table = table
        self.data  = data
        
        # Create a horizontal sizer
        self.sizer = wx.BoxSizer(wx.HORIZONTAL)
        
        # The first element is a record type choice
        self.choice = wx.Choice(self, wx.ID_ANY, 
                                choices=self.table.layouts.keys())
        self.choice.SetSelection(0) # FIXME
        self.Bind(wx.EVT_CHOICE, self.on_type_changed, self.choice)
        self.sizer.Add(self.choice)

        # The remaining elements are based on the record type
        self.controls = []
        self._update_controls()

        # Use the sizer
        self.SetSizerAndFit(self.sizer)
    # def __init__

    
    def on_type_changed(self, event):
        self._update_controls()
        self.GetSizer().Layout()
    # def on_type_changed


    # Delete the old controls and create new controls when the 
    # record type changed
    def _update_controls(self):
        layout = self.table.layouts[self.choice.GetStringSelection()]
        sizer = self.sizer

        # Delete the old controls
        for ctrl in self.controls:
            sizer.Detach(ctrl)
            ctrl.Destroy()
        self.controls = []

        # Create new controls
        for field in layout.layout:
            bin_value     = self.data.get(field.name, 0)
            if not field.name :
                continue
            elif not field.choices:
                # binary number
                value     = str(bin_value)
#                validator = wx.TextValidator(wx.FILTER_NUMERIC, value)
                ctrl      = wx.TextCtrl(self, wx.ID_ANY, value=value)#, validator=validator)
            elif type(field.choices) in (ListType, TupleType):
                # list of allowed values, position is the binary representation
                ctrl = wx.wxChoice(self, wx.ID_ANY, choices=field.choices)
                ctrl.Select(bin_value)
            elif type(field.choices) == DictType:
                # dictionary of allowed values, key is the binary representation
                ctrl = wx.wxChoice(self, wx.ID_ANY, choices=field.choices.values())
                ctrl.SetStringSelection(field.choices[bin_value])
            else:
                print "Invalid choices type: %s" % type(field.choices)
                continue
            # Add the new field
            sizer.Add(ctrl)
            sizer.Layout()
            self.controls.append(ctrl)
    # def _update_controls

# class Multi_Layout_Window


# A panel that contains multiple rows of mutli-layout windows
class Multi_Layout_Panel(wx.Panel):
    
    # Arguments:
    #  table - instance of Config_Table that defines the allowed layouts
    #  data - list of dictionaries of input/output data {"name": value}
    def __init__(self, table, data, parent, id):
        wx.Panel.__init__(self, parent, id)
        self.table = table
        self.data  = data

        #XXX: REMOVE
        if len(data) < 10:
            for i in range(10-len(data)):
                data.append({})

        sizer = wx.FlexGridSizer(cols=2);
        for ofs, d in enumerate(data):
            w = Multi_Layout_Window(table, d, self)
            sizer.Add(wx.StaticText(self, wx.ID_ANY, str(ofs)), 0)
            sizer.Add(w,1)
        self.SetSizerAndFit(sizer)

