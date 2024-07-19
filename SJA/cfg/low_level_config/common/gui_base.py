#! /usr/bin/env python
# -*- coding: iso-8859-1 -*-
########################################################################
# File name:       gui_base.py
# Author:          Petr Grillinger
# Revision:        $Revision: 1.10 $
# Last changed by: $Author: nxp30460 $
# Last chanded on: $Date: Fri Jul  1 11:30:45 2016 $
# Description:     wxWidgets support for basic low-level editing and viewing
########################################################################

import wx
import wx.aui
import wx.grid
from types import *

# Show a message diaglog
def MsgDlg(window, string, caption="", style=wx.YES_NO|wx.CANCEL):
    dlg = wx.MessageDialog(window, string, caption, style)
    result = dlg.ShowModal()
    dlg.Destroy()
    return result


# Generic table base for data formated according to a Layout class
# Expects the following methods to be defined in its descendants:
#   get_key(self, row, col)
#   get_data(self, row, col, key)
#   set_data(self, row, col, value)
#   IsEmptyCell(self, row, col)
class Common_Table_Base(wx.grid.PyGridTableBase):
    def __init__(self, table, data, fixed_size):
        wx.grid.PyGridTableBase.__init__(self)
        self.table       = table
        self.layout      = table.layout
        self.data        = data
        self.fixed_size  = fixed_size

        self.layout_attr  = []
        self.layout_types = []
        self.layout_names = []
        for le in self.layout.num_lookup:
            if not le.choices:
                t = wx.grid.GRID_VALUE_NUMBER
            elif type(le.choices) == IntType and le.choices != 10:
                t = wx.grid.GRID_VALUE_STRING
            elif type(le.choices) == DictType:
                t = wx.grid.GRID_VALUE_CHOICE + ':' + ','.join(le.choices.values())
            else:
                t = wx.grid.GRID_VALUE_CHOICE + ':' + ','.join(map(str,le.choices))
            self.layout_types.append(t)
            self.layout_names.append(le.full_name)
            attr = wx.grid.GridCellAttr()
            attr.SetAlignment(wx.RIGHT, wx.CENTRE)
            self.layout_attr.append(attr)
    # def __init__

    def GetNumberCols(self):
        return len(self.col_names)

    def GetValue(self, row, col):
        key = self.get_key(row, col)

        # get the contents of the cell
        if self.IsEmptyCell(row, col):
            if self.layout[key].default != None:
                v = self.layout[key].default
            else:
                v = 0
        else:
            v = self.get_data(row, col, key)

        # convert the binary representation to string if necessary
        choices = self.layout[key].choices
        if   type(choices) == IntType and choices == 16:
            return "0x%X" % v
        elif type(choices) in (DictType, ListType, TupleType):
            return choices[v]
        else:
            return abs(v)

    def SetValue(self, row, col, val):
        if type(val) in StringTypes:
            choices = self.layout[self.get_key(row,col)].choices
            if type(choices) in (NoneType, IntType):
                if val[0:2] == "0x":
                    val = int(val[2:], 16)
                else:
                    val = int(val)
            elif type(choices) == ListType:
                val = choices.index(val)
            elif type(choices) == TupleType:
                val = list(choices).index(val)
            elif type(choices) == DictType:
                val = filter(lambda x: choices[x] == val, choices.keys())
                if val:
                    val = val[0]
                else:
                    val = 0
        self.set_data(row, col, val)

    def GetColLabelValue(self, col):
        return self.col_names[col]

    def GetTypeName(self, row, col):
        return self.col_types[col]

    def GetAttr(self, row, col, kind):
        self.layout_attr[col].IncRef()
        return self.layout_attr[col]
# class Common_Table_Base


# Table base for a list/tuple/dict of data
class Iterable_Table_Base(Common_Table_Base):

    def __init__(self, table, data, fixed_size=True):
        Common_Table_Base.__init__(self, table, data, fixed_size)
        self.col_types = self.layout_types
        self.col_names = self.layout_names
        self.first_row = table.first_index
        # the following method depends on data type
        if hasattr(data, "__getitem__") and hasattr(data, "keys"):
            self.IsEmptyCell      = self._IsEmptyCell_dict
        else:
            self.IsEmptyCell      = self._IsEmptyCell_list

    def get_key(self, row, col):
        return self.col_names[col]

    def get_data(self, row, col, key):
        return self.data[row+self.first_row][key]

    def set_data(self, row, col, val):
        self.data[row+self.first_row][self.get_key(row, col)] = val

    def GetNumberRows(self):
        if self.fixed_size:
            return self.table.size
        else:
            return len(self.data)

    def GetRowLabelValue(self, row):
        if self.table.name == "Buffer Control":
            if row < 4:
                s = {0: "Shared", 1: "BG 0", 2: "BG 1", 3: "BG 2"}[row]
            else:
                s = str(row-4)
        else:
            s = str(row + self.table.first_index)
        return s

    def _IsEmptyCell_list(self, row, col):
        key = self.get_key(row, col)
        return row >= len(self.data) or key not in self.data[row+self.first_row]

    def _IsEmptyCell_dict(self, row, col):
        key = self.get_key(row, col)
        return row not in self.data or key not in self.data[row]
# class Iterable_Table_Base


# Table base for a dictionary of data with multiple possible layouts
#   First column is the layout of the entry
#   Second column is a compacted version of the other fields (read-only)
# Editing is possible by opening a dedicated edit window
class Multi_Layout_Table_Base(Iterable_Table_Base):
    def __init__(self, table, data):
        Iterable_Table_Base.__init__(self, table, data)
        l_names = sorted(table.layouts.keys())
        self.col_types = [
            wx.grid.GRID_VALUE_CHOICE + ':' + ','.join(l_names),
            wx.grid.GRID_VALUE_STRING ]
        self.col_names = ["Layout Type", "Data"]
        self.layout_attr = [wx.grid.GridCellAttr(),wx.grid.GridCellAttr()]
        self.layout_attr[0].SetAlignment(wx.LEFT, wx.CENTRE)
        self.layout_attr[1].SetAlignment(wx.LEFT, wx.CENTRE)
        self.layout_attr[1].SetReadOnly(True)
        # override the method from parent
        self.IsEmptyCell = self._IsEmptyCell

    def GetValue(self, row, col):
        key = self.get_key(row, col)
        
        empty = self.IsEmptyCell(row, col)
        if empty:
            layout = self.table.default_layout
        else:
            layout = self.data[row]["_type"]
        if col == 0:  # layout type
            return layout.name
        elif not empty:
            return layout.to_string(self.data[self.first_row+row])
        else:
            return ""

    def SetValue(self, row, col, val):
        if col == 0:  # layout type
            layout = self.table.layouts[val]
            if self.IsEmptyCell(row, col):
                self.data[row] = {}
            self.data[row]["_type"] = layout
        else:   # contents is not mutable
            assert False

    def _IsEmptyCell(self, row, col):
        if type(self.data) is dict:
            return row not in self.data
        else:
            return row >= len(self.data)
# class Multi_Layout_Table_Base


class Single_Table_Base(Common_Table_Base):
    def __init__(self, table, data):
        Common_Table_Base.__init__(self, table, data, True)
        self.col_names = ["Value"]
        # Create name/type strings for the rows
        self.row_types = self.layout_types
        self.row_names = self.layout_names

    def get_key(self, row, col):
        return self.row_names[row]

    def get_data(self, row, col, key):
        return self.data[key]

    def set_data(self, row, col, val):
        self.data[self.get_key(row, col)] = val

    def GetNumberRows(self):
        return len(self.row_names)

    def IsEmptyCell(self, row, col):
        key = self.get_key(row, col)
        return key not in self.data

    def GetRowLabelValue(self, row):
        return self.row_names[row]

    def GetTypeName(self, row, col):
        return self.row_types[row]

    def GetAttr(self, row, col, kind):
        self.layout_attr[row].IncRef()
        return self.layout_attr[row]
# class Single_Table_Base


# Display contents of a table in a single grid
# Used for tables that have a single part
class Table_Window(wx.grid.Grid):
    def __init__(self, table, data, parent):
        wx.grid.Grid.__init__(self, parent, -1, 
            size  = (240,380),
            style = wx.BORDER_NONE|wx.WANTS_CHARS)

        # Select the base table according to the layout
        if table.size == 1:
            self.base = Single_Table_Base(table, data)
        elif len(table.layouts) > 1:
            self.base = Multi_Layout_Table_Base(table, data)
        elif hasattr(data, "__getitem__"):
            self.base = Iterable_Table_Base(table, data, False)
        else:
            assert False

        self.SetTable(self.base, True)

        # Automatically compute the column width and header height
        self.SetRowLabelSize(-1)
        self.SetColLabelSize(-1)
        self.AutoSizeColumns(True)

        # Set alignment for the row headers
        self.SetRowLabelAlignment(wx.LEFT, wx.CENTRE)

        # Compute the required window size to show all columens
        size     = self.GetSize()
        cli_size = self.GetClientSize()
        width  = self.GetRowLabelSize()
        width += sum(map(self.GetColSize, range(self.GetNumberCols())))
        width += size[0] - cli_size[0]
        width += 30
        width = min(width, 800)
        cli_size = parent.GetClientSize()
        if width > cli_size[0]:
            cli_size[0] = width
            parent.SetClientSize(cli_size)
        self.SetSize([width, 380])
    # def __init__
# class Table_Window


# Display contents of a list of tables in a notebook of grids
# Used for tables that have multiple parts (e.g. schedule table)
class Tabbed_Table_Window(wx.aui.AuiNotebook):
    def __init__(self, hex_tab, parent):
        wx.aui.AuiNotebook.__init__(self, 
            parent = parent, 
            size   = (300,160),
            style  = wx.aui.AUI_NB_TOP|wx.aui.AUI_NB_TAB_SPLIT|wx.aui.AUI_NB_TAB_MOVE|wx.aui.AUI_NB_SCROLL_BUTTONS)
        self.hex_tab = hex_tab
        self.tab_num = hex_tab.table.parts
        for i in range(self.tab_num):
            tab_win = Table_Window(
                table      = hex_tab.table, 
                data       = hex_tab.data[i], 
                parent     = parent)
            self.AddPage(tab_win, "S%d" % i)
# class Tabbed_Table_Window


# Main application window (managed by AUI)
class Main_Frame(wx.Frame):

    def __init__(self, title, hex_config, parent=None, file_name=None, start_addr=True):
        wx.Frame.__init__(self, 
            parent = parent, 
            title  = title, 
            style  = wx.DEFAULT_FRAME_STYLE|wx.BORDER_SUNKEN )

        # initialize local variables
        self.title      = title
        self.file_name  = ""
        self.config     = hex_config
        self.frames     = []
        self.start_addr = start_addr

        self.setup_gui()
        if file_name:
            self.open_file(file_name)
    # def __init__

    def setup_gui(self):
        # tell FrameManager to manage this frame        
        self.aui_mgr = wx.aui.AuiManager()
        self.aui_mgr.SetManagedWindow(self)

        # Create menu
        self.menubar = wx.MenuBar()

        # Create a file menu
        menu = wx.Menu()
        item = menu.Append(wx.ID_OPEN, '&Open', 'Open configuration file')
        self.Bind(wx.EVT_MENU, self.OnOpen, item)
        item = menu.Append(wx.ID_ANY, '&Save', 'Save configuration file')
        self.Bind(wx.EVT_MENU, self.OnSave, item)
        item = menu.Append(wx.ID_NEW, '&New', 'Start a new configuration')
        self.Bind(wx.EVT_MENU, self.OnNew, item)
        item = menu.Append(wx.ID_EXIT, 'E&xit', 'Exit program')
        self.Bind(wx.EVT_MENU, self.OnExit, item)
        self.menubar.Append(menu, '&File')

        # Create a configuration table menu (dynamically)
        menu = wx.Menu()
        self.tab_id = {}  # mapping between wxID and table
        for tm in self.config.table_map:
            # assign a new wxID and store it
            id = wx.NewId()
            self.tab_id[id] = tm
            # create the menu entry
            item = menu.Append(id, tm.table.name, 'Open table %s' % tm.table.name)
            self.Bind(wx.EVT_MENU, self.OnOpenTable, item)
        self.menubar.Append(menu, '&Table')

        # Attach the menu bar to the window.
        self.SetMenuBar(self.menubar)

        # Close frame handler
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        # initial perspective
        self.perspective_init = self.aui_mgr.SavePerspective()
        self.aui_mgr.Update()
    # def setup_gui

    ############################################################################  
    # Data manipulation
    ############################################################################  

    def open_file(self, file_name):
        try:
            self.config.clear()
            self.config.load_nvm_file(file_name, start_addr=self.start_addr)
            self.file_name = file_name
            self.SetTitle("%s - %s" % (self.title,file_name))
            self.refresh_gui()
        except IOError:
            MsgDlg(self, 'Cannot open file.', 'Error!', wx.OK)

    def save_file(self, file_name):
        try:
            self.config.create_nvm_file(file_name)
            self.file_name = file_name
            self.SetTitle("%s - %s" % (self.title,file_name))
        except IOError:
            MsgDlg(self, 'Cannot save file.', 'Error!', wx.OK)

    def refresh_gui(self):
        for i in self.frames:
            self.aui_mgr.DetachPane(i)
            i.Destroy()
        self.frames = []
        self.aui_mgr.Update()

    ############################################################################  
    # Event handlers
    ############################################################################  

    def OnOpen(self, event):
        dlg = wx.FileDialog(self, 'Choose an input file', '.', self.file_name, '*.hex', wx.OPEN)
        if dlg.ShowModal() == wx.ID_OK:
            self.open_file(dlg.GetPath())
        dlg.Destroy()

    def OnNew(self, event):
        self.file_name = ""
        self.config.clear()
        self.refresh_gui()

    def OnSave(self, event):
        dlg = wx.FileDialog(self, 'Select an output file', '.', self.file_name, '*.hex', wx.SAVE)
        if dlg.ShowModal() == wx.ID_OK:
            self.save_file(dlg.GetPath())
        dlg.Destroy()

    # Close request - can rejected
    def OnExit(self, event):
        self.Close()

    # Clean-up when the application terminates
    def OnClose(self, event):
        self.aui_mgr.UnInit()
        self.Destroy()

    def OnOpenTable(self, event):
        tm   = self.tab_id[event.GetId()]
        name = tm.table.name
        # Try to find the pane
        pane_inf = self.aui_mgr.GetPane(name)
        if pane_inf.IsOk():
            pane_inf.Show()
        else: # Pane not found, create it
            if tm.table.parts == 1:
                tab_frame = Table_Window(parent=self, table=tm.table, data=tm.data)
            else:
                tab_frame = Tabbed_Table_Window(parent=self, hex_tab=tm)
            # register the Pane
            self.aui_mgr.AddPane(tab_frame, wx.aui.AuiPaneInfo().Name(name).
                Caption(name).CloseButton(True).MaximizeButton(True).
                Dockable().Floatable().Float().Left().Row(len(self.frames))
            )
            self.frames.append(tab_frame)
        self.aui_mgr.Update()


class Hex_Gui_App(wx.App):
    def __init__(self, title, hex_config, file_name=None, start_addr=True):
        self.title      = title
        self.hex_config = hex_config
        self.file_name  = file_name
        self.start_addr = start_addr
        wx.App.__init__(self, 0)

    def OnInit(self):
        frame = Main_Frame(parent=None, title=self.title, hex_config=self.hex_config, file_name=self.file_name, start_addr=self.start_addr)
        frame.Show()
        return True


