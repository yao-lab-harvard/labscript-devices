#####################################################################
#                                                                   #
# /PiezoEO.py                                                  #
#                                                                   #
# Copyright 2013, Monash University                                 #
#                                                                   #
# This file is part of the module labscript_devices, in the         #
# labscript suite (see http://labscriptsuite.org), and is           #
# licensed under the Simplified BSD License. See the license.txt    #
# file in the root of the project for the full license.             #
#                                                                   #
#####################################################################
from msilib import sequence
from labscript_devices import runviewer_parser, labscript_device, BLACS_tab, BLACS_worker

from labscript import Device, IntermediateDevice, Pseudoclock, ClockLine, PseudoclockDevice, config, LabscriptError, StaticAnalogQuantity, AnalogOut, DigitalOut, set_passed_properties, WaitMonitor, compiler, DDS, DDSQuantity, DigitalQuantity
import copy

import numpy as np
import labscript_utils.h5_lock, h5py
from ctypes import *
import struct
import serial
import time
max_deviations = [
    {'low': 0.1,        'high': 249.999,    'dev': 10.0},
    {'low': 249.999,    'high': 500.0,      'dev': 5.0},
    {'low': 500.0,      'high': 1000.0,     'dev': 10.0},
    {'low': 1000.0,     'high': 2000.0,     'dev': 20.0},
    {'low': 2000.0,     'high': 4000.0,     'dev': 40.0}
]


class PiezoEODDS(DDSQuantity):
    description = 'PiezoEODDS'
    def __init__(self, *args, **kwargs):
        if 'call_parents_add_device' in kwargs:
            call_parents_add_device = kwargs['call_parents_add_device']
        else:
            call_parents_add_device = True

        kwargs['call_parents_add_device'] = False
        DDSQuantity.__init__(self, *args, **kwargs)

        self.gate = DigitalQuantity(self.name + '_gate', self, 'gate')
        self.phase_reset = DigitalQuantity(self.name + '_phase_reset', self, 'phase_reset')

        if call_parents_add_device:
            self.parent_device.add_device(self)


class PiezoEO(IntermediateDevice):
    # This device can only have Pseudoclock children (digital outs and DDS outputs should be connected to a child device)
    allowed_children = [DigitalOut, DDS, PiezoEODDS]

    def __init__(self, name, parent_device=None, clock_terminal=None,  **kwargs):
        self.BLACS_connection = None
        # Create the internal pseudoclock
        # Create the internal direct output clock_line
        IntermediateDevice.__init__(self, name, parent_device, **kwargs)

    def add_device(self, device):
        IntermediateDevice.add_device(self, device)


    def _check_wait_monitor_ok(self):
        if (
            compiler.master_pseudoclock is self
            and compiler.wait_table
            and compiler.wait_monitor is None
            and self.programming_scheme != 'pb_stop_programming/STOP'
        ):
            msg = """If using waits without a wait monitor, the PiezoEO used as a
                master pseudoclock must have
                programming_scheme='pb_stop_programming/STOP'. Otherwise there is no way
                for BLACS to distinguish between a wait, and the end of a shot. Either
                use a wait monitor (see labscript.WaitMonitor for details) or set
                programming_scheme='pb_stop_programming/STOP for %s."""
            raise LabscriptError(dedent(msg) % self.name)


    def _make_PiezoEO_settings_table(self, inputs):
        """Collect analog input instructions and create the acquisition table"""
        if not inputs:
            return None

        for connection, input in inputs.items():
            amp = input.__dict__['amplitude'].__dict__['raw_output'][0]


        settings = [(amp)]
        settings_dtypes = [('amp', float)]
        settings_table = np.empty(len(settings), dtype=settings_dtypes)
        for i, acq in enumerate(settings):
            settings_table[i] = acq
        return settings
    def generate_code(self, hdf5_file):
        # Generate the hardware instructions
        IntermediateDevice.generate_code(self, hdf5_file)
        DDS_set = {}
        for device in self.child_devices:
            if isinstance(device, (DDS, PiezoEODDS)):
                DDS_set[device.connection] = device
        DDStable = self._make_PiezoEO_settings_table(DDS_set)

        grp = self.init_device_group(hdf5_file)
        if DDStable is not None:
            grp.create_dataset('DDS', data=DDStable, compression=config.compression)

class PiezoEODirectOutputs(IntermediateDevice):
    allowed_children = [DDS, PiezoEODDS, DigitalOut]
    description = 'PB-DDSII-300 Direct Outputs'
  
    def add_device(self, device):
        IntermediateDevice.add_device(self, device)
        if isinstance(device, DDS):
            # Check that the user has not specified another digital line as the gate for this DDS, that doesn't make sense.
            # Then instantiate a DigitalQuantity to keep track of gating.
            if device.gate is None:
                device.gate = DigitalQuantity(device.name + '_gate', device, 'gate')
            else:
                raise LabscriptError('You cannot specify a digital gate ' +
                                     'for a DDS connected to %s. '% (self.name) + 
                                     'The digital gate is always internal to the ZCU4.')
import time

from blacs.tab_base_classes import Worker, define_state
from blacs.tab_base_classes import MODE_MANUAL, MODE_TRANSITION_TO_BUFFERED, MODE_TRANSITION_TO_MANUAL, MODE_BUFFERED

from blacs.device_base_class import DeviceTab
from qtutils.qt import QtWidgets

@BLACS_tab
class PiezoEOTab(DeviceTab):
    def initialise_GUI(self):
        # Capabilities
                # Create status labels

        self.base_units =    {'amp':'um'}
        self.base_min =      {'amp':0}
        self.base_max =      {'amp':100}
        self.base_step =     {'amp':1}
        self.base_decimals = {'amp':4} # TODO: find out what the phase precision is!

        # Create DDS Output objects
        RF_prop = {}
        
        RF_prop['Output Channel'] = {}
        for subchnl in ['amp']:
            RF_prop['Output Channel'][subchnl] = {'base_unit':self.base_units[subchnl],
                                                'min':self.base_min[subchnl],
                                                'max':self.base_max[subchnl],
                                                'step':self.base_step[subchnl],
                                                'decimals':self.base_decimals[subchnl]
                                                }
        
        # Create the output objects
        self.create_dds_outputs(RF_prop)

        # Create widgets for output objects
        dds_widgets,ao_widgets,do_widgets = self.auto_create_widgets()
        # and auto place the widgets in the UI
        self.auto_place_widgets(("RF Output",dds_widgets) )


        # Create and set the primary worker
        self.create_worker("main_worker", PiezoEOWorker)
        self.primary_worker = "main_worker"

        # Create status labels
        self.status_label = QtWidgets.QLabel("Status: Unknown")
        self.clock_status_label = QtWidgets.QLabel("Clock status: Unknown")
        self.get_tab_layout().addWidget(self.status_label)
        self.get_tab_layout().addWidget(self.clock_status_label)

import re
class PiezoEOWorker(Worker):

    def init(self):

        global h5py; import labscript_utils.h5_lock, h5py
        self.mydll = cdll.LoadLibrary("C:\Program Files\Edmund Optics\EO-Drive\EO-Drive.dll")
        functions_list = ['EO_GetHandleBySerial', 'EO_InitHandle', 'EO_InitAllHandles','EO_GetAllHandles','EO_NumberOfCurrentHandles','EO_ReleaseHandle','EO_ReleaseAllHandles','EO_Move','EO_GetMaxCommand','EO_GetCommandPosition','EO_GetSerialNumber']

        self.mydll.EO_ReleaseAllHandles()
        self.handle = self.mydll.EO_InitHandle()
        self.logger.info("current handle is " + str(self.handle))
        #self.mydll.EO_Move.argtype = (c_int, c_double)

    def program_manual(self,front_panel_values):
        results = {}

        results['Output Channel']=  {}

        values = front_panel_values['Output Channel']
        v_amp = values['amp']
        results['Output Channel']['amp'] = v_amp
        #raise LabscriptError(v_amp)
        success_value = self.mydll.EO_Move(self.handle, c_double(int(v_amp)))
        self.position = c_double()
        self.logger.info("attempted move:" + str(success_value))
        self.mydll.EO_GetCommandPosition(self.handle, byref(self.position))
        self.logger.info("current position is " +str(self.position.value))
        return results

    def transition_to_buffered(self,device_name,h5file,initial_values,fresh):
        self.h5file = h5file
        self.started = False
        return_values = {'a': 1}

        with h5py.File(h5file,'r') as hdf5_file:
            group = hdf5_file['devices/%s'%device_name]
            DDS_table = group['DDS'][:]
            v_amp = DDS_table[0]
        self.mydll.EO_Move(self.handle,c_double(int(v_amp)))
        self.position = c_double()
        self.mydll.EO_GetCommandPosition(self.handle, byref(self.position))
        self.logger.info("current position is " +str(self.position.value))
        return return_values

    def abort_transition_to_buffered(self):
        return self.transition_to_manual(True)

    def abort_buffered(self):
        return self.transition_to_manual(True)

    def transition_to_manual(self,abort = False):


        return True

    def shutdown(self):

        self.mydll.EO_ReleaseHandle(self.handle)
        return




import labscript_devices

labscript_device_name = 'PiezoEO'
blacs_tab = 'labscript_devices.PiezoEO.PiezoEOTab'
parser = 'labscript_devices.PiezoEO.PiezoEOParser'

labscript_devices.register_classes(
    labscript_device_name=labscript_device_name,
    BLACS_tab=blacs_tab,
    runviewer_parser=parser,
)