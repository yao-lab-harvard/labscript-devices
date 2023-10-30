from msilib import sequence
from labscript_devices import runviewer_parser, labscript_device, BLACS_tab, BLACS_worker

from labscript import Device, IntermediateDevice, Pseudoclock, ClockLine, PseudoclockDevice, config, LabscriptError, StaticAnalogQuantity, AnalogOut, DigitalOut, set_passed_properties, WaitMonitor, compiler, DDS, DDSQuantity, DigitalQuantity

import numpy as np
import h5py
from ctypes import *



class ZCU111DDS(DDSQuantity):
    description = 'ZCU111DDS'
    def __init__(self, *args, **kwargs):
        if 'call_parents_add_device' in kwargs:
            call_parents_add_device = kwargs['call_parents_add_device']
        else:
            call_parents_add_device = True

        kwargs['call_parents_add_device'] = False
        DDSQuantity.__init__(self, *args, **kwargs)

        self.reps = '1'
        self.delay_time_repetitions = '10' #us
        self.start_src = "internal" 

        self.pulse_sequence_list = [] # [[6, 'const', 0, 100, 30000, 100, 0, 'oneshot', 'product', '[]']] #const = 0, arb = 1. oneshot = 0, periodic = 1. product  = 0, table = 1
        self.sequence_list = []


        if call_parents_add_device:
            self.parent_device.add_device(self)

    def set_repetitions(self,t, reps):
        self.reps = reps

    def set_delay_time(self,t, delay_time):
        self.delay_time_repetitions = delay_time

    def set_start_src(self,t, start):
        self.start_src = start

    def add_pulse(self, channel, style, start_time, length, gain, frequency, phase = 0, mode = 'oneshot', outsel = 'product', function_type = '[]'):
        self.pulse_sequence_list.append([channel, style, start_time, length, gain, frequency, phase, mode , outsel, function_type])

    def add_TTL(self, channel, start_time, end_time):
        self.sequence_list.append((channel, int((start_time)*(10**9)), int((end_time)*(10**9))))


class ZCU111(IntermediateDevice):
    # This device can only have Pseudoclock children (digital outs and DDS outputs should be connected to a child device)
    allowed_children = [DigitalOut, DDS, ZCU111DDS]
    
    @set_passed_properties(
        property_names = {"connection_table_properties": ["com_port", ]}
        )
    def __init__(self, name, parent_device=None, clock_terminal=None, com_port = "COM5", **kwargs):
        self.BLACS_connection = com_port
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
            msg = """If using waits without a wait monitor, the ZCU111 used as a
                master pseudoclock must have
                programming_scheme='pb_stop_programming/STOP'. Otherwise there is no way
                for BLACS to distinguish between a wait, and the end of a shot. Either
                use a wait monitor (see labscript.WaitMonitor for details) or set
                programming_scheme='pb_stop_programming/STOP for %s."""
            raise LabscriptError(dedent(msg) % self.name)


    def _make_ZCU111_settings_table(self, inputs):
        """Collect analog input instructions and create the acquisition table"""
        if not inputs:
            return None

        for connection, input in inputs.items():
            reps = input.__dict__['reps']
            delay_time = input.__dict__['delay_time_repetitions']
            start_src = input.__dict__['start_src']
            pulse_sequence_list = input.__dict__['pulse_sequence_list']
            sequence_list = input.__dict__['sequence_list']

        settings = [(reps, delay_time, start_src)]
        settings_dtypes = [
            ('reps', str),
            ('delay_time', str),
            ('start_src', str)
        ]
        settings_table = np.empty(len(settings), dtype=settings_dtypes)
        for i, acq in enumerate(settings):
            settings_table[i] = acq

        DDS_dtype = [('channel', float),('style', str),('start_time', float), 
            ('length', float), ('gain', float),
            ('frequency', float), ('phase', float), ('mode', str), ('outsel', str), ('function_type', str)]
        #print(pulse_sequence_list)
        DDS_table = np.empty(len(pulse_sequence_list),dtype = DDS_dtype)
        for i,j in enumerate(pulse_sequence_list):
            for k in range(len(j)):
                j[k] = str(j[k])
            DDS_table[i] = (j[0], j[1],j[2],j[3], j[4], j[5], j[6], j[7], j[8], j[9])
        #print(DDS_table)
        #print(pulse_sequence_list)
        return settings, pulse_sequence_list, sequence_list
    
    def generate_code(self, hdf5_file):
        # Generate the hardware instructions
        IntermediateDevice.generate_code(self, hdf5_file)
        DDS_set = {}
        for device in self.child_devices:
            if isinstance(device, (DDS, ZCU111DDS)):
                DDS_set[device.connection] = device
        SettingsTable, PulseTable, SequenceTable = self._make_ZCU111_settings_table(DDS_set)
        #print(PulseTable)

        grp = self.init_device_group(hdf5_file)
        dt = h5py.string_dtype(encoding='utf-8') 
        grp.create_dataset('Settings', data=SettingsTable, compression=config.compression, dtype = dt)
        grp.create_dataset('DDS', compression=config.compression,data = PulseTable, dtype=dt)
        grp.create_dataset('TTL', compression=config.compression,data = SequenceTable)

class ZCU111DirectOutputs(IntermediateDevice):
    allowed_children = [DDS, ZCU111DDS, DigitalOut]
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
                                     'The digital gate is always internal to the ZCU4.')   ###TODO: Should this ZCU4 be changed?