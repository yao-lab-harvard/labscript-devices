#####################################################################
#                                                                   #
# /ZCU111.py                                                  #
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
                                     'The digital gate is always internal to the ZCU4.')
import time

from blacs.tab_base_classes import Worker, define_state
from blacs.tab_base_classes import MODE_MANUAL, MODE_TRANSITION_TO_BUFFERED, MODE_TRANSITION_TO_MANUAL, MODE_BUFFERED

from blacs.device_base_class import DeviceTab
from qtutils.qt import QtWidgets

@BLACS_tab
class ZCU111Tab(DeviceTab):
    def initialise_GUI(self):
        # Capabilities
                # Create status labels
        self.base_units =    {'freq':'MHz',         'amp':'dBm',   'phase':'Degrees', 'length': "ns"}
        self.base_min =      {'freq':0,           'amp':-136.0,  'phase':0, 'length': 0}
        self.base_max =      {'freq':4000.,         'amp':25.0,    'phase':360, 'length':10000}
        self.base_step =     {'freq':1.0,           'amp':1.0,     'phase':1, 'length':1}
        self.base_decimals = {'freq':4,             'amp':4,       'phase':3, 'length':3} # TODO: find out what the phase precision is!
        self.num_DO = 8

        # Create DDS Output objects
        RF_prop = {}
        for i in range(7):
            RF_prop['channel '+str(i)] = {}
            for subchnl in ['freq', 'amp', 'phase', 'length']:
                RF_prop['channel '+str(i)][subchnl] = {'base_unit':self.base_units[subchnl],
                                                    'min':self.base_min[subchnl],
                                                    'max':self.base_max[subchnl],
                                                    'step':self.base_step[subchnl],
                                                    'decimals':self.base_decimals[subchnl]
                                                    }
        do_prop = {}
        for i in range(self.num_DO): 
            do_prop['flag %d'%i] = {}
        
        # Create the output objects    
        self.create_digital_outputs(do_prop)        
        # Create widgets for output objects
        
        # Define the sort function for the digital outputs
        def sort(channel):
            flag = channel.replace('flag ','')
            flag = int(flag)
            return '%02d'%(flag)

        # Create the output objects
        self.create_dds_outputs(RF_prop)

        # Create widgets for output objects
        dds_widgets,ao_widgets,do_widgets = self.auto_create_widgets()
        # and auto place the widgets in the UI
        self.auto_place_widgets(("RF Output",dds_widgets) ,("Flags",do_widgets,sort))


        # Store the COM port to be used
        self.com_port = str(self.settings['connection_table'].find_by_name(self.device_name).BLACS_connection)

        # Create and set the primary worker
        self.create_worker("main_worker", ZCU111Worker, {'com_port':self.com_port})
        self.primary_worker = "main_worker"

        # Create status labels
        self.status_label = QtWidgets.QLabel("Status: Unknown")
        self.clock_status_label = QtWidgets.QLabel("Clock status: Unknown")
        self.get_tab_layout().addWidget(self.status_label)
        self.get_tab_layout().addWidget(self.clock_status_label)


import re
class ZCU111Worker(Worker):

    def init(self):

        global h5py; import labscript_utils.h5_lock, h5py

        self.COMPort = self.com_port
        self.baudrate = 115200
        self.reps = 1
        self.delay_time_repetitions = 10 #us
        self.start_src = "internal" 
        self.sequence_list = []
        self.pulse_list = []
        self.final_values = {}
        self.first_run = True

        self.smart_cache = {'RF_DATA': None,
                            'SWEEP_DATA': None}
        '''with open('C:/Users/Yao Lab/labscript-suite/plotter/start_pulse.py') as f:
            exec(f.read())'''
        self.logger.info("INITIALIZING")
        self.ZCU4ser = serial.Serial(self.COMPort, baudrate=self.baudrate, timeout=1)

        if(self.ZCU4ser.isOpen() == False):
            self.ZCU4ser.open()

        self.ZCU4ser.write(b"cd jupyter_notebooks\r\n")

        self.ZCU4ser.write(b"cd qick\r\n")
        
        self.ZCU4ser.write(b"cd qick_demos\r\n")

        self.ZCU4ser.write(b"sudo python3\r\n")
        time.sleep(2)

        self.ZCU4ser.write(b"xilinx\r\n")
        time.sleep(2)
        self.ZCU4ser.write(b"exec(open('initialize.py').read())\r\n")
        time.sleep(15)
        self.logger.info(self.ZCU4ser.read(self.ZCU4ser.inWaiting()).decode())
        #self.ZCU4ser.write(b"exec(open('test_pulse.py').read())\r\n")
        self.ZCU4ser.close()

    def check_remote_values(self):
        results = {}
        for i in range(7):
            results['channel '+str(i)]=  {}
        self.final_values = {}


        for i in range(7):
            results['channel '+str(i)]['freq'] = 0
            results['channel '+str(i)]['amp'] = 0
            results['channel '+str(i)]['phase'] = 0
            results['channel '+str(i)]['length'] = 0


        return results

    def program_manual(self,front_panel_values):
        self.logger.info("In MANUAL")

        '''ZCU4ser = serial.Serial(self.COMPort, baudrate=self.baudrate, timeout=1)
        if(ZCU4ser.isOpen() == False):
            ZCU4ser.open()

        for i in range(7):
            values = front_panel_values['channel ' + str(i)]

        sequence_list = []
        for i in range(8):
            if front_panel_values['flag %d'%i]:
                sequence_list.append((i, 0, 1))
                #raise LabscriptError(str(sequence_list) + " attempt to use digital output")

        pulse_list = [[6, 'const', 0, 100, 30000, 100, 0, 'oneshot', 'product', '[]']]
        #pulse_list = []
        pulse_list_string = "pulse_list = " + str(pulse_list) + "\r\n"
        ZCU4ser.write(pulse_list_string.encode())
        time.sleep(1)
        sequence_list_string = "sequence_list = " + str(sequence_list) + "\r\n"
        ZCU4ser.write(sequence_list_string.encode())
        time.sleep(1)
        loop_number_string = "loop_number = " + str(1) + "\r\n"
        ZCU4ser.write(loop_number_string.encode())
        time.sleep(1)
        delay_time_string = "delay_time = " + str(self.delay_time_repetitions) + "\r\n"
        ZCU4ser.write(delay_time_string.encode())
        time.sleep(1)
        start_src_string = "start = " + str(self.start_src) + "\r\n"
        ZCU4ser.write(start_src_string.encode())
        time.sleep(1)
        ZCU4ser.write(b"exec(open('send_pulse.py').read())\r\n")

        ZCU4ser.close()'''

        # Now that a manual update has been done, we'd better invalidate the saved RF_DATA:
        self.smart_cache['RF_DATA'] = None

        return self.check_remote_values()

    def start_run(self):
        #raise LabscriptError(str(sequence_list_string))
        self.started = True

    def transition_to_buffered(self,device_name,h5file,initial_values,fresh):
        self.logger.info("IN BUFFERED")
        self.h5file = h5file
        self.started = False
        self.device_name = device_name
        self.sequence_list = []
        self.pulse_list = []
        return_values = {'a': 1}

        with h5py.File(h5file,'r') as hdf5_file:
            group = hdf5_file['devices/%s'%device_name]
            Settings = group['Settings']
            DDS = group['DDS']
            TTL = group['TTL']
            group = hdf5_file['devices/%s'%device_name]
            DDS_table = group['DDS'][:]
            self.reps = int(Settings[0][0].decode())
            self.delay_time_repetitions = float(Settings[0][1].decode())
            for i in range(len(DDS)):
                self.pulse_list.append([int(DDS[i][0].decode()), DDS[i][1].decode(), int(float(DDS[i][2].decode())*(10**9)), float(DDS[i][3].decode()),int(DDS[i][4].decode()),int(DDS[i][5].decode()),int(DDS[i][6].decode()),DDS[i][7].decode(),DDS[i][8].decode(),DDS[i][9].decode()    ]   )
            self.logger.info(self.pulse_list)
            for i in range(len(TTL)):
                self.sequence_list.append( (int(TTL[i][0]), int(TTL[i][1]), int(TTL[i][2] )))  
            self.logger.info(self.sequence_list)

        if(self.ZCU4ser.isOpen() == False):
            self.ZCU4ser.open()

        
        pulse_list_string = "pulse_list = " + str(self.pulse_list) + "\r\n"
        self.ZCU4ser.write(pulse_list_string.encode())
        sequence_list_string = "sequence_list = " + str(self.sequence_list) + "\r\n"
        self.ZCU4ser.write(sequence_list_string.encode())
        loop_number_string = "number_of_loops = " + str(self.reps) + "\r\n"
        self.ZCU4ser.write(loop_number_string.encode())
        delay_time_string = "delay_time = " + str(self.delay_time_repetitions) + "\r\n"
        self.ZCU4ser.write(delay_time_string.encode())
        start_src_string = "start = " + self.start_src + "\r\n"
        self.ZCU4ser.write(start_src_string.encode())
        self.ZCU4ser.write(b"exec(open('send_pulse.py').read())\r\n")
        #self.logger.info(self.ZCU4ser.read(self.ZCU4ser.inWaiting()).decode())
        self.ZCU4ser.close()
        return return_values


    def abort_transition_to_buffered(self):
        return self.transition_to_manual(True)

    def abort_buffered(self):
        return self.transition_to_manual(True)

    def transition_to_manual(self,abort = False):
        self.logger.info("TRANSITION TO MANUAL")

        '''self.sequence_list = []
        self.pulse_list = []
        with h5py.File(self.h5file,'r') as hdf5_file:
            group = hdf5_file['devices/%s'%self.device_name]
            Settings = group['Settings']
            DDS = group['DDS']
            TTL = group['TTL']
            group = hdf5_file['devices/%s'%self.device_name]
            DDS_table = group['DDS'][:]
            self.reps = int(Settings[0][0].decode())
            self.delay_time_repetitions = float(Settings[0][1].decode())
            #self.logger.info(self.reps)
            #self.logger.info(self.delay_time_repetitions)
            #self.logger.info(self.start_src)

            for i in range(len(DDS)):
                self.pulse_list.append([int(DDS[i][0].decode()), DDS[i][1].decode(), int(float(DDS[i][2].decode())*(10**9)), float(DDS[i][3].decode()),int(DDS[i][4].decode()),int(DDS[i][5].decode()),int(DDS[i][6].decode()),DDS[i][7].decode(),DDS[i][8].decode(),DDS[i][9].decode()    ]   )
            self.logger.info(self.pulse_list)
            for i in range(len(TTL)):
                self.sequence_list.append( (int(TTL[i][0]), int(TTL[i][1]), int(TTL[i][2] )))  
            self.logger.info(self.sequence_list)

        if(self.ZCU4ser.isOpen() == False):
            self.ZCU4ser.open()

        pulse_list_string = "pulse_list = " + str(self.pulse_list) + "\r\n"
        self.ZCU4ser.write(pulse_list_string.encode())
        sequence_list_string = "sequence_list = " + str(self.sequence_list) + "\r\n"
        self.ZCU4ser.write(sequence_list_string.encode())
        loop_number_string = "number_of_loops = " + str(self.reps) + "\r\n"
        self.ZCU4ser.write(loop_number_string.encode())
        delay_time_string = "delay_time = " + str(self.delay_time_repetitions) + "\r\n"
        self.ZCU4ser.write(delay_time_string.encode())
        self.ZCU4ser.write(b"exec(open('send_pulse.py').read())\r\n")
        self.ZCU4ser.close()'''

        return True

    def shutdown(self):
        self.ZCU4ser.close()
        return




import labscript_devices

labscript_device_name = 'ZCU111'
blacs_tab = 'labscript_devices.ZCU111.ZCU111Tab'

labscript_devices.register_classes(
    labscript_device_name=labscript_device_name,
    BLACS_tab=blacs_tab
)