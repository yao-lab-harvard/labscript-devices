#####################################################################
#                                                                   #
# /SRS384.py                                                  #
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


class SRS384DDS(DDSQuantity):
    description = 'SRS384DDS'
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
        self.output = False
        self.enable_settings = False
        self.IQ_settings = False #[IQ on/off, FREQ Sweep on/off]
        self.freq_sweep_settings = False #[IQ on/off, FREQ Sweep on/off]
        self.freq_sweep_rate = 0
        self.freq_sweep_dev = 0

    def hold_phase(self, t):
        self.phase_reset.go_high(t)

    def release_phase(self, t):
        self.phase_reset.go_low(t)

    def enable_output(self, t):
        self.output = True

    def enable_mod(self, t, enable_mod_status):
        self.enable_settings = enable_mod_status

    def enable_IQ(self, t):
        self.IQ_settings = True

    def enable_freq_sweep(self,t):
        self.freq_sweep_settings = True

    def set_sweep_rate(self,t, freq_rate):
        self.freq_sweep_rate = freq_rate

    def set_sweep_dev(self,t, freq_dev):
        self.freq_sweep_dev = freq_dev

profiles = {}
def profile(funct):
    func = funct.__name__
    if func not in profiles:
        profiles[func] = {'total_time':0, 'min':None, 'max':0, 'num_calls':0, 'average_time_per_call':0}
    
    def new_func(*args,**kwargs):
        start_time = time.time()
        ret = funct(*args,**kwargs)
        runtime = time.time()-start_time
        profiles[func]['total_time'] += runtime
        profiles[func]['num_calls'] += 1
        profiles[func]['min'] = profiles[func]['min'] if profiles[func]['min'] is not None and profiles[func]['min'] < runtime else runtime
        profiles[func]['max'] = profiles[func]['max'] if profiles[func]['max'] > runtime else runtime
        profiles[func]['average_time_per_call'] = profiles[func]['total_time']/profiles[func]['num_calls']
        
        return ret
    # return new_func
    return funct
    
def start_profile(name):
    if name not in profiles:
        profiles[name] = {'total_time':0, 'min':None, 'max':0, 'num_calls':0, 'average_time_per_call':0}
        
    if 'start_time' in profiles[name]:
        raise Exception('You cannot call start_profile for %s without first calling stop_profile'%name)
        
    profiles[name]['start_time'] = time.time()
    
def stop_profile(name):
    if name not in profiles or 'start_time' not in profiles[name]:
        raise Exception('You must first call start_profile for %s before calling stop_profile')
        
    runtime = time.time()-profiles[name]['start_time']
    del profiles[name]['start_time']
    profiles[name]['total_time'] += runtime
    profiles[name]['num_calls'] += 1
    profiles[name]['min'] = profiles[name]['min'] if profiles[name]['min'] is not None and profiles[name]['min'] < runtime else runtime
    profiles[name]['max'] = profiles[name]['max'] if profiles[name]['max'] > runtime else runtime
    profiles[name]['average_time_per_call'] = profiles[name]['total_time']/profiles[name]['num_calls']


class SRS384(IntermediateDevice):
    # This device can only have Pseudoclock children (digital outs and DDS outputs should be connected to a child device)
    allowed_children = [DigitalOut, DDS, SRS384DDS]
    
    @set_passed_properties(
        property_names = {"connection_table_properties": ["com_port", ]}
        )
    def __init__(self, name, parent_device=None, clock_terminal=None, com_port = "COM4", **kwargs):
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
            msg = """If using waits without a wait monitor, the SRS384 used as a
                master pseudoclock must have
                programming_scheme='pb_stop_programming/STOP'. Otherwise there is no way
                for BLACS to distinguish between a wait, and the end of a shot. Either
                use a wait monitor (see labscript.WaitMonitor for details) or set
                programming_scheme='pb_stop_programming/STOP for %s."""
            raise LabscriptError(dedent(msg) % self.name)


    def _make_SRS_settings_table(self, inputs):
        """Collect analog input instructions and create the acquisition table"""
        if not inputs:
            return None

        for connection, input in inputs.items():
            amp = input.__dict__['amplitude'].__dict__['raw_output'][0]
            freq = input.__dict__['frequency'].__dict__['raw_output'][0]
            output = input.__dict__['output']
            enable_settings = input.__dict__['enable_settings']
            IQ_settings = input.__dict__['IQ_settings']
            freq_sweep_settings = input.__dict__['freq_sweep_settings']
            freq_sweep_rate = input.__dict__['freq_sweep_rate']
            freq_sweep_dev = input.__dict__['freq_sweep_dev']

        settings = [(amp, freq, output, enable_settings, IQ_settings, freq_sweep_settings, freq_sweep_dev, freq_sweep_rate)]
        settings_dtypes = [
            ('amp', float),
            ('freq', float),
            ('output', bool),
            ('enable_settings', bool),
            ('IQ_settings', bool),
            ('freq_sweep_settings', bool),
            ('freq_sweep_dev', float),
            ('freq_sweep_rate', float)
        ]
        settings_table = np.empty(len(settings), dtype=settings_dtypes)
        for i, acq in enumerate(settings):
            settings_table[i] = acq
        return settings
    def generate_code(self, hdf5_file):
        # Generate the hardware instructions
        IntermediateDevice.generate_code(self, hdf5_file)
        DDS_set = {}
        for device in self.child_devices:
            if isinstance(device, (DDS, SRS384DDS)):
                DDS_set[device.connection] = device
        DDStable = self._make_SRS_settings_table(DDS_set)

        grp = self.init_device_group(hdf5_file)
        if DDStable is not None:
            grp.create_dataset('DDS', data=DDStable, compression=config.compression)

class SRS384DirectOutputs(IntermediateDevice):
    allowed_children = [DDS, SRS384DDS, DigitalOut]
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
class SRS384Tab(DeviceTab):
    def initialise_GUI(self):
        # Capabilities
                # Create status labels

        self.base_units =    {'freq':'MHz',         'amp':'dBm',   'phase':'Degrees', 'rate':'Hz', 'deviation': "MHz"}
        self.base_min =      {'freq':0,           'amp':-50,  'phase':0, 'rate':0, 'deviation': 0}
        self.base_max =      {'freq':4000.,         'amp':0,    'phase':360, 'rate':10**9, 'deviation': 100}
        self.base_step =     {'freq':1.0,           'amp':0.1,     'phase':1, 'rate':1, 'deviation': 0.001}
        self.base_decimals = {'freq':4,             'amp':4,       'phase':3,  'rate':4, 'deviation':4} # TODO: find out what the phase precision is!
        self.num_DO = 8

        # Create DDS Output objects
        RF_prop = {}
        
        RF_prop['Output Channel'] = {}
        for subchnl in ['freq', 'amp', 'phase']:
            RF_prop['Output Channel'][subchnl] = {'base_unit':self.base_units[subchnl],
                                                'min':self.base_min[subchnl],
                                                'max':self.base_max[subchnl],
                                                'step':self.base_step[subchnl],
                                                'decimals':self.base_decimals[subchnl]
                                                }
        RF_prop["Modulation Settings"] = {}
        for subchnl in ['rate', 'deviation']:
            RF_prop['Modulation Settings'][subchnl] = {'base_unit':self.base_units[subchnl],
                                                'min':self.base_min[subchnl],
                                                'max':self.base_max[subchnl],
                                                'step':self.base_step[subchnl],
                                                'decimals':self.base_decimals[subchnl]
                                                }        



        do_prop = {}
        do_prop['Output'] = {}
        do_prop['IQ'] = {}
        do_prop["Frequency Sweep"] = {}

        # Create the output objects    
        self.create_digital_outputs(do_prop)        
        # Create widgets for output object
        
        # Create the output objects
        self.create_dds_outputs(RF_prop)

        # Create widgets for output objects
        dds_widgets,ao_widgets,do_widgets = self.auto_create_widgets()
        # and auto place the widgets in the UI
        self.auto_place_widgets(("RF Output",dds_widgets) ,("Modulation Settings",do_widgets))


        # Store the COM port to be used
        self.com_port = str(self.settings['connection_table'].find_by_name(self.device_name).BLACS_connection)

        # Create and set the primary worker
        self.create_worker("main_worker", SRS384Worker, {'com_port':self.com_port})
        self.primary_worker = "main_worker"

        # Create status labels
        self.status_label = QtWidgets.QLabel("Status: Unknown")
        self.clock_status_label = QtWidgets.QLabel("Clock status: Unknown")
        self.get_tab_layout().addWidget(self.status_label)
        self.get_tab_layout().addWidget(self.clock_status_label)

        # Set the capabilities of this device
        self.supports_smart_programming(True)

        # Create status monitor timout
        self.statemachine_timeout_add(2000, self.status_monitor)
        # Set the capabilities of this device
        self.supports_remote_value_check(False) # !!!
        self.supports_smart_programming(False) # !!!
    @define_state(
        MODE_MANUAL
        | MODE_BUFFERED
        | MODE_TRANSITION_TO_BUFFERED
        | MODE_TRANSITION_TO_MANUAL,
        True,
    )
    def status_monitor(self, notify_queue=None):
        """Gets the status of the PrawnBlaster from the worker.

        When called with a queue, this function writes to the queue
        when the PrawnBlaster is waiting. This indicates the end of
        an experimental run.

        Args:
            notify_queue (:class:`~queue.Queue`): Queue to notify when
                the experiment is done.

        """

        status, clock_status, waits_pending = yield (
            self.queue_work(self.primary_worker, "check_status")
        )

        # Manual mode or aborted
        done_condition = status == 0 or status == 5
        done_condition = True
        # Update GUI status/clock status widgets
        self.status_label.setText(f"Status: {status}")
        self.clock_status_label.setText(f"Clock status: {clock_status}")

        if notify_queue is not None and done_condition and not waits_pending:
            # Experiment is over. Tell the queue manager about it, then
            # set the status checking timeout back to every 2 seconds
            # with no queue.
            notify_queue.put("done")
            self.statemachine_timeout_remove(self.status_monitor)
            self.statemachine_timeout_add(2000, self.status_monitor)


import re
class SRS384Worker(Worker):

    def init(self):

        global h5py; import labscript_utils.h5_lock, h5py

        self.COMPort = self.com_port
        self.baudrate = 115200
        self.final_values = {}

        self.smart_cache = {'RF_DATA': None,
                            'SWEEP_DATA': None}


    def check_status(self):
        return 2, 0, False

    def check_remote_values(self):
        results = {}
        self.logger.info("CHECK REMOTE VALUES")
        self.logger.info("flag" + str(self.flag))
        for i in range(1):
            results['channel '+str(i)]=  {}
        self.final_values = {}

        SRS384ser = serial.Serial(self.COMPort, baudrate=self.baudrate, timeout=1)
        if(SRS384ser.isOpen() == False):
            SRS384ser.open()
        
        SRS384ser.write(b"FREQ? MHz \r\n")
        freq = SRS384ser.read(SRS384ser.inWaiting()).decode()
        while freq == '':
            SRS384ser.write(b"FREQ? MHz \r\n")
            freq = SRS384ser.read(SRS384ser.inWaiting()).decode()
        freq = re.findall("\d+\.\d+", freq)[0]
        results['channel '+str(0)]['freq'] = float(freq)
        self.logger.info(freq)
        SRS384ser.read(SRS384ser.inWaiting()).decode()
        SRS384ser.write(b"AMPL? \r\n")
        amp = SRS384ser.read(SRS384ser.inWaiting()).decode()

        while amp == '':
            SRS384ser.write(b"AMPL? \r\n")
            amp = SRS384ser.read(SRS384ser.inWaiting()).decode()
            self.logger.info("ASASA" + amp)
        self.logger.info("ASDQSAAS" + str(re.findall("\d+\.\d+", amp)))
        amp = re.findall("\d+\.\d+", amp)[0]
        results['channel '+str(0)]['amp'] = float(amp)
        SRS384ser.read(SRS384ser.inWaiting()).decode()

        results['channel '+str(0)]['phase'] = 0
        SRS384ser.close()

        return results


    def program_manual(self,front_panel_values):
        self.logger.info("PROGRAM MANUAL")

        SRS384ser = serial.Serial(self.COMPort, baudrate=self.baudrate, timeout=1)
        if(SRS384ser.isOpen() == False):
            SRS384ser.open()
        enable_LF = "ENBL " + str(0) + " \r\n" 
        SRS384ser.write(enable_LF.encode())
        enable_BR = "ENBR " + str(0) + " \r\n" 
        SRS384ser.write(enable_BR.encode())

        values = front_panel_values['Output Channel']
        power_dBm = values['amp']
        amp_string = "AMPL " + str(power_dBm) + " \r\n" 
        SRS384ser.write(amp_string.encode())
        amp_string = "AMPR " + str(power_dBm) + " \r\n" 
        SRS384ser.write(amp_string.encode())

        freq = values['freq']
        freq_string = "FREQ " + str(freq) + " MHZ \r\n" 
        SRS384ser.write(freq_string.encode())


        # Now that a manual update has been done, we'd better invalidate the saved RF_DATA:
        self.smart_cache['RF_DATA'] = None

        #return self.check_remote_values()
        results = {}

        results['Output Channel']=  {}
        results['Output Channel']['freq'] = freq
        results['Output Channel']['amp'] = power_dBm
        results['Output Channel']['phase'] = 0

        mod_values = front_panel_values['Modulation Settings']
        #print(mod_values)
        mod_rate = mod_values['rate']
        mod_rate_string = "SRAT " + str(mod_rate) + " \r\n"
        mod_deviation = mod_values['deviation']
        mod_deviation_string = "SDEV " + str(mod_deviation) + " MHZ \r\n" 
        mod_function_string = "SFNC " + str(1) + " MHZ \r\n" 

        if front_panel_values["Output"]:
            enable_LF = "ENBL " + str(1) + " \r\n" 
            SRS384ser.write(enable_LF.encode())
            enable_BR = "ENBR " + str(1) + " \r\n" 
            SRS384ser.write(enable_BR.encode())
        else:
            enable_LF = "ENBL " + str(0) + " \r\n" 
            SRS384ser.write(enable_LF.encode())
            enable_BR = "ENBR " + str(0) + " \r\n" 
            SRS384ser.write(enable_BR.encode())

        if front_panel_values["IQ"] and not front_panel_values["Frequency Sweep"]:
            enable_mod_string = "MODL " + str(1) + " \r\n" 
            SRS384ser.write(enable_mod_string.encode())
            mod_string = "TYPE " + '6' + " \r\n" 
            SRS384ser.write(mod_string.encode())

            IQ_setting_string = "QFNC " + str(5) + " \r\n"
            SRS384ser.write(IQ_setting_string.encode())

        elif front_panel_values["Frequency Sweep"] and not front_panel_values["IQ"]:
            enable_mod_string = "MODL " + str(1) + " \r\n" 
            SRS384ser.write(enable_mod_string.encode())
            mod_string = "TYPE " + '3' + " \r\n" 
            SRS384ser.write(mod_string.encode())

            SRS384ser.write(mod_function_string.encode())
            SRS384ser.write(mod_rate_string.encode())
            SRS384ser.write(mod_deviation_string.encode())

        else:
            disable_mod_string = "MODL " + str(0) + " \r\n" 
            SRS384ser.write(disable_mod_string.encode())

        results['Modulation Settings'] = {}
        results['Modulation Settings']['rate'] = mod_rate
        results['Modulation Settings']['deviation'] = mod_deviation

        SRS384ser.close()

        return results

    def start_run(self):

        self.started = True

    def transition_to_buffered(self,device_name,h5file,initial_values,fresh):
        self.h5file = h5file
        self.started = False
        return_values = {'a': 1}
        SRS384ser = serial.Serial(self.COMPort, baudrate=self.baudrate, timeout=1)

        if(SRS384ser.isOpen() == False):
            SRS384ser.open()
        with h5py.File(h5file,'r') as hdf5_file:
            group = hdf5_file['devices/%s'%device_name]
            DDS_table = group['DDS'][:]
            self.logger.info(DDS_table)
            power_dBm = DDS_table[0][0]
            amp_string = "AMPL " + str(power_dBm) + " \r\n" 
            SRS384ser.write(amp_string.encode())
            amp_string = "AMPR " + str(power_dBm) + " \r\n" 
            SRS384ser.write(amp_string.encode())

            freq = DDS_table[0][1]
            freq_string = "FREQ " + str(freq) + " MHZ \r\n" 
            SRS384ser.write(freq_string.encode())

            if DDS_table[0][3]:
                enable_LF = "ENBL " + str(1) + " \r\n" 
                SRS384ser.write(enable_LF.encode())
                enable_BR = "ENBR " + str(1) + " \r\n" 
                SRS384ser.write(enable_BR.encode())
                self.logger.info("SRS turned on")
            else:
                enable_LF = "ENBL " + str(0) + " \r\n" 
                SRS384ser.write(enable_LF.encode())
                enable_BR = "ENBR " + str(0) + " \r\n" 
                SRS384ser.write(enable_BR.encode())
                self.logger.info("SRS turned off")

            if DDS_table[0][4]:
                enable_mod_string = "MODL " + str(1) + " \r\n" 
                SRS384ser.write(enable_mod_string.encode())
                mod_string = "TYPE " + '6' + " \r\n" 
                SRS384ser.write(mod_string.encode())

                IQ_setting_string = "QFNC " + str(5) + " \r\n"
                SRS384ser.write(IQ_setting_string.encode())
                coup_string = "COUP " + '1' + " \r\n" 
                SRS384ser.write(coup_string.encode())
            elif DDS_table[0][5]:
                enable_mod_string = "MODL " + str(1) + " \r\n" 
                SRS384ser.write(enable_mod_string.encode())
                mod_string = "TYPE " + '3' + " \r\n" 
                SRS384ser.write(mod_string.encode())
                coup_string = "COUP " + '1' + " \r\n" 
                SRS384ser.write(coup_string.encode())

                freq_sweep_string = "SFNC " + '5' + " \r\n"
                SRS384ser.write(freq_sweep_string.encode())

                SDEV_string = "SDEV " + str(DDS_table[0][6]) + " \r\n"
                print(SDEV_string)
                SRS384ser.write(SDEV_string.encode())

                SRAT_string = "SRAT " + str(DDS_table[0][7]) + " \r\n"
                SRS384ser.write(SRAT_string.encode())
            #raise LabscriptError(DDS_table)
        SRS384ser.close()

        return return_values

    def abort_transition_to_buffered(self):
        return self.transition_to_manual(True)

    def abort_buffered(self):
        return self.transition_to_manual(True)

    def transition_to_manual(self,abort = False):


        return True

    def shutdown(self):
        if(SRS384ser.isOpen() == False):
            SRS384ser.open()
        enable_LF = "ENBL " + str(0) + " \r\n" 
        SRS384ser.write(enable_LF.encode())
        enable_BR = "ENBR " + str(0) + " \r\n" 
        SRS384ser.write(enable_BR.encode())
        SRS384ser.close()

        return




import labscript_devices

labscript_device_name = 'SRS384'
blacs_tab = 'labscript_devices.SRS384.SRS384Tab'

labscript_devices.register_classes(
    labscript_device_name=labscript_device_name,
    BLACS_tab=blacs_tab
)