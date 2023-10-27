from labscript_devices import labscript_device, BLACS_tab, BLACS_worker
from labscript import IntermediateDevice, DigitalOut, AnalogOut, config, DDSQuantity, DDS, LabscriptError
import numpy as np
import h5py


class ChaseAWGDDS(DDSQuantity):
    description = 'ChaseAWGDDS'
    def __init__(self, *args, **kwargs):
        if 'call_parents_add_device' in kwargs:
            call_parents_add_device = kwargs['call_parents_add_device']
        else:
            call_parents_add_device = True

        kwargs['call_parents_add_device'] = False
        DDSQuantity.__init__(self, *args, **kwargs)

        if call_parents_add_device:
            self.parent_device.add_device(self)

        self.channel_list = [ ]
        self.NumPoints_list = [ ]
        self.NumLoops_list = [ ]
        self.Waveform_list = [ ]
    def add_pulse(self,channel, NumPoints, NumLoops, Waveform):
        self.channel_list.append(channel)
        self.NumPoints_list.append(NumPoints)
        self.NumLoops_list.append(NumLoops)
        self.Waveform_list.append(bytes(Waveform,'utf-8'))

class ChaseAWG(IntermediateDevice):

    description = 'Dummy IntermediateDevice'
    clock_limit = 1e6

    # If this is updated, then you need to update generate_code to support whatever types you add
    allowed_children = [DigitalOut, AnalogOut, DDS, ChaseAWGDDS]

    def __init__(self, name, parent_device, BLACS_connection='dummy_connection', **kwargs):
        self.BLACS_connection = BLACS_connection
        IntermediateDevice.__init__(self, name, parent_device, **kwargs)


    def _make_ChaseAWG_settings_table(self, inputs):
        """Collect analog input instructions and create the acquisition table"""
        if not inputs:
            return None

        for connection, input in inputs.items():
            DDS_channels = input.__dict__['channel_list']
            DDS_NumPoints = input.__dict__['NumPoints_list']
            DDS_NumLoops = input.__dict__['NumLoops_list']
            DDS_Waveform = input.__dict__['Waveform_list']

            #raise LabscriptError(pulse_list_2)
        DDS_table = [DDS_channels, DDS_NumPoints, DDS_NumLoops, DDS_Waveform]
            #raise LabscriptError(DDS_table)
        #raise LabscriptError(DDS_table)
        return DDS_table

    def generate_code(self, hdf5_file):
        IntermediateDevice.generate_code(self, hdf5_file)
        DDS_set = {}
        for device in self.child_devices:
            if isinstance(device, (DDS, ChaseAWGDDS)):
                DDS_set[device.connection] = device 
        DDStable = self._make_ChaseAWG_settings_table(DDS_set)
        #raise LabscriptError(DDStable)
        dt = h5py.string_dtype(encoding='utf-8') 
        grp = self.init_device_group(hdf5_file)

        if DDStable is not None:
            grp.create_dataset('DDS', data=DDStable, compression=config.compression)


''' from labscript_devices import labscript_device, BLACS_tab, BLACS_worker
from labscript import IntermediateDevice, DigitalOut, AnalogOut, config, DDSQuantity, DDS
import numpy as np


class ChaseAWGDDS(DDSQuantity):
    description = 'ChaseAWGDDS'
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

        self.pulse_list_1 = [ ]
        self.pulse_list_2 = [ ]
    def add_pulse(self,channel, NumPoints, NumLoops, Waveform):
        if channel == 1:
            self.pulse_list_1.append((NumPoints, NumLoops, Waveform))
        else:
            self.pulse_list_2.append((NumPoints, NumLoops, Waveform))

class ChaseAWG(IntermediateDevice):

    description = 'Dummy IntermediateDevice'
    clock_limit = 1e6

    # If this is updated, then you need to update generate_code to support whatever types you add
    allowed_children = [DigitalOut, AnalogOut]

    def __init__(self, name, parent_device, BLACS_connection='dummy_connection', **kwargs):
        self.BLACS_connection = BLACS_connection
        IntermediateDevice.__init__(self, name, parent_device, **kwargs)


    def _make_ChaseAWG_settings_table(self, inputs):
        """Collect analog input instructions and create the acquisition table"""
        if not inputs:
            return None

        for connection, input in inputs.items():
            pulse_list_1 = input.__dict__['pulse_list_1']
            pulse_list_2 = input.__dict__['pulse_list_2']

        settings = [(pulse_list_1, pulse_list_2)]
        settings_dtypes = [
            ('pulse_list_1', list),
            ('pulse_list_2', list)
        ]
        settings_table = np.empty(len(settings), dtype=settings_dtypes)
        for i, acq in enumerate(settings):
            settings_table[i] = acq
        return settings

    def generate_code(self, hdf5_file):
        IntermediateDevice.generate_code(self, hdf5_file)
        DDS_set = {}
        for device in self.child_devices:
            if isinstance(device, (DDS, ChaseAWGDDS)):
                DDS_set[device.connection] = device
        DDStable = self._make_ChaseAWG_settings_table(DDS_set)

        grp = self.init_device_group(hdf5_file)
        if DDStable is not None:
            grp.create_dataset('DDS', data=DDStable, compression=config.compression)'''