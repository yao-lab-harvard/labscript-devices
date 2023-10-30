#### The BLACS_tab defines the GUI widgets taht control the device.

from labscript_devices import BLACS_tab
from blacs.tab_base_classes import Worker, define_state
from blacs.tab_base_classes import MODE_MANUAL, MODE_TRANSITION_TO_BUFFERED, MODE_TRANSITION_TO_MANUAL, MODE_BUFFERED

from blacs.device_base_class import DeviceTab
from qtutils.qt import QtWidgets

@BLACS_tab
class ZCU111Tab(DeviceTab):
    
    ### Class variables below:
    base_units =    {'freq':'MHz',         'amp':'dBm',       'phase':'Degrees',    'length': "ns"}
    base_min =      {'freq':0,             'amp':-136.0,      'phase':0,            'length': 0}
    base_max =      {'freq':4000.,         'amp':25.0,        'phase':360,          'length':10000}
    base_step =     {'freq':1.0,           'amp':1.0,         'phase':1,            'length':1}
    base_decimals = {'freq':4,             'amp':4,           'phase':3,            'length':3} # TODO: find out what the phase precision is!
    num_DO = 8   
    
    
    def initialise_GUI(self):
        # Capabilities
                # Create status labels
        

        # Create DDS Output objects
        RF_prop = {}
        for i in range(7):
            RF_prop['channel ' + str(i)] = {}
            for subchnl in ['freq', 'amp', 'phase', 'length']:
                RF_prop['channel ' + str(i)][subchnl] = {'base_unit': ZCU111Tab.base_units[subchnl],
                                                    'min': ZCU111Tab.base_min[subchnl],
                                                    'max': ZCU111Tab.base_max[subchnl],
                                                    'step': ZCU111Tab.base_step[subchnl],
                                                    'decimals': ZCU111Tab.base_decimals[subchnl]
                                                    }
        do_prop = {}
        for i in range(ZCU111Tab.num_DO): 
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
        dds_widgets, ao_widgets, do_widgets = self.auto_create_widgets()
        
        # and auto place the widgets in the UI
        self.auto_place_widgets(("RF Output",dds_widgets) ,("Flags",do_widgets,sort))

        # Store the COM port to be used
        ### was self.com_port
        com_port = str(self.settings['connection_table'].find_by_name(self.device_name).BLACS_connection)
        
        worker_initialization_kwargs = {
            "com_port": com_port,
        }

        # Create and set the primary worker
        self.create_worker(
            "main_worker", 
            "labscript_devices.ZCU111.blacs_workers.ZCU111Worker",
            worker_initialization_kwargs,
        )
        
        self.primary_worker = "main_worker"

        # Create status labels
        self.status_label = QtWidgets.QLabel("Status: Unknown")
        self.clock_status_label = QtWidgets.QLabel("Clock status: Unknown")
        self.get_tab_layout().addWidget(self.status_label)
        self.get_tab_layout().addWidget(self.clock_status_label)
