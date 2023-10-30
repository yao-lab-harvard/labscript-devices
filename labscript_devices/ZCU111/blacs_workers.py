from blacs.tab_base_classes import Worker
import time
import re
import labscript_utils.h5_lock, h5py

class ZCU111Worker(Worker):

    def init(self):

        global h5py; 

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
