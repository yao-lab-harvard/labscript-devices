#####################################################################
#                                                                   #
# /labscript_devices/DummyPseudoclock/blacs_worker.py               #
#                                                                   #
# Copyright 2017, Christopher Billington                            #
#                                                                   #
# This file is part of labscript_devices, in the labscript suite    #
# (see http://labscriptsuite.org), and is licensed under the        #
# Simplified BSD License. See the license.txt file in the root of   #
# the project for the full license.                                 #
#                                                                   #
#####################################################################
import time
import labscript_utils.h5_lock
import h5py
from blacs.tab_base_classes import Worker
import labscript_utils.properties as properties
import os
def awg_pulse_generator(pulse_list_1 = None, pulse_list_2 = None):
        #pulse_list_1 = [ (NumPoints_1, NumLoops_1, '2047.5 + 2047.5*sin( 2.0*pi* x/(32) )' ), (NumPoints_2, NumLoops_2, 2047.5 + 2047.5*sin( 2.0*pi* x/(64) ) )     ]
        s_1 = '#include <iostream>\n#include <stdio.h>\n#include <stdlib.h>\n#include <math.h>\n#include "dax22000_lib_DLL32.h"\n'
        s_2 = 'typedef struct {PVOID SegmentPtr; DWORD NumPoints; DWORD NumLoops; DWORD TrigEn;} SegmentStruct; int main(int argc, char** argv){DWORD NumCards = 0;DWORD CardNum = 1;DWORD Chan = 1;int x;double Actual_Frequency; double pi = 3.14159265358979;'

        s_3 = ''
        if pulse_list_1 != None: 
            s_3  = 'SegmentStruct SegmentArray1[' + str(len(pulse_list_1))+'];'

            for i in range(len(pulse_list_1)):
                temp_string = 'DWORD MemoryDepth' + str(i) +' = ' +str(pulse_list_1[i][0]) + ';WORD TempArray' + str(i) + '['  + str(pulse_list_1[i][0]) +'];for (x=0; x < (MemoryDepth'+str(i) +'); x++) {'
                temp_string_2 = 'TempArray' + str(i) + '[x] = (unsigned int) ( ceil(' + pulse_list_1[i][2] + '));}'

                s_3 = s_3 + temp_string+temp_string_2
                segment_i = 'Segment' + str(i)
                temp_string_3 = 'SegmentStruct ' + segment_i + ';' + segment_i +'.SegmentPtr = TempArray' + str(i) + ';'
                temp_string_4 = segment_i +'.NumPoints =' + str(pulse_list_1[i][0]) + ';'
                temp_string_5 = segment_i +'.NumLoops =' + str(pulse_list_1[i][1]) + ';'
                temp_string_6 = segment_i + '.TrigEn = 1;'
                segment_array_1 = 'SegmentArray1' + '[' + str(i) + '] =' + segment_i +';'

                s_3 = s_3 + temp_string_3 + temp_string_4 + temp_string_5 + temp_string_6 + segment_array_1
                
        if pulse_list_2 != None:
            s_3 += 'SegmentStruct SegmentArray2[' + str(len(pulse_list_2))+'];'
            for i in range(len(pulse_list_2)): 
                temp_string = 'DWORD MemoryDepth' + str(i) +' = ' +str(pulse_list_2[i][0]) + ';WORD TempArrayB' + str(i) + '['  + str(pulse_list_2[i][0]) +'];for (x=0; x < (MemoryDepth'+str(i)+'); x++) {'
                temp_string_2 = 'TempArrayB' + str(i) + '[x] = (unsigned int) ( ceil(' + pulse_list_2[i][2] + '));}'

                s_3 = s_3 + temp_string+temp_string_2
                segment_i = 'SegmentB' + str(i)
                temp_string_3 = 'SegmentStruct ' + segment_i + ';' + segment_i +'.SegmentPtr = TempArray' + str(i) + ';'
                temp_string_4 = segment_i +'.NumPoints =' + str(pulse_list_2[i][0]) + ';'
                temp_string_5 = segment_i +'.NumLoops =' + str(pulse_list_2[i][1]) + ';'
                temp_string_6 = segment_i + '.TrigEn = 1;'
                segment_array_2 = 'SegmentArray2' + '[' + str(i) +'] =' + segment_i +';'
                s_3 = s_3 + temp_string_3 + temp_string_4 + temp_string_5 + temp_string_6 + segment_array_2 
        
        s_4 = 'NumCards = DAx22000_GetNumCards();if (NumCards != 1) exit(0);x = DAx22000_Open(1);x = DAx22000_Initialize(1);Actual_Frequency = DAx22000_SetClkRate(1, 2.5e9);x = DAx22000_SelExtTrig(1, true);'
        s_5 = ''
        if pulse_list_1 != None:
            s_5 = 'x = DAx22000_CreateSegments(1, 1,' +str(len(pulse_list_1)) + ', 2047, 2047,SegmentArray1, false);'
        if pulse_list_2 != None:
            s_5 = s_5 + 'x = DAx22000_CreateSegments(1, 2,' +str(len(pulse_list_2)) + ', 2047, 2047,SegmentArray2, false);'
        
        
        s_6 = 'DAx22000_Run(1, false);return 0;}'


        f = open("send_pulse.cpp", "w")
        f.write(s_1+s_2+s_3+s_4+s_5+s_6)
        f.close()
        f = open("send_pulse.cpp", "r")

        print(f.read())
        f.close()
        #print(os.getcwd())
        os.system('g++ -o send_pulse.exe send_pulse.cpp ftd2xx.dll dax22000_lib_DLL32.dll')
        os.startfile("C:Users/Yao Lab/send_pulse.exe")
class ChaseAWGWorker(Worker):
    def init(self):
        pass

    
    def program_manual(self, front_panel_values):
        return front_panel_values 

    def transition_to_buffered(self, device_name, h5file, initial_values, fresh):
        pulse_list_1 = []
        pulse_list_2 = []
        with h5py.File(h5file,'r') as h5_file:
            group = h5_file['devices/%s'%device_name]
            DDS = group['DDS']
            self.logger.info(DDS[0])
            for i in range(len(DDS[0])):
                if int(DDS[0][i]) == 1:
                    pulse_list_1.append( ( int(DDS[1][i]), int(DDS[2][i]), DDS[3][i].decode('utf-8')))
                else: 
                    pulse_list_2.append( (int(DDS[1][i]), int(DDS[2][i]), DDS[3][i].decode('utf-8')))

        self.logger.info(pulse_list_1)
        self.logger.info(pulse_list_2)
        if len(pulse_list_1) == 0:
            pulse_list_1 = None
        elif len(pulse_list_2) == 0:
            pulse_list_2 = None
        awg_pulse_generator(pulse_list_1, pulse_list_2)
        return initial_values

    def transition_to_manual(self,abort = False):
        return True

    def abort_transition_to_buffered(self):
        return self.transition_to_manual(True)
        
    def abort_buffered(self):
        return self.transition_to_manual(True)

    def shutdown(self):
        pass


''' 
#####################################################################
#                                                                   #
# /labscript_devices/DummyPseudoclock/blacs_worker.py               #
#                                                                   #
# Copyright 2017, Christopher Billington                            #
#                                                                   #
# This file is part of labscript_devices, in the labscript suite    #
# (see http://labscriptsuite.org), and is licensed under the        #
# Simplified BSD License. See the license.txt file in the root of   #
# the project for the full license.                                 #
#                                                                   #
#####################################################################


import os
def awg_pulse_generator(pulse_list_1 = None, pulse_list_2 = None):
    #pulse_list_1 = [ (NumPoints_1, NumLoops_1, '2047.5 + 2047.5*sin( 2.0*pi* x/(32) )' ), (NumPoints_2, NumLoops_2, 2047.5 + 2047.5*sin( 2.0*pi* x/(64) ) )     ]
    s_1 = '#include <iostream>\n#include <stdio.h>\n#include <stdlib.h>\n#include <math.h>\n#include "dax22000_lib_DLL32.h"\n'
    s_2 = 'typedef struct {PVOID SegmentPtr; DWORD NumPoints; DWORD NumLoops; DWORD TrigEn;} SegmentStruct; int main(int argc, char** argv){DWORD NumCards = 0;DWORD CardNum = 1;DWORD Chan = 1;int x;double Actual_Frequency; double pi = 3.14159265358979;'

    s_3 = ''
    if pulse_list_1 != None: 
        s_3  = 'SegmentStruct SegmentArray1[' + str(len(pulse_list_1))+'];'

        for i in range(len(pulse_list_1)):
            temp_string = 'DWORD MemoryDepth' + str(i) +' = ' +str(pulse_list_1[i][0]) + ';WORD TempArray' + str(i) + '['  + str(pulse_list_1[i][0]) +'];for (x=0; x < (MemoryDepth'+str(i) +'); x++) {'
            temp_string_2 = 'TempArray' + str(i) + '[x] = (unsigned int) ( ceil(' + pulse_list_1[i][2] + '));}'

            s_3 = s_3 + temp_string+temp_string_2
            segment_i = 'Segment' + str(i)
            temp_string_3 = 'SegmentStruct ' + segment_i + ';' + segment_i +'.SegmentPtr = TempArray' + str(i) + ';'
            temp_string_4 = segment_i +'.NumPoints =' + str(pulse_list_1[i][0]) + ';'
            temp_string_5 = segment_i +'.NumLoops =' + str(pulse_list_1[i][1]) + ';'
            temp_string_6 = segment_i + '.TrigEn = 1;'
            segment_array_1 = 'SegmentArray1' + '[' + str(i) + '] =' + segment_i +';'

            s_3 = s_3 + temp_string_3 + temp_string_4 + temp_string_5 + temp_string_6 + segment_array_1
            
    if pulse_list_2 != None:
        s_3 += 'SegmentStruct SegmentArray2[' + str(len(pulse_list_2))+'];'
        for i in range(len(pulse_list_2)): 
            temp_string = 'DWORD MemoryDepth' + str(i) +' = ' +str(pulse_list_2[i][0]) + ';WORD TempArrayB' + str(i) + '['  + str(pulse_list_2[i][0]) +'];for (x=0; x < (MemoryDepth'+str(i)+'); x++) {'
            temp_string_2 = 'TempArrayB' + str(i) + '[x] = (unsigned int) ( ceil(' + pulse_list_2[i][2] + '));}'

            s_3 = s_3 + temp_string+temp_string_2
            segment_i = 'SegmentB' + str(i)
            temp_string_3 = 'SegmentStruct ' + segment_i + ';' + segment_i +'.SegmentPtr = TempArray' + str(i) + ';'
            temp_string_4 = segment_i +'.NumPoints =' + str(pulse_list_2[i][0]) + ';'
            temp_string_5 = segment_i +'.NumLoops =' + str(pulse_list_2[i][1]) + ';'
            temp_string_6 = segment_i + '.TrigEn = 1;'
            segment_array_2 = 'SegmentArray2' + '[' + str(i) +'] =' + segment_i +';'
            s_3 = s_3 + temp_string_3 + temp_string_4 + temp_string_5 + temp_string_6 + segment_array_2 
    
    s_4 = 'NumCards = DAx22000_GetNumCards();if (NumCards != 1) exit(0);x = DAx22000_Open(1);x = DAx22000_Initialize(1);Actual_Frequency = DAx22000_SetClkRate(1, 2.5e9);x = DAx22000_SelExtTrig(1, true);'
    s_5 = ''
    if pulse_list_1 != None:
        s_5 = 'x = DAx22000_CreateSegments(1, 1,' +str(len(pulse_list_1)) + ', 2047, 2047,SegmentArray1, false);'
    if pulse_list_2 != None:
        s_5 = s_5 + 'x = DAx22000_CreateSegments(1, 2,' +str(len(pulse_list_2)) + ', 2047, 2047,SegmentArray2, false);'
    
    
    s_6 = 'DAx22000_Run(1, false);std::cout << "outputting";std::cin.get();DAx22000_Stop(1);DAx22000_Close(1);DAx22000_PWR_DWN(1);return 0;}'


    f = open("send_pulse.cpp", "w")
    f.write(s_1+s_2+s_3+s_4+s_5+s_6)
    f.close()
    f = open("send_pulse.cpp", "r")

    print(f.read())
    f.close()

    os.system('cd/')
    os.system('cd Users/nateu/Yao Group')
    os.system('g++ -o send_pulse.exe send_pulse.cpp ftd2xx.dll dax22000_lib_DLL32.dll')
    os.startfile("C:Users/nateu/Yao Group/send_pulse.exe")

import time
import labscript_utils.h5_lock
import h5py
from blacs.tab_base_classes import Worker
import labscript_utils.properties as properties
import os
class ChaseAWGWorker(Worker):
    def init(self):
        pass

    def program_manual(self, front_panel_values):
        return front_panel_values 

    def transition_to_buffered(self, device_name, h5file, initial_values, fresh):
        with h5py.File(h5file,'r') as h5_file:
            group = hdf5_file['devices/%s'%device_name]

            pulse_program = group['PULSE_PROGRAM'][2:]
            DDS = group['DDS']
            pulse_list_1 = DDS[0][0]
            pulse_list_2 = DDS[0][1]
            awg_pulse_generator(pulse_list_1, pulse_list_2)
        #os.startfile("C:Users/nateu/Downloads/dax22000_user_sw/dax22000_user_sw/Win7_32_bit/test_dll.exe")

        return initial_values

    def transition_to_manual(self,abort = False):
        return True

    def abort_transition_to_buffered(self):
        return self.transition_to_manual(True)
        
    def abort_buffered(self):
        return self.transition_to_manual(True)

    def shutdown(self):
        pass


'''