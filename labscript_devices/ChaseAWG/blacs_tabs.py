from blacs.device_base_class import DeviceTab, define_state, MODE_BUFFERED
class ChaseAWGTab(DeviceTab):
    def initialise_GUI(self):
        self.create_worker("main_worker","labscript_devices.ChaseAWG.blacs_workers.ChaseAWGWorker",{})
        self.primary_worker = "main_worker"