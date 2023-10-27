import labscript_devices

labscript_device_name = 'ChaseAWG'
blacs_tab = 'labscript_devices.ChaseAWG.blacs_tabs.ChaseAWGTab'

labscript_devices.register_classes(
    labscript_device_name=labscript_device_name,
    BLACS_tab=blacs_tab
)