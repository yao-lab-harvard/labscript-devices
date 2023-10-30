from labscript_devices import register_classes

labscript_device_name = 'ZCU111'
blacs_tab = 'labscript_devices.ZCU111.blacs_tabs.ZCU111Tab'
runviewer_parser = 'labscript_devices.ZCU111.runviewer_parsers.'

register_classes(
    labscript_device_name = labscript_device_name,
    BLACS_tab = blacs_tab,
    runviewer_parser = None
)