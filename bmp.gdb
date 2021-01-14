# Attach a Black Magic Probe to the connected microcontroller
# Requires that the BMP associated udev rules are installed
tar ext /dev/ttyBmpGdb
mon tpwr enable
mon swdp_scan
attach 1
set mem inaccessible-by-default off