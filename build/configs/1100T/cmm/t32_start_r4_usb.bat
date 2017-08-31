@echo off
rem Batch file to start Trace32 for R4
set MX_FW_DIR=Z:\repos\scsc-fw-maxwell-dev\fw

start c:\t32\bin\windows64\t32marm -c r4_config_usb.t32 USB -s r4_device.cmm
