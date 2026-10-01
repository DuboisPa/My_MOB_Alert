# Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
# Get-ExecutionPolicy -List
# 0x160000 = 1 441 792 en décimal

# Partitions schemes are in C:/Users/%USERNAME%/AppData/Local/Arduino15/packages/esp32/hardware/esp32/3.3.11/tools/partitions
# copy espool.exe from C:\Users\%USERNAME%\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.3.1  (or other library number)


# minimal spiffs
# Name,   Type, SubType, Offset,  Size, Flags
#nvs,      data, nvs,     0x9000,  0x5000,
#otadata,  data, ota,     0xe000,  0x2000,
#app0,     app,  ota_0,   0x10000, 0x1E0000,
#app1,     app,  ota_1,   0x1F0000,0x1E0000,
#spiffs,   data, spiffs,  0x3D0000,0x20000,
#coredump, data, coredump,0x3F0000,0x10000,

../mkspiffs -c data -b 4096 -p 256 -s 0x20000 spiffs.bin
#./esptool.exe --chip esp32 -b 921600 -a hard-reset write-flash -z 0x3D0000 spiffs.bin

