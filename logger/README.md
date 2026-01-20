
## Update repo and update submodules

### S3
Zadig
Обрати USB JTAG/serial debug unit (Interface 0) → залишити драйвер USB Serial (CDC) (це Serial порт).
Обрати USB JTAG/serial debug unit (Interface 2) → поставити драйвер WinUSB або libusbK.

### S2
#### Upload and monitor by USB
- press boot button
- then connect USB cable to board
- release boot button
- download Zadig https://zadig.akeo.ie/
- oped Zadig and select "Options->List all devices", select ESP32-S2 (Interface 0),  select "USB Serial (CDC)" install
- select select ESP32-S2 (Interface 2) "WinUSB"  driver,  Install
- reboot
- do the same for regular boar mode
- switch board mode to boot
- build project
- select COM port in platform io
- upload
- reset
- next Uploads and monitor should automatically upload and listen serial monitor

## Configuation

There is ```doc/configuration.example.ini```  example  write it to SD card with name ```configuration.ini```

## Board setup

read esp-hints.md file