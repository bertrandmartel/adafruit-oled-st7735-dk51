# Adafruit OLED 1.69 display on DK51 board

Integration of Adafruit OLED ST7735 1.8 display on Nordic Semiconductor NRF51 DK board

![screenshot](img/adafruit-oled-dk51.jpg)

## Scope

* SPI communication between NRF51 board & display
* draw pixel
* fill screen
* joystick (5 actions)
* draw 128x160 bitmap 16 bit 

## Setup/Installation

* follow SDK/Toolchain Installation steps section of <a href="https://gist.github.com/akinaru/a38315c5fe79ec5c8c6a9ed90b8df260#installation-steps">this tutorial</a>

* specify NRF51 SDK directory with :

```
export NRF51_SDK_DIR=/path/to/sdk
```

## Build

```
cd pca10028
make
```

## Upload

```
//erase firmware
nrfjprog --family  nRF51 -e

//upload firmware
nrfjprog --family  nRF51 --program _build/nrf51422_xxac.hex

//start firmware
nrfjprog --family  nRF51 -r
```

To debug your code : <a href="https://gist.github.com/akinaru/a38315c5fe79ec5c8c6a9ed90b8df260#debug-your-code">check this link</a>

## External projects

* https://github.com/NordicSemiconductor/nrf51-ADC-examples
* https://github.com/adafruit/Adafruit-ST7735-Library
* https://github.com/NordicSemiconductor/nrf51-ble-app-lbs
* Nordic Semicondictor NRF51 SDK SPI sample code
* https://github.com/akinaru/spi-master-slave-dk51

## License

The MIT License (MIT) Copyright (c) 2016 Bertrand Martel