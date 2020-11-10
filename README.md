ESP-IDF BME-280 Driver
====================
Wrapper around the driver provided by Bosch (included as submodule).

This consists of a minimal driver located in the components directory and a demo application in the main directory.
Tested with ESP-IDF v4.0, v4.3-dev on ubuntu 20.04

### Usage
```
git clone https://github.com/jmare/esp-idf-bme280 --recursive
cd esp-idf-bme280
idf.py build flash monitor
```

### Example Output
```
I (340) MAIN: MAIN ENTRY
25.34 deg C, 1007.80 hPa, 44.30%
25.34 deg C, 1007.80 hPa, 44.29%
25.34 deg C, 1007.80 hPa, 44.26%
25.34 deg C, 1007.81 hPa, 44.23%
...
```
