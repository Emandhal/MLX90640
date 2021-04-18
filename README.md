:warning: WORK IN PROGRESS, ONLY PARAMETERS EXTRACTION HAVE BEEN TESTED YET!

# MLX90640
MLX90640 is a fully hardware independant **Driver** primarily aimed at embedded world

# Presentation
This driver only takes care of configuration and check of the internal registers and the formatting of the communication with the device. That means it does not directly take care of the physical communication, there is functions interfaces to do that.
Each driver's functions need a device structure that indicate with which device he must threat and communicate. Each device can have its own configuration.

## Feature

This driver has been designed to:
* Be fully configurable (all known features of the MLX90640 are managed)
* Detect which one of the MLX90640BAA or MLX90640BAB is connected
* Have no limit of configuration except the ones imposed by the device
* Manage devices completely independently
* Prevent all configuration errors
* Can be used with EEPROM data already extracted and saved in flash (reduce the use of RAM used)
* Can be used with Parameters already calculated and saved in flash (reduce the use of RAM used)

## Limitations

To use this driver and device, you need:
* At least 20k of RAM. The driver itself needs 1664byte for EEPROM, 106+10704bytes for parameters, 1666bytes for the data frame, and 3080 for the frame result
* An FPU on the CPU else the driver will be very slow and only slow refresh speed can be achieved

# Usage

## Installation

### Get the sources
Get and add the 4 following files to your project
```
MLX90640.c
MLX90640.h
Conf_MLX90640_Template.h
ErrorsDef.h
```
Copy or rename the file `Conf_MLX90640_Template.h` to `Conf_MLX90640.h`... Et voila!

## Others directories

### Tests\ directory
The **Tests** folder contains an example of use on the _SAMV71 Xplained Ultra board_
See the `main.c` header for the hardware setup.

# Configuration
To set up one or more devices in the project, you must:
* Configure the driver (`Conf_MLX90640.h`) which will be the same for all devices but modify only its behavior in the project
* Create and define the configuration of as many device structures as there are devices to use
* Create and define a configuration for each device. Multiple devices can share the same configuration
* Initialize the device with the configuration structure previously defined

**All is explained in the MLX90640 driver library guide** available soon

# Driver usage

**All is explained in the MLX90640 driver library guide** available soon