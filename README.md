# STM32H7 Demo Code

A demonstration project for STM32H753xx microcontrollers featuring USB CDC communication, logging utilities, and various peripheral drivers.

## Overview

This project is built for the STM32H7xx Nucleo board and includes:
- USB Device CDC (Virtual COM Port)
- Logging infrastructure
- Bit-bang SPI implementation
- Lightweight ring buffer (lwrb)
- JSON parsing (jsmn)
- Utility functions

## Hardware Requirements

- STM32H753xx Nucleo board
- ST-Link programmer/debugger
- USB cable for CDC communication

## Software Requirements

- CMake 3.22 or higher
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- OpenOCD (for flashing and debugging)
- STM32CubeMX (optional, for hardware configuration)

## Project Structure

```
├── Core/               # Application source code
│   ├── Inc/           # Header files
│   └── Src/           # Source files
├── Drivers/           # STM32 HAL and CMSIS drivers
│   ├── CMSIS/        # CMSIS core files
│   ├── STM32H7xx_HAL_Driver/  # HAL drivers
│   └── BSP/          # Board Support Package
├── Middlewares/       # USB device library
├── USB_DEVICE/        # USB CDC application
├── cmake/             # CMake toolchain files
└── build/             # Build output directory
```

## Building the Project

### Configure (Debug)

```bash
cmake --preset Debug
```

Or use the VS Code task:
```
CMake: Configure (Debug)
```

### Build (Debug)

```bash
cmake --build build/Debug --config Debug --target all -j 10
```

Or use the VS Code task:
```
CMake: Build (Debug)
```

### Build (Release)

```bash
cmake --build build/Release --config Release --target all -j 10
```

Or use the VS Code task:
```
CMake: Build (Release)
```

### Clean Build

```bash
cmake --build build/Debug --target clean
```

Or use the VS Code task:
```
CMake: Clean
```

## Flashing the Firmware

### Using OpenOCD

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program build/Debug/demo-code.hex reset exit"
```

Or use the VS Code task:
```
Flash Firmware
```

## Debugging

### Start OpenOCD Server

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg
```

Or use the VS Code task:
```
Start OpenOCD
```

Then connect with GDB or use VS Code's debug configuration.

## Features

### USB CDC (Virtual COM Port)
The project includes USB CDC support for serial communication over USB. After flashing, the device will appear as a virtual COM port on your computer.

### Logging System
A lightweight logging system is included (`logging.h/c`) for debug output over the USB CDC interface.

### Bit-bang SPI
Optional bit-bang SPI implementation available alongside the HAL SPI driver.

### Utilities
- **lwrb**: Lightweight ring buffer library for efficient data handling
- **jsmn**: Minimalistic JSON parser
- **util**: General utility functions

## Configuration

Hardware configuration can be modified using STM32CubeMX by opening the `demo-code.ioc` file. After making changes, regenerate the code while preserving user code sections.

## VS Code Integration

This project includes pre-configured VS Code tasks and launch configurations for:
- Building (Debug and Release)
- Cleaning
- Flashing firmware
- Starting OpenOCD for debugging

### Generating VS Code Files

To generate the `.vscode` files (`launch.json` and `tasks.json`) required for building, debugging, and programming the firmware, create platform.json using the example as a guide then run the following script:

```bash
python ./generate_vscode_files.py
```

This script ensures that the VS Code environment is correctly set up for the project. Access tasks via `Terminal > Run Task...` or use the keyboard shortcut.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

The STM32 HAL drivers and CMSIS files are Copyright © STMicroelectronics and are subject to their own license terms.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Author

STM32 Development Team

## Acknowledgments

- STMicroelectronics for the HAL and CMSIS libraries
- lwrb library contributors
- jsmn library contributors
