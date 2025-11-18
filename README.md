# STM32 Blackpill – Battery Management System

Embedded C project for the STM32F411 “Blackpill” board that implements a complete battery management system (BMS). The firmware relies on FreeRTOS together with HAL drivers generated in STM32CubeMX, while an external MCP2515 controller handles CAN communication with the rest of the system.

## Key Features
- CAN bus communication through MCP2515 with UART frame printing for debugging,
- battery charging control with current/energy monitoring,
- active cell balancing and voltage measurements via ADC,
- peripheral control (GPIO, PWM, I2C, SPI, UART) executed inside multiple FreeRTOS tasks,
- easy parameter tuning (target current, end voltage, charger fault diagnostics).

## Repository Layout
- `Core/` – main application sources (e.g. `main.c`, `battery_balance.*`, `battery_charge.*`, CAN handling),
- `Drivers/` – STM32 HAL and CMSIS libraries,
- `Middlewares/` – FreeRTOS sources,
- `cmake/`, `CMakeLists.txt`, `CMakePresets.json` – build configuration,
- `blackpill.ioc` – STM32CubeMX configuration file,
- `startup_stm32f411xe.s`, `STM32F411XX_FLASH.ld` – startup file and linker script,
- `build/` – default location for build artifacts (created after compilation).

## Prerequisites
- CMake (>=3.20),
- `arm-none-eabi-gcc` toolchain together with `arm-none-eabi-gdb`,
- Make or Ninja,
- SWD flashing tool (e.g. ST-Link Utility, OpenOCD).

## Building
```bash
mkdir -p build
cmake -S . -B build
cmake --build build
```
After the build completes, the `build/` directory contains the `.elf`, `.bin`, and build logs.

## Flashing the MCU
1. Connect the Blackpill board via ST-Link or another SWD probe.
2. Create the binary image: `arm-none-eabi-objcopy -O binary build/<name>.elf build/firmware.bin`.
3. Flash it with `st-flash --format binary write build/firmware.bin 0x8000000` or configure OpenOCD for your interface.

## Firmware Architecture
- `main.c` initializes HAL peripherals and starts FreeRTOS tasks: UART handling (`uartTask`), CAN communication (`canTask`), charger control (`chargerTask`), ADC monitoring (`adcTask`), and cell balancing (`balanceTask`).
- Modules inside `Core/Src/` expose helpers for CAN frame processing, charger/balancer control, and data acquisition.
- Runtime parameters (e.g. `end_voltage`, `set_current`, `system_state`) are stored in global variables, and fault diagnostics is available over UART/CAN.

## Next Steps
- Extend UART commands to modify parameters at runtime.
- Add unit tests for logic-heavy modules (e.g. balancing algorithm).
- Consider automatic documentation generation from comments using Doxygen.

Further configuration notes and project assumptions are also documented in `GEMINI.md`.
