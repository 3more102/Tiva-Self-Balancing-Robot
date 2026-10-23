# TI Code Composer Studio + TivaWare setup

Target: `TM4C123GH6PM` on `EK-TM4C123GXL` (ARM Cortex-M4F).

1. Install TI Code Composer Studio and the TM4C/TivaWare software package. See https://www.ti.com/tool/EK-TM4C123GXL .
2. Create a new CCS project for the exact device, with the correct startup file/vector table and linker command file. For first-time setup, start from TI's TivaWare `blinky` example for EK-TM4C123GXL so startup/linker/device options are already appropriate, then add these source files rather than mixing incompatible example main files.
3. Add source files `src/main.c` and `src/controller.c`; add `include/` to header search paths; add TI TivaWare root to includes (for `inc/hw_memmap.h`, `driverlib/...`). Ensure TivaWare's `driverlib` library for your compiler/ABI is linked, and math floating support matches FPU settings.
4. Keep `MOTOR_OUTPUT_ALLOWED 0` until hardware tests prove safe wiring and control direction. This zero means STBY is never asserted during running.
5. Debug with wheels removed from the floor. The sample loop is polled from Timer0A; excessive I2C latency can overrun a 5 ms step; instrument and profile on target before tuning.

**Known limitations:** board application is not compiled/tested in CCS in this repository; default model does not model motors/gears/battery/saturation; no wheel encoders, UART telemetry, independent watchdog, physical E-stop, or automated real-target tests yet. The MPU6050 I2C bus assumes proper 3.3 V pull-ups. Parameterize and independently verify all motor orientations before energizing. If you need real code coverage or tuning, add UART logger and record verified timestamps.

**Caution:** Do not power motors directly from Tiva GPIO, do not feed 5 V into MCU pins, and do not power high-current motors from USB/3.3 V MCU output. Battery grounds must be common.
