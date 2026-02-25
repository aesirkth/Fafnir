# Fafnir
Propulsion Engine Controller

# Prerequisites
Follow [zephyr getting started guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)

# Build 

```bash
cd zephyr-firmware
west update
cd fafnir
west build -b fafnir app --pristine
west flash
```

# Build for native simulation
```bash
docker run -ti -v /Volumes/external/zephyrproject/:/workdir -v /Volumes/external/aesir/Fafnir/zephyr-firmware/:/app ghcr.io/zephyrproject-rtos/zephyr-build:main
cd /app/fafnir
west build -b native_sim/native/64 app --pristine -- -DEXTRA_CONF_FILE=debug.conf
./build/zephyr/zephyr.exe
```
or
```bash
west build -b native_sim/native/64 app -- -DEXTRA_CONF_FILE=debug.conf && ./build/zephyr/zephyr.exe
```

## Using shell in simulation

You can send can messages like this
```bash
uart: ~$ can send can_loopback0 ID DATA
```

For example, running
```bash
uart: ~$ can send can_loopback0 123 0
```
will tell Fafnir to go into state STATE_INIT.

Running 
```bash
uart: ~$ can send can_loopback0 124 0 1
```
will use the override feature to set N2 Valve HIGH.

# CAN message specification to Fafnir

| CAN ID | byte | data                         | type  |
| ------ | ---- | ---------------------------- | ----- |
| 0x123  | 0    | launch sequence number (1-6) | uint8 | (setting states mode)
| 0x124  | 0    | Solenoid ID (1-4)            | uint8 | (override mode)
|        | 1    | 1 == open, 0 == closed       | bool  |

Note that the launch sequence number corresponds to the launch sequence number as given in https://aesir.slab.com/posts/mjollnir-launch-sequence-2juh5ekp

Note that the Solenoid ID corresponds to the solenoid ports on the Fafnir board.
Mjollnir only has two solenoids which correspond to:
```
Solenoid ID 2 => Bypass valve
Solenoid ID 3 => Pilot valve
Solenoid ID 4 => Control vent
```
Solenoid IDs 1 and 4 are not connected and don't do anything.
