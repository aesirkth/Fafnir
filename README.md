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

## CAN acknowledges

| CAN ID | byte | data | meaning |
| ------ | ---- | ------  | -----|
| 0x125  | 0   | 0x01    | Changing state! |
|        | 1   |  \<byte\> |  the state we're changing to |
| 0x125  | 0   | 0x02    | Received valid override command! |
|        | 1   | \<byte\> | The first byte we received  |
|        | 2   |  \<byte\> | The second byte we received  |
| 0x125  | 0   | 0xF0    | Recieved invalid state |
| 0x125  | 0   | 0xF1    | Cannot perform ignition command while attempting ignition. (Protection against double send) |
|  | 1   | \<byte\>    | State command we received |
| 0x125  | 0   | 0xF2    | Invalid state command |
| 0x125  | 0   | 0xF3    | Invalid override command |
| 0x125  | 0   | 0xF4    | Invalid pin given in override command |


where `systemState` is of type: Note this is different to the launch sequence numbers from above.
```c
typedef enum {
    STATE_IDLE,
    STATE_INIT,
    STATE_FILL,
    STATE_STOP_FILL,
    STATE_UMBILICAL,
    STATE_N2_PRESSURIZATION,
    STATE_IGNITION_1,
    STATE_IGNITION_2,
    STATE_IGNITION_3,
    STATE_IGNITION_4,
    STATE_SAFE,
    STATE_ABORT_BEFORE_COUNTDOWN,
    STATE_ABORT_AFTER_COUNTDOWN
} State_t;
```

