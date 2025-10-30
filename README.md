# Fafnir
Propulsion Engine Controller



# Prerequisites
Follow [zephyr getting started guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)

# Build 

```bash
cd zephyr-firmware
west update
cd example-application
west build -b fafnir app --pristine
west flash
```