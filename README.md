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