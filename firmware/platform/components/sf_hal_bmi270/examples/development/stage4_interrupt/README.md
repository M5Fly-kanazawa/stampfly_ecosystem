<!--
SPDX-License-Identifier: MIT

Copyright (c) 2025 Kouhei Ito

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-->

# BMI270 Stage 4: Interrupt-based Data Reading

This example demonstrates interrupt-driven data acquisition from the BMI270 sensor using the Data Ready interrupt feature.

## Features

- **Data Ready Interrupt**: BMI270 generates an interrupt when new sensor data is available
- **ESP32 GPIO Interrupt**: Uses ESP32's GPIO interrupt system to detect BMI270 interrupts
- **Event-Driven Architecture**: No polling, CPU-efficient operation
- **Real-time Data Output**: Teleplot-compatible streaming format
- **Interrupt Statistics**: Tracks interrupt count for monitoring

## Hardware Configuration

### M5StampFly Pin Connections

| BMI270 Pin | ESP32-S3 GPIO | Function |
|------------|---------------|----------|
| MOSI       | GPIO 14       | SPI Data Out |
| MISO       | GPIO 43       | SPI Data In |
| SCK        | GPIO 44       | SPI Clock |
| CS         | GPIO 46       | Chip Select |
| INT1       | **GPIO 11**   | **Interrupt Output** |

**Note**: On M5StampFly hardware, BMI270's INT1 pin is connected to GPIO 11. INT2 pin is not connected.

## Sensor Configuration

### Accelerometer
- **Range**: ±4g
- **ODR**: 100 Hz
- **Filter**: Performance mode

### Gyroscope
- **Range**: ±1000 °/s
- **ODR**: 200 Hz
- **Filter**: Performance mode

### Interrupt Configuration
- **Pin**: INT1
- **Mode**: Active High, Push-Pull
- **Latch**: Non-latched (Pulse mode)
- **Type**: Data Ready interrupt

## How It Works

1. **Initialization**: BMI270 is initialized with accelerometer and gyroscope enabled
2. **Interrupt Setup**:
   - BMI270's INT1 pin is configured as active-high, push-pull output
   - Data Ready interrupt is mapped to INT1
   - ESP32 GPIO is configured to trigger on rising edge
3. **ISR Execution**: When BMI270 has new data:
   - INT1 pin goes HIGH
   - ESP32 GPIO interrupt fires
   - ISR sends notification to queue
4. **Data Reading**: Main task:
   - Waits on queue (blocking, no CPU usage)
   - Receives interrupt notification
   - Reads sensor data
   - Outputs to Teleplot

## Output Configuration

By default, the program outputs only physical values (g, °/s, °C) for cleaner visualization. To enable raw sensor values (LSB), modify [main.c:35](main/main.c#L35):

```c
#define OUTPUT_RAW_VALUES   1  // Set to 1 to output raw sensor values (LSB)
```

## Expected Output

### Default Output (Physical Values Only)
The program outputs 8 data channels in Teleplot format at the sensor's ODR (gyro at 200Hz):

```
>acc_x:-0.0299
>acc_y:0.0156
>acc_z:1.0000
>gyr_x:0.366
>gyr_y:-0.244
>gyr_z:0.091
>temp:25.00
>int_count:1234
```

### With Raw Values Enabled (OUTPUT_RAW_VALUES=1)
15 data channels including raw sensor values:

```
>acc_raw_x:-245
>acc_raw_y:128
>acc_raw_z:8192
>acc_x:-0.0299
>acc_y:0.0156
>acc_z:1.0000
>gyr_raw_x:12
>gyr_raw_y:-8
>gyr_raw_z:3
>gyr_x:0.366
>gyr_y:-0.244
>gyr_z:0.091
>temp_raw:1024
>temp:25.00
>int_count:1234
```

Every 100 interrupts, a status message is printed:
```
I (12345) BMI270_STAGE4: Interrupt count: 100
I (13456) BMI270_STAGE4: Interrupt count: 200
```

## Building and Running

```bash
# Navigate to this example
cd examples/stage4_interrupt

# Set up ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Build
idf.py build

# Flash and monitor
idf.py -p /dev/ttyACM0 flash monitor
```

## Advantages Over Polling (Stage 3)

| Aspect | Polling (Stage 3) | Interrupt (Stage 4) |
|--------|------------------|-------------------|
| CPU Usage | High (continuous checking) | Low (event-driven) |
| Power Consumption | Higher | Lower |
| Timing Accuracy | Depends on polling interval | Exact (sensor ODR) |
| Latency | Up to polling interval | Minimal (µs range) |
| Code Complexity | Simple | Moderate (ISR + queue) |

## Troubleshooting

### No interrupts received
1. **Check INT1 GPIO connection**: Verify GPIO 11 is properly connected to BMI270's INT1 pin (standard on M5StampFly)
2. **Check INT1 configuration**: Verify INT1_IO_CTRL register (should show output_en=1, active_high=1)
3. **Check interrupt mapping**: Verify INT_MAP_DATA register (bit 2 should be set)
4. **Check GPIO pull-down**: Ensure GPIO has pull-down enabled to avoid floating state

### Interrupt rate too high/low
- Verify sensor ODR configuration (ACC_CONF, GYR_CONF registers)
- Data Ready interrupt triggers at the **higher** of accel/gyro ODR (200Hz in this example)

### Data reading errors
- Ensure proper SPI timing (10 MHz, Mode 0)
- Check for SPI bus contention with other devices (PMW3901)

## Technical Details

### Data Ready Interrupt Behavior

The BMI270 Data Ready interrupt:
- Triggers when **both** accelerometer and gyroscope data are updated
- Frequency = max(accel_ODR, gyro_ODR)
- In this example: max(100Hz, 200Hz) = **200Hz**
- Pulse width: ~2.5 µs (at 200Hz ODR)

### ESP32 GPIO Interrupt

- **Interrupt Type**: `GPIO_INTR_POSEDGE` (rising edge)
- **ISR Location**: IRAM (fast execution)
- **Queue Size**: 10 events (handles burst interrupts)
- **Pull-down**: Enabled to prevent floating input

### Timing Characteristics

Typical latencies:
- INT1 pulse width: ~2.5 µs
- GPIO interrupt latency: ~1-5 µs
- Queue send: ~1 µs
- Total interrupt-to-read latency: **< 10 µs**

## Next Steps

- **Stage 5**: FIFO-based high-speed reading for maximum throughput
- **Advanced**: Motion detection interrupts (any-motion, no-motion, step counter)
- **Power Optimization**: Low-power modes with wake-on-motion

## References

- [BMI270 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi270.html)
- [ESP32-S3 GPIO & Interrupts](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/gpio.html)
- [FreeRTOS Queues](https://www.freertos.org/a00018.html)
