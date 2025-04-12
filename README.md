
# VL6180X Go Package

This Go package provides an easy-to-use interface for interfacing with VL6180X Time-of-Flight (ToF) sensors via a PCA9548 I²C multiplexer. It allows you to configure up to 8 sensors on a single I²C bus, read distance in millimeters, and obtain ambient light (lux) measurements from each sensor. The package replicates the behavior of the original C code but exposes an idiomatic and configurable API in Go.

## Features

- **Multiple Sensor Support:**  
  Support for up to 8 sensors on the same I²C bus via the PCA9548 multiplexer.

- **Simple Reading Functions:**  
  Use `ReadDistance(sensorIndex int)` to trigger a ranging measurement and `ReadAmbient(sensorIndex int)` to obtain ambient light readings in lux.

- **Automatic Multiplexer Switching:**  
  The package automatically switches the active channel of the PCA9548 multiplexer before communicating with a VL6180X sensor.

- **Configurable Options:**  
  During initialization you can set global defaults or per-sensor values for:
  - **Scaling:**  
    This parameter controls the sensor’s range scaling mode:
      1. **Scaling1 (1x):** Approximately 100 mm maximum range (default).
      2. **Scaling2 (2x):** Approximately 200 mm maximum range with reduced resolution.
      3. **Scaling3 (3x):** Approximately 300 mm maximum range with further reduced precision.
  - **ALSGain:**  
    Controls the analog gain for the ambient light sensor:
      1. Options include `ALSGain20`, `ALSGain10`, `ALSGain5`, `ALSGain2_5`, `ALSGain1_67`, `ALSGain1_25`, `ALSGain1` (factory default), and `ALSGain40`.  
      These are used to boost the sensor’s sensitivity in low light; for example, `ALSGain1` (0x46) is suitable for normal conditions while `ALSGain40` may be useful in very low light.
  - **MUX and Sensor Addresses:**  
    Change the default I²C addresses if your design uses different values.

Additionally, you have the option to set per-sensor scaling and gain overrides by specifying arrays in the options.

## Installation

Make sure you have Go installed (version 1.24.1 or later recommended). To get the module in your project, run:

```bash
go get github.com/LukasKompatscher/VL6180X
```

## Example Usage

The following example demonstrates how to initialize the sensors, configure options, and read both distance and ambient light values:

```go
package main

import (
    "fmt"
    "log"

    "github.com/LukasKompatscher/VL6180X" // import the package
)

func main() {
    // Configure options:
    // - Use default addresses (PCA9548: 0x70 and VL6180X: 0x29)
    // - Global scaling set to Scaling1 (1x: ~100 mm max range)
    // - Global ALS gain set to ALSGain1 (factory default for normal lighting)
    // You may also specify per-sensor overrides if needed.
    opts := vl6180x.Options{
        Scaling:   vl6180x.Scaling1,
        ALSGain:   vl6180x.ALSGain1,
        // Optional per-sensor overrides:
        // ScalingBySensor: []int{vl6180x.Scaling1, vl6180x.Scaling2},
        // ALSGainBySensor: []vl6180x.ALSGain{vl6180x.ALSGain1, vl6180x.ALSGain40},
    }

    // Initialize the multiplexer with 2 sensors on bus 1.
    device, err := vl6180x.NewVL6180Mux(1, 2, opts)
    if err != nil {
        log.Fatalf("Failed to initialize VL6180X sensors: %v", err)
    }
    defer device.Close()

    // Read distance from sensor 0.
    distance, err := device.ReadDistance(0)
    if err != nil {
        log.Printf("Error reading distance from sensor 0: %v", err)
    } else {
        fmt.Printf("Sensor 0 Distance: %d mm\n", distance)
    }

    // Read ambient light (lux) from sensor 1.
    ambientLux, err := device.ReadAmbient(1)
    if err != nil {
        log.Printf("Error reading ambient light from sensor 1: %v", err)
    } else {
        fmt.Printf("Sensor 1 Ambient Light: %.2f lux\n", ambientLux)
    }

    // Optionally, adjust sensor parameters at runtime.
    if err := device.SetScaling(0, vl6180x.Scaling2); err != nil {
        log.Printf("Failed to change scaling on sensor 0: %v", err)
    }
    if err := device.SetALSGain(1, vl6180x.ALSGain40); err != nil {
        log.Printf("Failed to change ALS gain on sensor 1: %v", err)
    }
}
```

## Explanation of Options

- **MuxAddress**:  
  The I²C address of the PCA9548 multiplexer. If your hardware uses a different address than the default (0x70), specify it here.

- **SensorAddress**:  
  The base I²C address for the VL6180X sensors. By default, this is 0x29. In cases where multiple sensors need to be addressed individually (by first changing their address), this can be set accordingly.

- **Scaling / ScalingBySensor**:  
  Controls the distance measurement range:
  - **Scaling1 (1x):**  
    Standard mode, giving roughly 100 mm maximum range with the best resolution.
  - **Scaling2 (2x):**  
    Doubles the maximum range (~200 mm) at the cost of some measurement precision.
  - **Scaling3 (3x):**  
    Triples the maximum range (~300 mm) with further reduced precision.
  
  You can set a global scaling for all sensors or provide per-sensor values in the `ScalingBySensor` slice.

- **ALSGain / ALSGainBySensor**:  
  Determines the sensitivity of the ambient light sensor. The available gain settings (such as `ALSGain1` or `ALSGain40`) alter the sensor’s response:
  - A lower gain (like `ALSGain1`) is suitable for normal lighting conditions.
  - A higher gain (such as `ALSGain40`) boosts sensitivity in low-light conditions.
  
  Like scaling, you can assign a global ALS gain or provide per-sensor overrides using the `ALSGainBySensor` slice.

## Contributing

Contributions and improvements are welcome. If you encounter issues or have ideas on how to enhance the package (for example, by adding additional error handling or support for more sensor features), please open an issue or submit a pull request on this github repo.
