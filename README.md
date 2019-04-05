# VL53L1X library for mbed

This is Pololu's VL53L1X library adapted for mbed framework. 

## Getting started

### Hardware

A [VL53L1X carrier](https://www.pololu.com/product/3415) can be purchased from Pololu's website.  Before continuing, careful reading of the [product page](https://www.pololu.com/product/3415) as well as the VL53L1X datasheet is recommended.

## Examples

**TARGET:** `ST NUCLEO F401RE`

### Continuous

```cpp
/**
 * main.cpp - continuous example
 */
#include <mbed.h>
#include <VL53L1X.h>

VL53L1X sensor(I2C_SDA,I2C_SCL);
Serial pc(USBTX, USBRX, 115200);
DigitalInOut xshout(D13,PIN_OUTPUT,OpenDrainNoPull,0);

int main() {
  
  xshout = 1;
  wait(1.0);
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    pc.printf("Failed to detect and initialize sensor!");
    while (1);
  }
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);

  while (1)
  {
    pc.printf("%u", sensor.read());
    if (sensor.timeoutOccurred())
    {
      pc.printf(" TIMEOUT\r\n");
    }

    printf("\r\n");
  }
}
```

### Continuous with details

```cpp
/**
 * main.cpp - continuous example with details
 */
#include <mbed.h>
#include <VL53L1X.h>

VL53L1X sensor(I2C_SDA,I2C_SCL);
Serial pc(USBTX, USBRX, 115200);
DigitalInOut xshout(D13,PIN_OUTPUT,OpenDrainNoPull,0);

int main() {

  xshout = 1;
  wait(1.0);
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    pc.printf("Failed to detect and initialize sensor!");
    while (1);
  }
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(200);

  while (1)
  {
    sensor.read();
    pc.printf("range: %u\r\nstatus: %s\r\npeak signal: %.2f\r\nambient: %.2f\r\n",
    sensor.ranging_data.range_mm,
    VL53L1X::rangeStatusToString(sensor.ranging_data.range_status),
    sensor.ranging_data.peak_signal_count_rate_MCPS,
    sensor.ranging_data.ambient_count_rate_MCPS);
    pc.printf("\t --- \r\n");
  }
}
```
## ST's VL53L1X API and this library

Most of the functionality of this library is based on the [VL53L1X API](http://www.st.com/content/st_com/en/products/embedded-software/proximity-sensors-software/stsw-img007.html) provided by ST (STSW-IMG007), and some of the explanatory comments in the code are quoted or paraphrased from the API source code, API user manual (UM2356), and the VL53L1X datasheet. For more explanation about the library code and how it was derived from the API, see the comments in VL53L1X.cpp.

## Library reference

* `RangingData ranging_data`<br>
  This struct contains information about the last ranging measurement. Its members are:
  * `uint16_t range_mm`<br>
    Range reading from the last measurement, in millimeters. (This reading can also be obtained as the return value of `read()`.)
  * `RangeStatus range_status`<br>
    Status of the last measurement; see the definition of the `RangeStatus` enumeration type in VL53L1X.h (or the API user manual and source code) for descriptions of the possible statuses. A status of `VL53L1X::RangeValid` means there were no problems with the measurement.
  * `float peak_signal_count_rate_MCPS`<br>
    Peak signal count rate of the last measurement, in units of mega counts per second.
  * `float ambient_count_rate_MCPS`<br>
    Ambient count rate of the last measurement, in units of mega counts per second.

* `uint8_t last_status`<br>
  The status of the last I&sup2;C write transmission. See the [`Wire.endTransmission()` documentation](http://arduino.cc/en/Reference/WireEndTransmission) for return values.

* `VL53L1X(PinName sda_pin, PinName scl_pin, int frequency)`<br>
  Constructor.

* `void setAddress(uint8_t new_addr)`<br>
  Changes the I&sup2;C slave device address of the VL53L1X to the given value (7-bit).

* `uint8_t getAddress()`<br>
  Returns the current I&sup2;C address.

* `bool init(bool io_2v8 = true)`<br>
  Iniitializes and configures the sensor. If the optional argument `io_2v8` is true (the default if not specified), the sensor is configured for 2V8 mode (2.8 V I/O); if false, the sensor is left in 1V8 mode. The return value is a bool indicating whether the initialization completed successfully.

* `void writeReg(uint16_t reg, uint8_t value)`<br>
  Writes an 8-bit sensor register with the given value.

  Register address constants are defined by the `regAddr` enumeration type in VL53L1X.h.<br>
  Example use: `sensor.writeReg(VL53L1X::SOFT_RESET, 0x00);`

* `void writeReg16Bit(uint16_t reg, uint16_t value)`<br>
  Writes a 16-bit sensor register with the given value.

* `void writeReg32Bit(uint16_t reg, uint32_t value)`<br>
  Writes a 32-bit sensor register with the given value.

* `uint8_t readReg(uint16_t reg)`<br>
  Reads an 8-bit sensor register and returns the value read.

* `uint16_t readReg16Bit(uint16_t reg)`<br>
  Reads a 16-bit sensor register and returns the value read.

* `uint32_t readReg32Bit(uint16_t reg)`<br>
  Reads a 32-bit sensor register and returns the value read.

* `bool setDistanceMode(DistanceMode mode)`<br>
  Sets the distance mode of the sensor (`VL53L1X::Short`, `VL53L1X::Medium`, or `VL53L1X::Long`). Shorter distance modes are less affected by ambient light but have lower maximum ranges. See the datasheet for more information. The return value is a bool indicating whether the requested mode was valid.

* `DistanceMode getDistanceMode()`<br>
  Returns the previously set distance mode.

* `bool setMeasurementTimingBudget(uint32_t budget_us)`<br>
  Sets the measurement timing budget to the given value in microseconds. This is the time allowed for one range measurement; a longer timing budget allows for more accurate measurements. The minimum budget is 20 ms (20000 us) in short distance mode and 33 ms for medium and long distance modes. See the VL53L1X datasheet for more information on range and timing limits. The return value is a bool indicating whether the requested budget was valid.

* `uint32_t getMeasurementTimingBudget()`<br>
  Returns the current measurement timing budget in microseconds.

* `void startContinuous(uint32_t period_ms)`<br>
  Starts continuous ranging measurements. The specified inter-measurement period in milliseconds determines how often the sensor takes a measurement; if it is shorter than the timing budget, the sensor will start a new measurement as soon as the previous one finishes.

* `void stopContinuous()`<br>
  Stops continuous mode.

* `uint16_t read(bool blocking = true)`<br>
  After continuous ranging measurements have been started, calling this function returns a range reading in millimeters and updates the `ranging_data` struct with details about the last measurement. If the optional argument `blocking` is true (the default if not specified), this function will wait until data from a new measurement is available before returning.

  If you do not want this function to block, you can use the `dataReady()` function to check if new data is available before calling `read(false)`. Calling `read(false)` before new data is available will return a reading of 0, and `ranging_data.range_status` will have the value `VL53L1X::None`, indicating that there has been no update.

* `uint16_t readRangeContinuousMillimeters(bool blocking = true)`<br>
  Alias of `read()` for convenience.

* `bool dataReady()`<br>
  Returns a bool indicating whether data from a new measurement is available from the sensor.

* `static const char * rangeStatusToString(RangeStatus status)`<br>
  Converts a `RangeStatus` into a readable string describing that status.
  
  Note that on an AVR, the strings in this function are stored in RAM (dynamic memory), which makes working with them easier but uses up 200+ bytes of RAM (many AVR-based Arduinos only have about 2000 bytes of RAM). You can avoid this memory usage if you do not call this function in your sketch.

* `void setTimeout(uint16_t timeout)`<br>
  Sets a timeout period in milliseconds after which read operations will abort if the sensor is not ready. A value of 0 disables the timeout.

* `uint16_t getTimeout()`<br>
  Returns the current timeout period setting.

* `bool timeoutOccurred()`<br>
  Indicates whether a read timeout has occurred since the last call to `timeoutOccurred()`.
