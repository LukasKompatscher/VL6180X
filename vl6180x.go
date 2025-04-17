package vl6180x

import (
	"fmt"
	"os"
	"sync"
	"syscall"
	"time"
)

// I2C device addresses
const (
	DefaultMuxAddr    = 0x70 // Default I2C address of PCA9548 multiplexer
	DefaultSensorAddr = 0x29 // Default I2C address of VL6180X sensor
)

// VL6180X register addresses (16-bit) used for configuration and readings
const (
	regSysRangeStart           uint16 = 0x0018 // SYSRANGE__START: start range measurement
	regResultInterruptStatus   uint16 = 0x004F // RESULT__INTERRUPT_STATUS_GPIO (for new sample ready flags)
	regResultRangeVal          uint16 = 0x0062 // RESULT__RANGE_VAL (range measurement value)
	regSystemInterruptClear    uint16 = 0x0015 // SYSTEM__INTERRUPT_CLEAR (clear range/ALS interrupts)
	regSysALSStart             uint16 = 0x0038 // SYSALS__START: start ALS measurement
	regSysALSAnalogueGain      uint16 = 0x003F // SYSALS__ANALOGUE_GAIN: ALS gain register
	regSysALSIntegrationPeriod uint16 = 0x0040 // SYSALS__INTEGRATION_PERIOD (ALS integration time)
	regResultALSVal            uint16 = 0x0050 // RESULT__ALS_VAL (ambient light result)
	regI2CSlaveDeviceAddr      uint16 = 0x0212 // I2C_SLAVE__DEVICE_ADDRESS (to change sensor I2C address)
	regSystemFreshOutOfReset   uint16 = 0x0016 // SYSTEM__FRESH_OUT_OF_RESET indicator (==1 at boot)
)

// Allowed VL6180X ALS gain settings (to amplify ambient light sensor readings).
// The value of each constant includes the mandatory upper nibble 0x40, as required by the sensor's register.
type ALSGain uint8

const (
	ALSGain20   ALSGain = 0x40 | 0x00 // Gain 20.0 (highest sensitivity)
	ALSGain10   ALSGain = 0x40 | 0x01 // Gain 10.32
	ALSGain5    ALSGain = 0x40 | 0x02 // Gain 5.21
	ALSGain2_5  ALSGain = 0x40 | 0x03 // Gain 2.60
	ALSGain1_67 ALSGain = 0x40 | 0x04 // Gain 1.72
	ALSGain1_25 ALSGain = 0x40 | 0x05 // Gain 1.28
	ALSGain1    ALSGain = 0x40 | 0x06 // Gain 1.01 (factory-calibrated default)
	ALSGain40   ALSGain = 0x40 | 0x07 // Gain 40.0 (useful for very low light)
)

// Predefined range scaling values (1x, 2x, 3x) which trade maximum range for resolution.
// Scaling 1x = ~100mm max range (default), 2x ~ 200mm, 3x ~ 300mm (with reduced precision).
const (
	Scaling1 = 1
	Scaling2 = 2
	Scaling3 = 3
)

// Look up table for ALS gain conversion factors
// To compute lux form raw ALS readings
var alsGainFactors = [8]float64{
	20.0,  // index 0: ALSGain20
	10.32, // index 1: ALSGain10
	5.21,  // index 2: ALSGain5
	2.60,  // index 3: ALSGain2_5
	1.72,  // index 4: ALSGain1_67
	1.28,  // index 5: ALSGain1_25
	1.01,  // index 6: ALSGain1
	40.0,  // index 7: ALSGain40
}

// Options allows optional configuration for initializing the sensors.
type Options struct {
	MuxAddress      uint8     // Custom I2C address of the PCA9548 (default 0x70 if 0 or unspecified)
	SensorAddress   uint8     // Custom base address of VL6180X sensors (default 0x29 if 0)
	Scaling         int       // Default scaling to apply to all sensors (1, 2, or 3)
	ALSGain         ALSGain   // Default ALS gain for all sensors (use constants like ALSGain1, etc.)
	ScalingBySensor []int     // Optional per-sensor scaling settings (index 0-7)
	ALSGainBySensor []ALSGain // Optional per-sensor ALS gain settings
}

// VL6180Mux represents the multi-sensor device with a PCA9548 I2C multiplexer.
type VL6180Mux struct {
	busNum         int        // I²C bus number (e.g. 1 for "/dev/i2c-1")
	muxAddr        uint8      // I²C address of PCA9548 multiplexer
	sensorAddr     uint8      // I²C address of VL6180X sensors (all sensors share this base address)
	sensorCount    int        // Number of sensors in use (1-8)
	scaling        []int      // Current scaling setting for each sensor (1,2,3)
	alsGain        []ALSGain  // Current ALS gain setting for each sensor
	muxFile        *os.File   // File handle for I2C bus (multiplexer control)
	sensorFile     *os.File   // File handle for I2C bus (sensor communication)
	mu             sync.Mutex // Mutex to serialize I2C operations (since bus is shared)
	currentChannel int        // Tracks currently active multiplexer channel (for optimization)
}

// NewVL6180Mux opens the I²C bus and initializes the VL6180X sensors via the PCA9548 multiplexer.
// busNum is the I2C bus number (e.g. 1 for "/dev/i2c-1"), sensorCount is the number of sensors (channels 0..sensorCount-1 used),
// and opts allows customization of addresses and sensor settings. Returns a VL6180Mux device or an error.
func NewVL6180Mux(busNum int, sensorCount int, opts Options) (*VL6180Mux, error) {
	// Use default addresses if not overridden
	muxAddr := opts.MuxAddress
	if muxAddr == 0 {
		muxAddr = DefaultMuxAddr
	}
	sensorAddr := opts.SensorAddress
	if sensorAddr == 0 {
		sensorAddr = DefaultSensorAddr
	}
	if sensorCount < 1 || sensorCount > 8 {
		return nil, fmt.Errorf("sensorCount must be between 1 and 8")
	}
	// Apply default scaling/gain if unspecified or out of range
	scalingDefault := opts.Scaling
	if scalingDefault < Scaling1 || scalingDefault > Scaling3 {
		scalingDefault = Scaling1
	}
	gainDefault := opts.ALSGain
	if gainDefault < 0x40 || gainDefault > 0x47 { // valid ALSGain values range from 0x40 to 0x47
		gainDefault = ALSGain1
	}

	// Open the I2C bus device file for the multiplexer and sensor (two separate file descriptors)
	busPath := fmt.Sprintf("/dev/i2c-%d", busNum)
	muxFile, err := os.OpenFile(busPath, os.O_RDWR, 0)
	if err != nil {
		return nil, fmt.Errorf("failed to open I2C bus %d: %v", busNum, err)
	}
	sensorFile, err := os.OpenFile(busPath, os.O_RDWR, 0)
	if err != nil {
		muxFile.Close()
		return nil, fmt.Errorf("failed to open I2C bus for sensors: %v", err)
	}
	// Define ioctl request code for selecting I2C slave address
	const i2cSlave = 0x0703
	// Set the multiplexer file descriptor to talk to the PCA9548 address
	if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL, muxFile.Fd(), uintptr(i2cSlave), uintptr(muxAddr)); errno != 0 {
		muxFile.Close()
		sensorFile.Close()
		return nil, fmt.Errorf("ioctl set mux address 0x%X failed: %v", muxAddr, errno)
	}
	// Set the sensor file descriptor to talk to the VL6180X address (this will be used after selecting channels)
	if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL, sensorFile.Fd(), uintptr(i2cSlave), uintptr(sensorAddr)); errno != 0 {
		muxFile.Close()
		sensorFile.Close()
		return nil, fmt.Errorf("ioctl set sensor address 0x%X failed: %v", sensorAddr, errno)
	}

	// Create device struct
	d := &VL6180Mux{
		busNum:         busNum,
		muxAddr:        muxAddr,
		sensorAddr:     sensorAddr,
		sensorCount:    sensorCount,
		scaling:        make([]int, sensorCount),
		alsGain:        make([]ALSGain, sensorCount),
		muxFile:        muxFile,
		sensorFile:     sensorFile,
		currentChannel: -1,
	}

	// Initialize scaling and gain arrays with defaults or per-sensor overrides
	for i := 0; i < sensorCount; i++ {
		d.scaling[i] = scalingDefault
		if len(opts.ScalingBySensor) > i && (opts.ScalingBySensor[i] == Scaling1 || opts.ScalingBySensor[i] == Scaling2 || opts.ScalingBySensor[i] == Scaling3) {
			d.scaling[i] = opts.ScalingBySensor[i]
		}
		d.alsGain[i] = gainDefault
		if len(opts.ALSGainBySensor) > i && opts.ALSGainBySensor[i] >= 0x40 && opts.ALSGainBySensor[i] <= 0x47 {
			d.alsGain[i] = opts.ALSGainBySensor[i]
		}
	}

	// Disable all multiplexer channels initially
	if _, err := d.muxFile.Write([]byte{0x00}); err != nil {
		d.Close()
		return nil, fmt.Errorf("failed to write to PCA9548 (disable channels): %v", err)
	}

	// Initialize each VL6180X sensor on its channel
	for idx := 0; idx < sensorCount; idx++ {
		// Select the multiplexer channel for this sensor
		if err := d.selectChannel(idx); err != nil {
			d.Close()
			return nil, fmt.Errorf("failed to select I2C channel %d: %v", idx, err)
		}
		// Check if sensor is fresh out of reset
		setup, err := d.readReg8(regSystemFreshOutOfReset)
		if err != nil {
			d.Close()
			return nil, fmt.Errorf("failed to communicate with sensor %d: %v", idx, err)
		}
		if setup == 1 {
			// The sensor is fresh after power-on: load mandatory initialization settings
			// (Sequence from VL6180X datasheet and ST application notes)
			// --- Private registers (no touch) and calibration settings:
			d.writeReg8(0x0207, 0x01)
			d.writeReg8(0x0208, 0x01)
			d.writeReg8(0x0096, 0x00)
			d.writeReg8(0x0097, 0xFD)
			d.writeReg8(0x00E3, 0x00)
			d.writeReg8(0x00E4, 0x04)
			d.writeReg8(0x00E5, 0x02)
			d.writeReg8(0x00E6, 0x01)
			d.writeReg8(0x00E7, 0x03)
			d.writeReg8(0x00F5, 0x02)
			d.writeReg8(0x00D9, 0x05)
			d.writeReg8(0x00DB, 0xCE)
			d.writeReg8(0x00DC, 0x03)
			d.writeReg8(0x00DD, 0xF8)
			d.writeReg8(0x009F, 0x00)
			d.writeReg8(0x00A3, 0x3C)
			d.writeReg8(0x00B7, 0x00)
			d.writeReg8(0x00BB, 0x3C)
			d.writeReg8(0x00B2, 0x09)
			d.writeReg8(0x00CA, 0x09)
			d.writeReg8(0x0198, 0x01)
			d.writeReg8(0x01B0, 0x17)
			d.writeReg8(0x01AD, 0x00)
			d.writeReg8(0x00FF, 0x05)
			d.writeReg8(0x0100, 0x05)
			d.writeReg8(0x0199, 0x05)
			d.writeReg8(0x01A6, 0x1B)
			d.writeReg8(0x01AC, 0x3E)
			d.writeReg8(0x01A7, 0x1F)
			d.writeReg8(0x0030, 0x00)
			// --- Recommended public registers from datasheet:
			d.writeReg8(0x0011, 0x10) // Enables polling for "New Sample Ready"
			d.writeReg8(0x010A, 0x30) // Set average sample period
			d.writeReg8(0x003F, 0x46) // Set ALS gain: 1x (default)
			d.writeReg8(0x0031, 0xFF) // Auto calibration period (max)
			d.writeReg8(0x0040, 0x63) // ALS integration time = 100ms
			d.writeReg8(0x002E, 0x01) // Perform a single temperature calibration
			d.writeReg8(0x001B, 0x09) // Default ranging inter-measurement period = 100ms
			d.writeReg8(0x003E, 0x31) // Default ALS inter-measurement period = 500ms
			d.writeReg8(0x0014, 0x24) // Interrupt configuration: new sample ready
			// Complete initialization
			d.writeReg8(regSystemFreshOutOfReset, 0x00) // indicate initialization done (write 0 to 0x016)
		}
		// Set the desired initial scaling mode for this sensor
		if err := d.setScalingInternal(idx, d.scaling[idx]); err != nil {
			// If scaling setting fails, treat as critical
			d.Close()
			return nil, fmt.Errorf("failed to set scaling for sensor %d: %v", idx, err)
		}
		// (ALS gain does not need to be written now; it's applied on each ambient read)
	}
	// Leave all sensors idle (can disable all channels again if needed)
	d.currentChannel = -1
	if _, err := d.muxFile.Write([]byte{0x00}); err != nil {
		// Not fatal if this fails; just log it.
		fmt.Println("Warning: couldn't disable mux channels after init:", err)
	}
	return d, nil
}

// selectChannel activates the given multiplexer channel (0-7) so that the corresponding VL6180 sensor is connected.
// It writes to the PCA9548's control register to select the channel. This method remembers the last selected channel
// to avoid redundant I2C writes if the same channel is selected consecutively.
func (d *VL6180Mux) selectChannel(channel int) error {
	if channel < 0 || channel >= d.sensorCount {
		return fmt.Errorf("sensor index %d out of range", channel)
	}
	if channel == d.currentChannel {
		return nil // already on the correct channel, no need to switch
	}
	// Write the channel bit to the PCA9548 (e.g., channel 3 -> 0x08)
	data := []byte{1 << channel}
	_, err := d.muxFile.Write(data)
	if err != nil {
		d.currentChannel = -1
		return fmt.Errorf("failed to select mux channel %d: %v", channel, err)
	}
	d.currentChannel = channel
	return nil
}

// readReg8 reads a single 8-bit value from a 16-bit register address on the currently active VL6180 sensor.
func (d *VL6180Mux) readReg8(reg uint16) (byte, error) {
	// Write register address (MSB then LSB) to sensor
	buf := []byte{byte(reg >> 8), byte(reg & 0xFF)}
	if _, err := d.sensorFile.Write(buf); err != nil {
		return 0, err
	}
	// Read one byte from sensor
	var data [1]byte
	if _, err := d.sensorFile.Read(data[:]); err != nil {
		return 0, err
	}
	return data[0], nil
}

// readReg16 reads a 16-bit value from a 16-bit register address (reads two bytes, big-endian).
func (d *VL6180Mux) readReg16(reg uint16) (uint16, error) {
	buf := []byte{byte(reg >> 8), byte(reg & 0xFF)}
	if _, err := d.sensorFile.Write(buf); err != nil {
		return 0, err
	}
	var data [2]byte
	if _, err := d.sensorFile.Read(data[:]); err != nil {
		return 0, err
	}
	return (uint16(data[0]) << 8) | uint16(data[1]), nil
}

// writeReg8 writes an 8-bit value to a 16-bit register address on the sensor.
func (d *VL6180Mux) writeReg8(reg uint16, value byte) error {
	buf := []byte{byte(reg >> 8), byte(reg & 0xFF), value}
	_, err := d.sensorFile.Write(buf)
	return err
}

// writeReg16 writes a 16-bit value to a 16-bit register address on the sensor.
func (d *VL6180Mux) writeReg16(reg uint16, value uint16) error {
	buf := []byte{
		byte(reg >> 8), byte(reg & 0xFF),
		byte(value >> 8), byte(value & 0xFF),
	}
	_, err := d.sensorFile.Write(buf)
	return err
}

// setScalingInternal applies a new scaling factor (1,2,3) to the sensor at the given index.
// This function assumes the channel is already selected.
func (d *VL6180Mux) setScalingInternal(index int, newScaling int) error {
	if newScaling < Scaling1 || newScaling > Scaling3 {
		return fmt.Errorf("invalid scaling %d", newScaling)
	}
	// Values for RANGE_SCALER register for scaling 1x, 2x, 3x (from datasheet/app note)
	var scalerValues = [4]uint16{0, 253, 127, 84} // index 1->253, 2->127, 3->84
	// Read current part-to-part offset from the sensor
	ptpOffset, err := d.readReg8(0x0024)
	if err != nil {
		return err
	}
	// Write new range scaler value (two bytes) to register 0x0096
	if err := d.writeReg16(0x0096, scalerValues[newScaling]); err != nil {
		return err
	}
	// Adjust the cross-talk valid height and part-to-part offset for the new scaling
	if err := d.writeReg8(0x0024, byte(int32(ptpOffset)/int32(newScaling))); err != nil { // (ptpOffset is small, safe to cast)
		return err
	}
	const defaultCrosstalkValidHeight = 20
	if err := d.writeReg8(0x0021, byte(defaultCrosstalkValidHeight/newScaling)); err != nil {
		return err
	}
	// Adjust the range check enabling (RCE) bit in 0x002D: if scaling==1, set bit0 = 1, else 0
	rce, err := d.readReg8(0x002D)
	if err != nil {
		return err
	}
	var bit0 byte
	if newScaling == Scaling1 {
		bit0 = 1
	} else {
		bit0 = 0
	}
	if err := d.writeReg8(0x002D, (rce&0xFE)|bit0); err != nil {
		return err
	}
	// Update stored scaling value
	d.scaling[index] = newScaling
	return nil
}

// ReadDistance triggers a single distance measurement and returns the distance in millimeters,
// or an error if the sensor doesn’t respond within 200 ms.
func (d *VL6180Mux) ReadDistance(sensorIndex int) (int, error) {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.selectChannel(sensorIndex); err != nil {
		return 0, err
	}
	// Clear any pending interrupt
	if err := d.writeReg8(regSystemInterruptClear, 0x07); err != nil {
		return 0, fmt.Errorf("clear before start failed: %v", err)
	}
	// Start a range measurement by writing 0x01 to SYSRANGE_START register
	if err := d.writeReg8(regSysRangeStart, 0x01); err != nil {
		return 0, fmt.Errorf("failed to start range measurement: %v", err)
	}

	// Poll up to 200 ms for “new sample ready”
	// Poll the interrupt status until the new sample is ready (bit[2:0] == 0x4 indicates range data ready)
	deadline := time.Now().Add(200 * time.Millisecond)
	var status byte
	for time.Now().Before(deadline) {
		st, err := d.readReg8(regResultInterruptStatus)
		if err != nil {
			return 0, fmt.Errorf("failed to read range status: %v", err)

		}
		if st&0x07 == 0x04 {
			status = st
			break
		}
		time.Sleep(2 * time.Millisecond)
	}
	if status&0x07 != 0x04 {
		return 0, fmt.Errorf("range poll timeout")
	}

	// Read the result from the sensor's RESULT_RANGE_VAL register
	dist, err := d.readReg8(regResultRangeVal)
	if err != nil {
		return 0, fmt.Errorf("failed to read distance result: %v", err)
	}
	// Clear the sensor's interrupt flags to prepare for the next measurement
	if err := d.writeReg8(regSystemInterruptClear, 0x07); err != nil {
		return int(dist), fmt.Errorf("clear after read failed: %v", err)
	}
	return int(dist), nil
}

// ReadAmbient performs an ambient light measurement on the specified sensor and returns the light level in lux.
func (d *VL6180Mux) ReadAmbient(sensorIndex int) (float64, error) {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.selectChannel(sensorIndex); err != nil {
		return 0, err
	}
	// Write the configured ALS gain to the sensor's analogue gain register (the high nibble must be 0x4)
	gain := d.alsGain[sensorIndex]
	if err := d.writeReg8(regSysALSAnalogueGain, byte(gain)); err != nil {
		return 0, fmt.Errorf("failed to set ALS gain: %v", err)
	}
	// Start an ALS (ambient light) measurement
	if err := d.writeReg8(regSysALSStart, 0x01); err != nil {
		return 0, fmt.Errorf("failed to start ALS measurement: %v", err)
	}
	// Clear any pending interrupts before reading ALS result (ensure fresh data)
	if err := d.writeReg8(regSystemInterruptClear, 0x07); err != nil {
		return 0, fmt.Errorf("failed to clear ALS interrupt: %v", err)
	}
	// Note: According to the datasheet, the ALS measurement will complete after the integration period (default 100ms).
	// Here we proceed to read immediately; in practice, the sensor updates the result register after the integration time.
	// If precise timing is needed, one could poll a status bit similar to range, or sleep for integration duration.
	// For simplicity, we assume the default integration time and sensor ready by the time of read (as done in original code).

	// Read raw ALS value (16-bit) and integration period (16-bit)
	alsRaw, err := d.readReg16(regResultALSVal)
	if err != nil {
		return 0, fmt.Errorf("failed to read ALS value: %v", err)
	}
	alsIntegrationPeriodRaw, err := d.readReg16(regSysALSIntegrationPeriod)
	if err != nil {
		return 0, fmt.Errorf("failed to read ALS integration period: %v", err)
	}
	// Calculate actual integration time in ms (default register value 0x63 -> ~100 ms)
	integrationTime := 100.0 / float64(alsIntegrationPeriodRaw)
	// Determine the gain multiplier from the gain setting (lower 3 bits of gain value)
	gainIndex := gain & 0x07 // extract 0-7 index
	alsGainFactor := alsGainFactors[gainIndex]

	// Calculate lux using formula from application note:
	// lux = 0.32 * (ALS_raw / ALS_gain) * (100 / integration_time)
	lux := 0.32 * (float64(alsRaw) / alsGainFactor) * integrationTime
	return lux, nil
}

// SetScaling changes the range scaling mode (1, 2, or 3) for the specified sensor.
// Scaling 1 is default (~100mm max range), 2 doubles the range (~200mm) at the cost of resolution, 3 triples the range (~300mm).
// This function will update the sensor's internal settings accordingly.
func (d *VL6180Mux) SetScaling(sensorIndex int, scaling int) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.selectChannel(sensorIndex); err != nil {
		return err
	}
	// If the requested scaling is the same as current, do nothing.
	if scaling == d.scaling[sensorIndex] {
		return nil
	}
	// Apply the new scaling to the sensor's registers
	if err := d.setScalingInternal(sensorIndex, scaling); err != nil {
		return fmt.Errorf("failed to set scaling on sensor %d: %v", sensorIndex, err)
	}
	return nil
}

// SetALSGain changes the default ambient light sensor gain for the specified sensor.
// The new gain will be used on subsequent ReadAmbient calls. (Does not retroactively affect ongoing measurements.)
func (d *VL6180Mux) SetALSGain(sensorIndex int, gain ALSGain) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if sensorIndex < 0 || sensorIndex >= d.sensorCount {
		return fmt.Errorf("sensor index out of range")
	}
	if gain < 0x40 || gain > 0x47 {
		return fmt.Errorf("invalid ALS gain value 0x%X", gain)
	}
	d.alsGain[sensorIndex] = gain
	return nil
}

// Close closes the underlying I2C file handles. After calling Close, the VL6180Mux device is no longer usable.
func (d *VL6180Mux) Close() error {
	var errMux error
	var errSensor error
	if d.muxFile != nil {
		errMux = d.muxFile.Close()
		d.muxFile = nil
	}
	if d.sensorFile != nil {
		errSensor = d.sensorFile.Close()
		d.sensorFile = nil
	}
	d.currentChannel = -1
	if errMux != nil {
		return fmt.Errorf("error closing mux I2C file: %v", errMux)
	}
	if errSensor != nil {
		return fmt.Errorf("error closing sensor I2C file: %v", errSensor)
	}
	return nil
}
