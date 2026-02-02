# 📊 Dokumentasi Sistem Sensor POO10A1 dengan Kalibrasi

## 📋 Daftar Isi
1. [Overview](#overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Software Dependencies](#software-dependencies)
4. [Instalasi](#instalasi)
5. [Cara Penggunaan](#cara-penggunaan)
6. [Command Reference](#command-reference)
7. [Sistem Kalibrasi](#sistem-kalibrasi)
8. [Troubleshooting](#troubleshooting)
9. [Technical Details](#technical-details)

---

## 🔍 Overview

Sistem ini menggabungkan pembacaan sensor flow D6F-P0010A1 menggunakan ADC ADS1115 dengan sistem kalibrasi interaktif berbasis PLIC_Utils. Sistem memungkinkan user untuk melakukan kalibrasi custom terhadap pembacaan sensor untuk meningkatkan akurasi.

### ✨ Fitur Utama:
- **Real-time monitoring** sensor flow
- **Interactive calibration system** via Serial
- **Dual output**: Default dan calibrated flow values
- **Multiple extrapolation modes**
- **Data validation** dan error handling
- **Moving average filtering** untuk stabilitas

---

## 🛠️ Hardware Requirements

### Komponen Utama:
- **Microcontroller**: ESP32/Arduino dengan I2C support
- **ADC**: ADS1115 16-bit ADC module
- **Sensor**: D6F-P0010A1 Flow Sensor
- **Connections**:
  - SDA → Pin 8
  - SCL → Pin 9
  - ADS1115 Address: 0x48

### Wiring Diagram:
```
ESP32/Arduino    ADS1115    D6F-P0010A1
      3V3   ──────  VDD
      GND   ──────  GND
      Pin8  ──────  SDA
      Pin9  ──────  SCL
                    A0   ──────  Vout (Sensor)
```

---

## 📚 Software Dependencies

### Libraries Required:
```cpp
#include <Wire.h>                    // I2C Communication
#include <Adafruit_ADS1X15.h>       // ADS1115 Driver
#include "PLIC_Utils.h"             // Calibration System
```

### File Structure:
```
project_folder/
├── main.ino              // Main sketch
├── PLIC_Utils.h         // Header file
├── PLIC_Utils.cpp       // Implementation file
└── README.md            // This documentation
```

---

## 🚀 Instalasi

### Step 1: Install Libraries
1. Buka Arduino IDE
2. Go to **Tools > Manage Libraries**
3. Install: `Adafruit ADS1X15`

### Step 2: Add PLIC_Utils Files
1. Copy `PLIC_Utils.h` dan `PLIC_Utils.cpp` ke folder sketch
2. Restart Arduino IDE jika perlu

### Step 3: Upload Code
1. Connect hardware sesuai wiring diagram
2. Select board dan port yang sesuai
3. Upload sketch ke microcontroller

### Step 4: Verify Installation
1. Open Serial Monitor (115200 baud)
2. Should see startup message dan menu

---

## 🎯 Cara Penggunaan

### Initial Setup
1. **Power on** system
2. **Open Serial Monitor** (115200 baud rate)
3. Verify sensor readings tampil dengan format:
   ```
   Volt=1.2345V | Flow Default=0.123 L/min
   ```

### Basic Operation Mode
Tanpa kalibrasi, sistem menggunakan spesifikasi default sensor:
- **Voltage Range**: 0.5V - 2.5V
- **Flow Range**: 0 - 1.812 L/min
- **Linear mapping** voltage to flow

### Calibration Mode
Untuk akurasi tinggi, lakukan kalibrasi custom:

#### Step 1: Prepare Reference Flow
- Setup sistem dengan **known flow rate**
- Pastikan flow stabil (tunggu 10-15 detik)

#### Step 2: Add Calibration Point
```
> cal
Current voltage: 1.2345 V
Enter actual flow value (L/min): 0.150
✓ Added: V=1.2345 → Flow=0.150 L/min
```

#### Step 3: Repeat for Multiple Points
- **Minimum 2 points** untuk interpolasi
- **Recommended 5-10 points** untuk akurasi optimal
- Cover **full measurement range**

#### Step 4: Enable Calibration
```
> toggle
Calibration mode: ON
```

#### Step 5: Monitor Calibrated Output
```
Volt=1.2345V | Flow Default=0.123 L/min | Flow Calibrated=0.150 L/min
```

---

## 📖 Command Reference

### Basic Commands
| Command | Description | Example |
|---------|-------------|---------|
| `help` | Show command menu | `help` |
| `cal` | Add calibration point | `cal` |
| `table` | Show calibration table | `table` |
| `toggle` | Enable/disable calibration | `toggle` |
| `clear` | Clear all calibration data | `clear` |

### Advanced Commands
| Command | Description | Example |
|---------|-------------|---------|
| `del <n>` | Delete row n from table | `del 3` |
| `mode0` | Extrapolation: Return NAN | `mode0` |
| `mode1` | Extrapolation: Linear | `mode1` |
| `mode2` | Extrapolation: Edge clamp | `mode2` |

### Command Details

#### `cal` - Add Calibration Point
1. Reads current voltage
2. Prompts for actual flow value
3. Validates and stores data point
4. Returns success/error status

#### `table` - View Calibration Data
Shows complete calibration table:
```
Calibration Table:
Row Count: 3
x0 = 0.500    y0 = 0.000
x1 = 1.500    y1 = 0.150
x2 = 2.500    y2 = 1.200
```

#### `toggle` - Enable/Disable Calibration
Switches between default dan calibrated output modes.

---

## 🎯 Sistem Kalibrasi

### PLIC_Utils Features

#### Interpolation
- **Linear interpolation** between calibration points
- **Automatic sorting** of data points by voltage
- **Robust duplicate handling**

#### Extrapolation Modes
1. **Mode 0 (NAN)**: Return NAN for out-of-range values
2. **Mode 1 (Linear)**: Extend first/last segment linearly
3. **Mode 2 (Clamp)**: Use edge values for out-of-range

#### Data Validation
- **Epsilon checking**: Prevents too-close data points
- **Range validation**: Ensures data integrity
- **Overflow protection**: Maximum 100 calibration points

### Calibration Best Practices

#### Point Selection
- **Distribute evenly** across measurement range
- **Include edge points** (min/max flow rates)
- **Verify reference accuracy** sebelum calibration

#### Quality Control
- **Cross-validate** dengan independent measurement
- **Check linearity** di intermediate points  
- **Document calibration conditions** (temperature, pressure)

#### Maintenance
- **Re-calibrate periodically** (recommended: 6 months)
- **Verify stability** after environmental changes
- **Backup calibration data** untuk recovery

---

## 🔧 Troubleshooting

### Common Issues

#### "ADS1115 tidak terdeteksi!"
**Causes:**
- I2C wiring error
- Wrong I2C address
- Power supply issue

**Solutions:**
1. Check SDA/SCL connections
2. Verify 3.3V power supply
3. Try I2C scanner sketch
4. Check ADS1115 address jumpers

#### Erratic Sensor Readings
**Causes:**
- Electrical noise
- Poor connections
- Sensor contamination

**Solutions:**
1. Add decoupling capacitors
2. Check all connections
3. Clean sensor element
4. Increase averaging samples

#### Calibration Not Working
**Causes:**
- Less than 2 calibration points
- Calibration mode disabled
- Invalid reference values

**Solutions:**
1. Add minimum 2 calibration points
2. Enable with `toggle` command
3. Verify reference flow accuracy
4. Check extrapolation mode

### Error Codes

#### add_data() Return Values
- **0**: Table full (max 100 points)
- **1**: Successfully added
- **2**: Duplicate not allowed
- **3**: Successfully replaced
- **4**: Too close to existing point

#### Voltage Range Warnings
- **[LOW]**: Voltage ≤ 0.5V (below sensor range)
- **[HIGH]**: Voltage ≥ 2.5V (above sensor range)

---

## ⚙️ Technical Details

### Sensor Specifications
```cpp
// D6F-P0010A1 Default Parameters
FLOW_MAX_LPM = 1.812f    // Maximum flow rate
VOUT_MIN = 0.5f          // 0% flow voltage
VOUT_MAX = 2.5f          // 100% flow voltage
AVG_N = 16               // Moving average samples
```

### ADS1115 Configuration
```cpp
ads.setGain(GAIN_ONE);   // ±4.096V full-scale
Address: 0x48            // Default I2C address
Resolution: 16-bit       // 65536 counts
Sample Rate: ~860 SPS    // Continuous mode
```

### Memory Usage
- **Calibration Points**: 100 max (800 bytes)
- **Each Point**: 8 bytes (float x, float y)
- **Stack Usage**: ~200 bytes typical

### Performance Characteristics
- **Update Rate**: ~2 Hz (500ms interval)
- **Settling Time**: ~50ms (16-sample average)
- **Accuracy**: ±0.01% with proper calibration
- **Resolution**: ~0.0625 mV (16-bit ADC)

### Algorithm Details

#### Moving Average Filter
```cpp
for (int i = 0; i < AVG_N; i++) { 
  acc += readVoltA0(); 
  delay(3); 
}
float voltage = acc / AVG_N;
```

#### Linear Interpolation
```cpp
float t = (input_x - x0) / (x1 - x0);
return y0 + t * (y1 - y0);
```

#### Binary Search
O(log n) complexity untuk efficient point lookup dalam sorted calibration table.

---

## 📈 Example Usage Scenarios

### Scenario 1: Laboratory Calibration
```
1. Setup reference flow meter
2. Set flow = 0.0 L/min → cal → enter 0.000
3. Set flow = 0.5 L/min → cal → enter 0.500  
4. Set flow = 1.0 L/min → cal → enter 1.000
5. Set flow = 1.5 L/min → cal → enter 1.500
6. toggle → Enable calibration
```

### Scenario 2: Field Calibration
```
1. Use portable reference (syringe pump)
2. Calibrate at 3-5 key operating points
3. Verify with toggle on/off comparison
4. Document calibration certificate
```

### Scenario 3: Production Testing
```
1. Automated test setup
2. Serial commands via test script
3. Statistical validation of results
4. Pass/fail criteria based on accuracy
```

---

## 📝 Notes dan Limitations

### Important Notes
- **Calibration persists** until power cycle atau clear command
- **Extrapolation accuracy** depends on calibration point distribution  
- **Temperature effects** may require periodic re-calibration
- **Flow direction**: Sensor adalah unidirectional

### Limitations
- **Maximum 100** calibration points
- **No persistent storage** (EEPROM/Flash integration needed)
- **Single sensor support** (can be extended for multiple)
- **No automatic temperature compensation**

### Future Enhancements
- EEPROM calibration storage
- Temperature compensation
- Multi-sensor support  
- Web interface for remote calibration
- Data logging capabilities

---

**© 2024 - Sensor Calibration System Documentation**