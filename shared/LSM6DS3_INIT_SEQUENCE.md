# LSM6DS3 Initialization Sequence

## Quick Reference: Register Configuration Order

### 1. Verify Device ID
```c
Read: WHO_AM_I (0x0F) → Expected: 0x69
```

### 2. Software Reset
```c
Write: CTRL3_C (0x12) = 0x01  // SW_RESET bit
Wait: 50ms for reset to complete
```

### 3. Configure Gyroscope FIRST
```c
Write: CTRL2_G (0x11) = 0xAC
// Bits [7:4] = 1010b → ODR = 6.66 kHz
// Bits [3:2] = 11b   → FS_G = 2000 dps (full scale)
// Result: 6.66 kHz, 2000 dps, sensitivity 70 mdps/LSB
```

### 4. Configure Accelerometer SECOND
```c
Write: CTRL1_XL (0x10) = 0xA4
// Bits [7:4] = 1010b → ODR = 6.66 kHz
// Bits [3:2] = 01b   → FS_XL = ±16g (full scale)
// Result: 6.66 kHz, ±16g, sensitivity 0.488 mg/LSB
```

### 5. Configure Common Settings
```c
Write: CTRL3_C (0x12) = 0x44
// Bit [6] = 1 → BDU (Block Data Update) enabled
// Bit [2] = 1 → IF_INC (auto-increment address for multi-byte read)
```

### 6. Configure Gyroscope High-Performance Mode
```c
Write: CTRL7_G (0x16) = 0x00
// CRITICAL: Must be 0x00 for gyroscope to work!
// Bits [7:6] = 00b → High-performance mode enabled
```

### 7. Enable Data Ready Interrupt (Optional)
```c
Write: INT1_CTRL (0x0D) = 0x01
// Bit [0] = 1 → INT1_DRDY_XL (accelerometer data ready on INT1 pin)
```

### 8. Disable Interrupt Latching (Optional)
```c
Write: TAP_CFG (0x58) = 0x00
// Bit [0] = 0 → LIR (interrupt not latched)
```

---

## Reading Data

### Multi-Byte Read with Auto-Increment
```c
// Read 12 bytes starting from OUTX_L_G (0x22)
SPI_Address = 0x22 | 0x80  // 0xA2 (READ bit set, auto-increment via IF_INC)

// Data layout (13 bytes including dummy):
// [0]      → Dummy byte
// [1-2]    → GYRO_X (LSB, MSB)
// [3-4]    → GYRO_Y (LSB, MSB)
// [5-6]    → GYRO_Z (LSB, MSB)
// [7-8]    → ACCEL_X (LSB, MSB)
// [9-10]   → ACCEL_Y (LSB, MSB)
// [11-12]  → ACCEL_Z (LSB, MSB)
```

### Data Conversion
```c
// Gyroscope (16-bit signed)
gyro_x_raw = (int16_t)(data[1] | (data[2] << 8));
gyro_x_dps = gyro_x_raw * 70.0f / 1000.0f;  // to degrees/sec

// Accelerometer (16-bit signed)
accel_x_raw = (int16_t)(data[7] | (data[8] << 8));
accel_x_g = accel_x_raw * 0.488f / 1000.0f;  // to g
```

---

## Critical Notes

1. **Order matters**: Configure CTRL2_G (gyro) BEFORE CTRL1_XL (accel)
2. **CTRL7_G = 0x00 is mandatory** for gyroscope operation
3. **IF_INC bit in CTRL3_C** enables auto-increment (NOT the 0x40 bit in SPI address)
4. **BDU bit** prevents reading mixed old/new data
5. **SPI Mode**: Mode 3 (CPOL=1, CPHA=1)
6. **SPI Speed**: Tested stable at 5.25 MHz (APB2/16)

---

## Alternative Configurations

### Lower Sample Rate (104 Hz)
```c
CTRL2_G = 0x40  // 104 Hz, 245 dps
CTRL1_XL = 0x40 // 104 Hz, ±2g
```

### Different Ranges
```c
// Gyroscope ranges (CTRL2_G bits [3:2]):
// 00b → 245 dps   (sensitivity: 8.75 mdps/LSB)
// 01b → 500 dps   (sensitivity: 17.50 mdps/LSB)
// 10b → 1000 dps  (sensitivity: 35.00 mdps/LSB)
// 11b → 2000 dps  (sensitivity: 70.00 mdps/LSB)

// Accelerometer ranges (CTRL1_XL bits [3:2]):
// 00b → ±2g   (sensitivity: 0.061 mg/LSB)
// 01b → ±16g  (sensitivity: 0.488 mg/LSB)
// 10b → ±4g   (sensitivity: 0.122 mg/LSB)
// 11b → ±8g   (sensitivity: 0.244 mg/LSB)
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Gyroscope reads zero | Set CTRL7_G = 0x00 |
| Accelerometer works, gyro doesn't | Configure CTRL2_G before CTRL1_XL |
| Mixed old/new data | Enable BDU in CTRL3_C |
| Single-byte read only returns one register | Enable IF_INC in CTRL3_C |
| WHO_AM_I fails | Check SPI wiring, mode, and CS polarity |

---

## Example Implementation
See: `shared/Src/LSM6DS3.c` functions:
- `Lsm6ds3_CheckId()` - Step 1
- `Lsm6ds3_Reset()` - Step 2
- `Lsm6ds3_InitHitMode()` - Steps 3-8
- `Lsm6ds3_GetData()` - Reading data
