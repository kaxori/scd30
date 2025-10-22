// Copyright (C) 2022 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be
// found in the LICENSE file.

import serial
import binary

class Measurements:
    co2         /float
    temperature /float
    humidity    /float

    constructor .co2 .humidity .temperature:

/**
Driver for the Sensirion SCD30 Sensor Module.
*/
class Scd30:
  static I2C_ADDRESS ::= 0x61

  // Available commands.
  static COMMAND_GET_DATA_READY_ ::= #[0x02, 0x02]
  static COMMAND_READ_MEASUREMENT_ ::= #[0x03, 0x00]
  static COMMAND_SET_CONTINUOUS_AND_PRESSURE_ ::= #[0x00, 0x10]
  static COMMAND_SET_FORCED_RECALIBRATION_FACTOR_ ::= #[0x52, 0x04]

  device_/serial.Device
  pressure_/int := ?
  continuous_mode_/bool := true

  /**
  Creates a driver and initializes the ambient pressure calibration
    (in millibar or hectopascal).
  The device is initially in continuous measurement mode, but this
    can be switched with the setter `driver.continuous_mode = false`.
  If continuous measurement was previously disabled there is a delay
    of a few seconds after running this constructor before the device
    is ready to provide measurements.  Reading too early can throw
    write exceptions.
  */
  constructor device/serial.Device --pressure/int=1013:
    device_ = device
    if not 0 < pressure < 0x10000: throw "OUT_OF_RANGE"
    pressure_ = pressure
    set_pressure_

  /**
  Returns whether data is ready.
  */
  is_ready_ -> bool:
    device_.write COMMAND_GET_DATA_READY_
    data := device_.read 3
    check_crc_ data
    value := binary.BIG_ENDIAN.int16 data 0
    return value == 1

  wait_for_ready_ -> none:
    for i := 0; not is_ready_; i++:
      if i == 100: throw "Device is not getting ready."
      sleep --ms=20

  /**
  Gets ambient pressure calibration in millibar or hectopascal.
  The device does not measure pressure.  This getter merely
    retrieves the value that is used to calibrate the other
    measurements.
  */
  pressure -> int:
    return pressure_

  /**
  Sets the ambient pressure in millibar or hectopascal.
  If the device is not in continuous measurement mode then the
    change takes effect when continuous measurement mode is enabled.
  */
  pressure= value/int:
    pressure_ = value
    if continuous_mode_:
      set_pressure_

  continuous_mode -> bool:
    return continuous_mode_

  /**
  Activates or deactivates continuous mode.  When continuous mode is
    activated there is a delay of a few seconds before the device is
    ready to provide measurements.  Reading too early can throw write
    exceptions.
  */
  continuous_mode= value/bool:
    continuous_mode_ = value
    if value:
      set_pressure_
    else:
      device_.write #[0x01, 0x04]

  /**
  Activates the forced recalibration factor.
  */
  set-forced-recalibration-factor concentration/int:
    if concentration < 400 or concentration > 2000:
      throw "wrong concentration value"
    
    concentration_bytes := ByteArray 2
    binary.BIG_ENDIAN.put_int16 concentration_bytes 0 concentration
    command := COMMAND_SET_FORCED_RECALIBRATION_FACTOR_ + concentration_bytes + #[compute_crc8_ concentration_bytes]
    device_.write command


  set_pressure_ -> none:
    // Set continuous mode with the given (or default) air pressure.
    pressure_bytes := ByteArray 2
    binary.BIG_ENDIAN.put_int16 pressure_bytes 0 pressure_
    command := COMMAND_SET_CONTINUOUS_AND_PRESSURE_ + pressure_bytes + #[compute_crc8_ pressure_bytes]
    device_.write command

  /**
  Reads the measurements from the sensor.
  Waits until data is available and returns an object containing the CO2, temperature and humidity.
  */
  read -> Measurements:
    wait_for_ready_

    device_.write COMMAND_READ_MEASUREMENT_
    data := device_.read 18

    check_crc_ data[0..3]
    check_crc_ data[3..6]
    co2_data := data[0..2] + data[3..5]
    co2 := binary.BIG_ENDIAN.float32 co2_data 0

    check_crc_ data[6..9]
    check_crc_ data[9..12]
    temperature_data := data[6..8] + data[9..11]
    temperature := binary.BIG_ENDIAN.float32 temperature_data 0

    check_crc_ data[12..15]
    check_crc_ data[15..18]
    humidity_data := data[12..14] + data[15..17]
    humidity := binary.BIG_ENDIAN.float32 humidity_data 0

    return Measurements co2 humidity temperature

  /**
  Checks checksum and throws if wrong.
  */
  check_crc_ data/ByteArray -> none:
    crc := compute_crc8_ data[..data.size - 1]
    if crc != data.last: throw "Bad CRC"

  /**
  Computes the CRC8 checksum.
  */
  compute_crc8_ data/ByteArray -> int:
    crc := 0xFF
    for x := 0; x < data.size; x++:
      crc ^= data[x]
      for i := 0; i < 8; i++:
        if (crc & 0x80) != 0:
          crc = (crc << 1) ^ 0x31
        else:
          crc <<= 1
        crc &= 0xFF

    return crc
