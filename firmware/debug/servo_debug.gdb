set pagination off
set print pretty on

define servo_status
  p servo_debug
end

define servo_probe
  monitor reset halt
  load
  monitor reset init
  continue
end

define servo_live
  break PCA9685Servo::set_pwm
  commands
    silent
    printf "stage=%lu found=%u addr=0x%02x angle=%d pwm=%u i2c_status=%lu i2c_error=0x%lx op=%lu reg=0x%02x writes=%lu reads=%lu\n", servo_debug.stage, servo_debug.pca9685_found, servo_debug.pca9685_address, servo_debug.servo_angle, servo_debug.servo_pwm, servo_debug.i2c_last_status, servo_debug.i2c_last_error, servo_debug.i2c_last_op, servo_debug.i2c_last_register, servo_debug.i2c_write_count, servo_debug.i2c_read_count
    continue
  end
  continue
end

define servo_start_live
  break PCA9685Servo::set_pwm
  commands
    silent
    printf "stage=%lu found=%u addr=0x%02x angle=%d pwm=%u i2c_status=%lu i2c_error=0x%lx op=%lu reg=0x%02x writes=%lu reads=%lu\n", servo_debug.stage, servo_debug.pca9685_found, servo_debug.pca9685_address, servo_debug.servo_angle, servo_debug.servo_pwm, servo_debug.i2c_last_status, servo_debug.i2c_last_error, servo_debug.i2c_last_op, servo_debug.i2c_last_register, servo_debug.i2c_write_count, servo_debug.i2c_read_count
    continue
  end
  monitor reset halt
  load
  monitor reset init
  continue
end

echo Loaded servo debug helpers.\n
echo   servo_probe  - flash/reset/run the board\n
echo   servo_status - print the SWD-readable servo_debug block\n
echo   servo_live   - print a line each time PWM is written\n
echo   servo_start_live - flash/reset/run and print each PWM write\n
