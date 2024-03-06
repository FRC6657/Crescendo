// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class TOFSensor {

  public DutyCycle sensor;

  public double[] rangeValues = {0, 0, 0, 0};

  /**
   * PWM output wired to the rio DIO
   *
   * @param port
   */
  public TOFSensor(int dioPort) {
    sensor = new DutyCycle(new DigitalInput(dioPort));
  }

  /***
   * Sets the output range of the sensor with the raw input range
   * @param inputMin minimum reading of the sensor
   * @param inputMax maximum reading of the sensor
   * @param outputMin minimum output value
   * @param outputMax maximum output value
   */
  public void setRange(double inputMin, double inputMax, double outputMin, double outputMax) {
    rangeValues[0] = inputMin;
    rangeValues[1] = inputMax;
    rangeValues[2] = outputMin;
    rangeValues[3] = outputMax;
  }

  /***
   * @return The distance from the sensor mapped to the range values specified by setRange
   */
  public double getMappedDistance() {
    return map(
        sensor.getHighTimeNanoseconds(),
        rangeValues[0],
        rangeValues[1],
        rangeValues[2],
        rangeValues[3]);
  }

  /**
   * @return The raw output of the sensor
   */
  public double getRawOutput() {
    return sensor.getHighTimeNanoseconds();
  }

  /**
   * Maps a value from one range to another
   *
   * @param x value to map
   * @param in_min minimum input value
   * @param in_max maximum input value
   * @param out_min minimum output value
   * @param out_max maximum output value
   * @return the mapped value
   */
  private double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}
