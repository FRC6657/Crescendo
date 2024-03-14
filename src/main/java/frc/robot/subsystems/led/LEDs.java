// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.Color;

public class LEDs extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;
  int flashTimer = 0; // used to smoothly flash signals to the human player

  /** Creates a new Led. */
  public LEDs() {
    led = new AddressableLED(0); // PWM port
    ledBuffer = new AddressableLEDBuffer(56);
    led.setLength(ledBuffer.getLength());
    led.start();
    changeColor(LEDConstants.kDisabledColor);
  }

  public void changeColor(Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, color.red, color.green, color.blue);
    }
    led.setData(ledBuffer);
  }

  public Command changeColorCommand(Color color) {
    return Commands.runOnce(() -> changeColor(color));
  }

  public void amplifySignal() {
    if (flashTimer
        <= 0) { // the periodic method calls amplifySignal whenever flashTimer is greater than 0
      // this will cause the flash timer to be reset only if it is called from outside periodic
      flashTimer = 512; // 511 = 256*2 this gives two flashes
    } else {
      flashTimer -= 4; // speed a flash will disapate.
      if (flashTimer
          > 0) { // if it is 0 or less than zero, the time has run out and it is time to set it back
        // to the deafult color
        double colorMultiplier =
            (flashTimer % 256)
                / 255.0; // makes the signal fade out and bounce in on subsequent flashes
        changeColor(
            new Color(
                (int) Math.round(LEDConstants.kEnabledColor.red * colorMultiplier),
                (int) Math.round(LEDConstants.kEnabledColor.green * colorMultiplier),
                (int) Math.round(LEDConstants.kEnabledColor.blue * colorMultiplier)));
      } else {
        changeColor(
            new Color(
                LEDConstants.kEnabledColor.red,
                LEDConstants.kEnabledColor.green,
                LEDConstants.kEnabledColor.blue));
        flashTimer =
            0; // makes sure that it is not called by periotic after the flash timer has run its
        // course
      }
    }
  }

  public void cancelSignal() { // turns off a signal and returns to default color
    flashTimer = 0;
    changeColor(
        new Color(
            LEDConstants.kEnabledColor.red,
            LEDConstants.kEnabledColor.green,
            LEDConstants.kEnabledColor.blue));
  }

  @Override
  public void periodic() {
    led.setData(ledBuffer);
    if (flashTimer > 0) {
      amplifySignal();
    }
  }
}
