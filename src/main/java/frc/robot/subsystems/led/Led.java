// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;
  int flashTimer = 0; // used to smoothly flash signals to the human player

  public class startingColor {
    public static int red = 255; // currently orange
    public static int green = 100;
    public static int blue = 50;
  } // color that the LEDs are set to after runing the startLED method

  public class defaultColor {
    public static int red = 0; // currently green
    public static int green = 255;
    public static int blue = 0;
  } // color that the LEDs will linger in when not signaling

  public class ampColor {
    public static int red = 128;
    public static int green = 0;
    public static int blue = 255;
  } // color that will be flashed when we signal to human player to amplify the speaker

  /** Creates a new Led. */
  public Led() {

    led = new AddressableLED(1); // PWM port
    ledBuffer = new AddressableLEDBuffer(20);
    led.setLength(ledBuffer.getLength());
  }

  public void startLED() {
    changeColor(startingColor.red, startingColor.green, startingColor.blue);
    led.setData(ledBuffer);
    led.start();
  }

  public void changeColor(int red, int green, int blue) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, red, green, blue);
    }
  }

  public void amplifySignal() {
    if (flashTimer
        > 0) { // start the flash if it is being called by an input, rather than the periodic
      // function which will only call if the flash timer has already started
      flashTimer = 511; // 511 = 256*2-1 this gives two flashes
    } else {
      flashTimer -= 4; // speed a flash will disapate.
      if (flashTimer
          > 0) { // if it is 0 or less than zero, the time has run out and it is time to set it back
        // to the deafult color
        int colorMultiplier =
            (flashTimer % 255)
                / 255; // makes the signal fade out and bounce in on subsequent flashes
        changeColor(
            ampColor.red * colorMultiplier,
            ampColor.green * colorMultiplier,
            ampColor.blue * colorMultiplier);
      } else {
        changeColor(defaultColor.red, defaultColor.green, defaultColor.blue);
        flashTimer =
            0; // makes sure that it is not called by periotic after the flash timer has run its
        // course
      }
    }
  }

  public void cancelSignal() { // turns off a signal and returns to default color
    flashTimer = 0;
    changeColor(defaultColor.red, defaultColor.green, defaultColor.blue);
  }

  @Override
  public void periodic() {
    led.setData(ledBuffer);
    if (flashTimer > 0) {
      amplifySignal();
    }
  }
}
