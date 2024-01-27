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
  /** Creates a new Led. */
  public Led(int red, int green, int blue) {
    led = new AddressableLED(1);//PWM port 1
    ledBuffer = new AddressableLEDBuffer(150);
    changeColor(red, green, blue);
    led.setData(ledBuffer);
    led.start();
  }

  public void changeColor(int red, int green, int blue){
    for(int i = 0; i<ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i,red,green,blue);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    led.setData(ledBuffer);
  }
}