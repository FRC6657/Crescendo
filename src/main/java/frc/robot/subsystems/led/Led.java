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
  int startRed = 255;
  int startGreen = 0;
  int startBlue = 0;
  /** Creates a new Led. */
  public Led() {

    led = new AddressableLED(1);//PWM port 
    ledBuffer = new AddressableLEDBuffer(15);
    led.setLength(ledBuffer.getLength());

  }

  public void startLED(){
    changeColor(startRed, startGreen, startBlue);
    led.setData(ledBuffer);
    led.start();    

  }

  public void changeColor(int red, int green, int blue){

    for(int i = 0; i<ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i,255,0,0);
    }
  }

  @Override
  public void periodic() {
    led.setData(ledBuffer);
  }
}
