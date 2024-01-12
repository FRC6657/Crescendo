// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.outake.OutakeIO.OutakeIOInputs;
import org.littletonrobotics.junction.Logger;

public class Outake extends SubsystemBase {
  private final OutakeIO outakeIO;
  private final OutakeIOInputs outakeInputs = new OutakeIOInputs();

  private double rpmSetpoint = 0.0;

  public Outake(OutakeIO outakeIO) {
    this.outakeIO = outakeIO;
  }

  public Command run() {
    return this.run(
        () -> {
          Logger.recordOutput("Outtake/RPM Setpoint", rpmSetpoint);
          outakeIO.run(rpmSetpoint);
        });
  }

  public Command changeRPM(double rpm){
    return this.runOnce(
        () -> {
          rpmSetpoint = rpm;
        });
  }

  @Override
  public void periodic() {
    outakeIO.updateInputs(outakeInputs);
    Logger.recordOutput("Outtake/RPM", outakeInputs.currentRPM);
    Logger.recordOutput("Outtake/Volts", outakeInputs.motorVoltage);
  }
}
