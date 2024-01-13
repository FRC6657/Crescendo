// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.outtake.OuttakeIO.OuttakeIOInputs;

import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final OuttakeIO outtakeIO;
  private final OuttakeIOInputsAutoLogged outtakeInputs = new OuttakeIOInputsAutoLogged();

  private double rpmSetpoint = 0.0;

  public Outtake(OuttakeIO outtakeIO) {
    this.outtakeIO = outtakeIO;
  }

  public Command run() {
    return this.run(
        () -> {
          outtakeIO.run(rpmSetpoint);
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
    outtakeIO.updateInputs(outtakeInputs);
    Logger.processInputs("Outtake", outtakeInputs);
  }
}
