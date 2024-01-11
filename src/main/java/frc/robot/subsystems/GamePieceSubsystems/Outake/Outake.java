// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GamePieceSubsystems.Outake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants.GamePiece;
import frc.robot.Constants.IntakeConstants.IntakeDirection;
import frc.robot.subsystems.GamePieceSubsystems.Outake.OutakeIO.OutakeIOInputs;



public class Outake extends SubsystemBase {
  private final OutakeIO outakeIO;
  private final OutakeIOInputs outakeInputs = new OutakeIOInputs();
  
  public Outake(OutakeIO outakeIO) {
    this.outakeIO = outakeIO;

    
  }

  
  public Command run(double rpm) {
    return this.run(
        () -> {
          Logger.recordOutput("Outtake/RPM Setpoint", rpm);
          outakeIO.run(rpm);
        });
  }



  @Override
  public void periodic() {
    outakeIO.updateInputs(outakeInputs);
    Logger.recordOutput("Outtake/RPM", outakeInputs.currentRPM);
    Logger.recordOutput("Outtake/Volts", outakeInputs.motorVoltage);
  }
}

