// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//unimportant comment added by Drew for test

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  private double angleSetpoint = 0.0;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  public Command run() {
    return this.run(
        () -> {
          intakeIO.runPivot(angleSetpoint);
        });
  }

  public Command changeAngle(double angle) {
    return this.runOnce(
        () -> {
          angleSetpoint = angle;
        });
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);
  }

  public Pose3d get3DPose() {
    return new Pose3d(
        0.332169,
        0,
        0.210783,
        new Rotation3d(0, Units.degreesToRadians(intakeInputs.pivotMotorPosition), 0));
  }
}
