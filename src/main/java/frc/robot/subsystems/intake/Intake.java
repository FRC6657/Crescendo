// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// unimportant comment added by Drew for test

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  public Command changePivotSetpoint(double angleDegrees) {
    return this.runOnce(
        () ->
            intakeIO.changePivotSetpoint(
                MathUtil.clamp(
                    angleDegrees, IntakeConstants.kMinPivotAngle, IntakeConstants.kMaxPivotAngle)));
  }

  public Command changeRollerSpeed(double speed) {
    return this.runOnce(() -> intakeIO.changeRollerSpeed(MathUtil.clamp(speed, -1, 1)));
  }

  public Command waitUntilIntookPiece() {
    return Commands.waitUntil(
        () ->
            (intakeInputs.rollerMotorAcceleration < -100)
                && (intakeInputs.rollerMotorCurrent > 10));
  }

  public boolean noteDetected(){
    return false;
    // return ((intakeInputs.rollerMotorAcceleration < -400) && (intakeInputs.pivotMotorPosition <
    // 0));
  }

  public boolean atSetpoint() {
    return intakeInputs.atSetpoint;
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
        new Rotation3d(
            0,
            -Units.degreesToRadians(
                intakeInputs.pivotMotorPosition - IntakeConstants.kMinPivotAngle),
            0));
  }
}
