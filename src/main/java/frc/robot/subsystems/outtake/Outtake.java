// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final OuttakeIO outtakeIO;
  private final OuttakeIOInputsAutoLogged outtakeInputs = new OuttakeIOInputsAutoLogged();

  public Outtake(OuttakeIO outtakeIO) {
    this.outtakeIO = outtakeIO;
  }

  public Command run() {
    return this.run(() -> {});
  }

  public Command changeRPM(double rpm) {
    return this.runOnce(
        () -> {
          outtakeIO.changeFlywheel(MathUtil.clamp(rpm, OuttakeConstants.kMinRpm, OuttakeConstants.kMaxRpm));
        });
  }

  public Command changeAngle(double angle) {
    return this.runOnce(
        () -> {
          outtakeIO.changePivot(MathUtil.clamp(angle, OuttakeConstants.kMinAngle, OuttakeConstants.kMaxAngle));
        });
  }

  @Override
  public void periodic() {
    outtakeIO.updateInputs(outtakeInputs);
    Logger.processInputs("Outtake", outtakeInputs);
  }

  public Pose3d get3DPose() {
    return new Pose3d(
        -0.32385,
        0,
        0.6312130886,
        new Rotation3d(0, -Units.degreesToRadians(outtakeInputs.pivotMotorPosition), 0));
  }
}
