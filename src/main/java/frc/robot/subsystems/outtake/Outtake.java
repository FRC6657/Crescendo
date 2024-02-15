// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {

  // IO
  private final OuttakeIO outtakeIO;
  private final OuttakeIOInputsAutoLogged outtakeInputs = new OuttakeIOInputsAutoLogged();

  /** The Robot's Outtake Subsystem */
  public Outtake(OuttakeIO outtakeIO) {
    this.outtakeIO = outtakeIO;
  }

  /**
   * Change the setpoint of the shooter pivot
   *
   * @param angleDegrees The new setpoint in degrees
   *     <p>Acceptable Range: [-27.5, 152.25] Increase in angle moves the pivot towards the back of
   *     the robot
   */
  public Command changePivotSetpoint(double angleDegrees) {
    return this.runOnce(() -> outtakeIO.changePivotSetpoint(angleDegrees));
  }

  /**
   * Change the setpoint of the flywheel
   *
   * @param rpm The new setpoint in RPM (Rotations per minute)
   *     <p>Aceptable range: [-3190, 3190] Positive RPM the note towards the back of the robot
   */
  public Command changeRPMSetpoint(double rpm) {
    return this.runOnce(() -> outtakeIO.changeFlywheelSetpoint(rpm));
  }

  @Override
  public void periodic() {
    outtakeIO.updateInputs(outtakeInputs);
    Logger.processInputs("Outtake", outtakeInputs);
  }

  /**
   * Get the 3D pose of the outtake's pivot
   *
   * @return The 3D pose of the outtake
   */
  public Pose3d get3DPose() {
    return new Pose3d(
        -0.32385,
        0,
        0.6312130886,
        new Rotation3d(0, -Units.degreesToRadians(outtakeInputs.pivotMotorPosition + 10), 0));
  }
}
