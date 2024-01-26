// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

  private final Climber leftClimber;
  private final Climber rightClimber;

  private double climbSetpoint;

  public Climb(ClimberIO[] test) {
    this.leftClimber = new Climber(test[0], "left ");
    this.rightClimber = new Climber(test[1], "right ");
  }

  @Override
  public void periodic() {
    leftClimber.updateInputs();
    rightClimber.updateInputs();
  }

  public Command run() {
    return this.run(
        () -> {
          leftClimber.run(climbSetpoint);
        });
  }

  public Pose3d[] get3DPoses() {
    return new Pose3d[] {leftClimber.get3DPose(), rightClimber.get3DPose()};
  }
}
