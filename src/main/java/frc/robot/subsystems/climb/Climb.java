// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

  private final Climber leftClimber;
  private final Climber rightClimber;

  public Climb(ClimberIO leftClimber, ClimberIO rightClimber) {
    this.leftClimber = new Climber(leftClimber, "Left");
    this.rightClimber = new Climber(rightClimber, "Right");
  }

  @Override
  public void periodic() {
    leftClimber.updateInputs();
    rightClimber.updateInputs();
  }

  public Pose3d[] get3DPoses() {
    return new Pose3d[] {leftClimber.get3DPose(), rightClimber.get3DPose()};
  }

}
