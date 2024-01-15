// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;

public class Climber {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private String name;

  public Climber(ClimberIO climberIO, String name) {
    this.io = climberIO;
    this.name = name;
  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs(name + " Climber", inputs);
  }

  public void run(double height) {
    io.run(height);
  }

  public Pose3d get3DPose() {
    return new Pose3d(0, 0, inputs.position, new Rotation3d(0, 0, 0));
  }

}
