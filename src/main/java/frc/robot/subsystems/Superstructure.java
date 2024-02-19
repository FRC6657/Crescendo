// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.MAXSwerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  MAXSwerve drivebase;
  Intake intake;
  Outtake outtake;
  Climb climb;

  private List<Command> commandQueue = new ArrayList<Command>();

  boolean intakeLock = false;
  boolean outtakeLock = false;

  public Superstructure(MAXSwerve drivebase, Intake intake, Outtake outtake, Climb climb) {
    this.drivebase = drivebase;
    this.intake = intake;
    this.outtake = outtake;
    this.climb = climb;
  }

  public void update3DPose() {
    Pose3d[] mechanismPoses = new Pose3d[4];
    mechanismPoses[0] = outtake.get3DPose();
    mechanismPoses[1] = intake.get3DPose();
    mechanismPoses[2] = climb.get3DPoses()[0];
    mechanismPoses[3] = climb.get3DPoses()[1];
    Logger.recordOutput("3D Poses", mechanismPoses);
  }

  public void processQueue() {
    if (commandQueue.size() > 0) {

      Command topCommand = commandQueue.get(0);

      System.out.println("Processing Command" + topCommand.getName());

      switch (topCommand.getName()) {
        default:
          denyCommand(topCommand);
          break;
      }

      commandQueue.remove(0);
    }
  }

  public Command queueCommand(Command command) {
    try {
      command.getName();
    } catch (Exception e) {
      return Commands.print("Command has no name!");
    }
    return Commands.runOnce(
        () -> {
          commandQueue.add(command);
        });
  }

  public void denyCommand(Command command) {
    System.out.println("Command" + command.getName() + "denied");
  }

  public Command lockIntake() {
    return Commands.runOnce(() -> intakeLock = true);
  }

  public Command unlockIntake() {
    return Commands.runOnce(() -> intakeLock = false);
  }

  public Command lockOuttake() {
    return Commands.runOnce(() -> outtakeLock = true);
  }

  public Command unlockOuttake() {
    return Commands.runOnce(() -> outtakeLock = false);
  }

}
