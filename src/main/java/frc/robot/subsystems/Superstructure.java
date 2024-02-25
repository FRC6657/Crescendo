// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.MAXSwerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;

import java.io.ObjectOutputStream.PutField;
import java.util.ArrayList;
import java.util.List;

import org.ejml.equation.Sequence;
import org.littletonrobotics.junction.Logger;
import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.signals.InvertedValue;
public class Superstructure {
  MAXSwerve drivebase;
  Intake intake;
  Outtake outtake;
  Climb climb;
  private List<Command> commandQueue = new ArrayList<Command>();
  boolean intakeLock = false;
  boolean outtakeLock = false;
  Trigger intookPiece;
  private enum noteState{Processing, Intake, Outtake, None};
  noteState ourNoteState = noteState.None;
  private boolean ampMode = true;
  public Superstructure(MAXSwerve drivebase, Intake intake, Outtake outtake, Climb climb) {
    this.drivebase = drivebase;
    this.intake = intake;
    this.outtake = outtake;
    this.climb = climb;

    intookPiece = new Trigger(intake::intookPiece);

    intookPiece.onTrue(
      Commands.sequence(
        Commands.runOnce(() -> ourNoteState = noteState.Processing),
        retractIntake(),
        Commands.waitUntil(intake::atSetpoint),
        feedPieceChamberNote(),
        endUpInRightSpot()
      ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withTimeout(5)
      .andThen(Commands.runOnce(() -> ourNoteState = noteState.Processing))
    );
    
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

  public Command extendIntake(){
    return Commands.sequence(
      Commands.runOnce(() -> intake.changePivotSetpoint(IntakeConstants.kMinPivotAngle), intake),
      Commands.run(()-> intake.changeRollerSpeed(0.5), intake)
    );
  }

  public Command retractIntake(){
    return Commands.sequence(
      Commands.runOnce(()-> intake.changeRollerSpeed(0), intake),
      Commands.runOnce(() -> intake.changePivotSetpoint(IntakeConstants.kMaxPivotAngle), intake)
      );
  }

  public Command feedPieceChamberNote(){
    return Commands.sequence(
      Commands.runOnce(() -> intake.changeRollerSpeed(-0.6), intake),
      Commands.runOnce(() -> outtake.changeRPMSetpoint(300), outtake),
      Commands.waitUntil(outtake::beamBroken),
      Commands.runOnce(() -> outtake.changeRPMSetpoint(0), outtake),
      Commands.runOnce(() -> intake.changeRollerSpeed(0), intake)

    );
  }

  public Command endUpInRightSpot(){
    if(ampMode){
      return Commands.waitSeconds(0);
    }else{
      return Commands.sequence(
        Commands.runOnce(() -> intake.changeRollerSpeed(0.4), intake),
        Commands.runOnce(() -> outtake.changeRPMSetpoint(-300), outtake),
        Commands.waitUntil(()-> !outtake.beamBroken()),
        Commands.runOnce(() -> intake.changeRollerSpeed(0), intake),
        Commands.runOnce(() -> outtake.changeRPMSetpoint(0), outtake)
      );
    }
  }

  public Command shootInSpeaker(){
    return Commands.sequence(
      Commands.waitSeconds(0)
    );
  }


  public Command testAuto(){
    return Commands.sequence(
      runPath("testPath.1", true, true),
      retractIntake(),
      runPath("testPath.2", true, false),
      Commands.runOnce(() -> drivebase.choreoDrive(0, 0, 0), drivebase),
      Commands.waitUntil(() -> false)
      // shootInSpeaker()
    );
  }
  private Command runPath(String pathName, boolean isBlue, boolean isFirstPath) {
    ChoreoTrajectory traj = Choreo.getTrajectory(pathName);
    var thetaController = AutoConstants.kThetaController;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Pose2d initialPose = isBlue ? traj.getInitialPose() : traj.flipped().getInitialPose();

    Command setPoseCommand;

    if(isFirstPath){
      setPoseCommand = Commands.runOnce(()-> drivebase.setPose(initialPose),drivebase);
    }
    else{
      setPoseCommand = Commands.runOnce(()-> drivebase.setPose(drivebase.getPose()),drivebase);
    }

    Command swerveCommand = Choreo.choreoSwerveCommand(
            traj, // Choreo trajectory from above
            drivebase::getPose,
            AutoConstants.kXController,
            AutoConstants.kYController,
            thetaController,
            (ChassisSpeeds speeds) ->
                drivebase.choreoDrive(speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond),
            ()->!isBlue, // Whether or not to mirror the path based on alliance (this assumes the path
            // is created for the blue alliance)
            drivebase
            );
    return Commands.sequence(
        setPoseCommand,
        swerveCommand,
        Commands.runOnce(() -> drivebase.choreoDrive(0, 0, 0), drivebase)
    );
  } 
  
}