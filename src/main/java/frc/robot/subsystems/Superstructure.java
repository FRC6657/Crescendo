// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.MAXSwerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  MAXSwerve drivebase;
  Intake intake;
  Outtake outtake;
  Climb climb;
  Trigger noteDetected;

  private enum noteState {
    Processing,
    Intake,
    Outtake,
    None
  };

  @AutoLogOutput(key = "Note State")
  noteState currentNoteState = noteState.None;

  private enum scoringModeState {
    Amp,
    Speaker
  };

  @AutoLogOutput(key = "Scoring Note State")
  private scoringModeState currentScoringModeState = scoringModeState.Speaker;

  public Superstructure(MAXSwerve drivebase, Intake intake, Outtake outtake, Climb climb) {
    this.drivebase = drivebase;
    this.intake = intake;
    this.outtake = outtake;
    this.climb = climb;

    noteDetected = new Trigger(() -> intake.noteDetected());

    noteDetected.onTrue(
        Commands.sequence(
                Commands.runOnce(() -> currentNoteState = noteState.Processing),
                retractIntake(),
                Commands.waitUntil(intake::atSetpoint),
                feedPieceChamberNote(),
                endUpInRightSpot())
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            .withTimeout(10));
  }

  public void update3DPose() {
    Pose3d[] mechanismPoses = new Pose3d[4];
    mechanismPoses[0] = outtake.get3DPose();
    mechanismPoses[1] = intake.get3DPose();
    mechanismPoses[2] = climb.get3DPoses()[0];
    mechanismPoses[3] = climb.get3DPoses()[1];
    Logger.recordOutput("3D Poses", mechanismPoses);
  }

  public Command extendIntake() {
    return Commands.sequence(
        intake.changePivotSetpoint(IntakeConstants.kMinPivotAngle), intake.changeRollerSpeed(0.5));
  }

  public Command retractIntake() {
    return Commands.sequence(
        intake.changeRollerSpeed(0), intake.changePivotSetpoint(IntakeConstants.kMaxPivotAngle));
  }

  public Command feedPieceChamberNote() {
    return Commands.sequence(
        intake.changeRollerSpeed(-0.6),
        outtake.changeRPMSetpoint(300),
        Commands.waitUntil(outtake::beamBroken).unless(RobotBase::isSimulation),
        outtake.changeRPMSetpoint(0),
        intake.changeRollerSpeed(0));
  }

  public Command endUpInRightSpot() {
    return Commands.either(
        Commands.runOnce(() -> currentNoteState = noteState.Outtake),
        Commands.sequence(
            intake.changeRollerSpeed(0.4),
            outtake.changeRPMSetpoint(-300),
            Commands.waitUntil(() -> !outtake.beamBroken()).unless(RobotBase::isSimulation),
            intake.changeRollerSpeed(0),
            outtake.changeRPMSetpoint(0),
            Commands.runOnce(() -> currentNoteState = noteState.Intake)),
        () -> currentScoringModeState == scoringModeState.Amp);
  }

  public Command shootPiece() {
    return Commands.either(
            Commands.sequence(
                outtake.changePivotSetpoint(96),
                outtake.waitUntilPivotAtSetpoint(), // we might not need this
                outtake.changeRPMSetpoint(600),
                Commands.waitUntil(() -> !outtake.beamBroken()).unless(RobotBase::isSimulation),
                outtake.changeRPMSetpoint(0),
                outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle),
                Commands.runOnce(() -> currentNoteState = noteState.None)),
            Commands.sequence(
                outtake.changeRPMSetpoint(OuttakeConstants.kMaxFlywheelRpm),
                outtake.waitUntilFlywheelAtSetpoint(),
                intake.changeRollerSpeed(-0.6),
                Commands.waitUntil(outtake::beamBroken).unless(RobotBase::isSimulation),
                outtake.changeRPMSetpoint(0),
                intake.changeRollerSpeed(0),
                Commands.runOnce(() -> currentNoteState = noteState.None)),
            () -> currentScoringModeState == scoringModeState.Amp)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command testAuto() {
    return Commands.sequence(
        shootPiece(),
        extendIntake(),
        runPath("testPath.1", true, true),
        retractIntake(),
        runPath("testPath.2", true, false),
        Commands.waitUntil(() -> currentNoteState == noteState.Intake),
        shootPiece());
  }

  private Command runPath(String pathName, boolean isBlue, boolean isFirstPath) {
    ChoreoTrajectory traj = Choreo.getTrajectory(pathName);
    var thetaController = AutoConstants.kThetaController;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Pose2d initialPose = isBlue ? traj.getInitialPose() : traj.flipped().getInitialPose();

    Command setPoseCommand =
        Commands.either(
            Commands.runOnce(() -> drivebase.setPose(initialPose), drivebase),
            Commands.none(),
            () -> isFirstPath);

    Command swerveCommand =
        Choreo.choreoSwerveCommand(
            traj, // Choreo trajectory from above
            drivebase::getPose,
            AutoConstants.kXController,
            AutoConstants.kYController,
            thetaController,
            (ChassisSpeeds speeds) -> drivebase.runChassisSpeeds(speeds),
            () -> !isBlue, // Whether or not to mirror the path based on alliance (this assumes the
            // path is created for the blue alliance)
            drivebase);
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput(pathName, traj.getPoses())),
        setPoseCommand,
        swerveCommand,
        Commands.runOnce(() -> drivebase.choreoStop(), drivebase));
  }
}
