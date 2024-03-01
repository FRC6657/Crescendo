// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import java.util.function.BooleanSupplier;
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

  @AutoLogOutput(key = "Ready to Shoot")
  private boolean readyToShoot = true;

  public Superstructure(MAXSwerve drivebase, Intake intake, Outtake outtake, Climb climb) {
    this.drivebase = drivebase;
    this.intake = intake;
    this.outtake = outtake;
    this.climb = climb;

    noteDetected = new Trigger(intake::noteDetected);

    noteDetected.onTrue(
        Commands.sequence(
                Commands.runOnce(() -> currentNoteState = noteState.Processing),
                retractIntake(),
                Commands.waitUntil(intake::atSetpoint),
                feedPieceChamberNote(),
                relocateNote())
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
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

  public Command relocateNote() {
    Command returnCommand = Commands.none();
    if ((currentNoteState == noteState.None)
        || (currentScoringModeState == scoringModeState.Amp
            && currentNoteState == noteState.Outtake)
        || (currentScoringModeState == scoringModeState.Speaker
            && currentNoteState == noteState.Intake)) {
    } else {
      if (readyToShoot) {
        returnCommand.andThen(
            Commands.sequence(
                outtake.changeRPMSetpoint(0),
                outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle),
                outtake.waitUntilFlywheelAtSetpoint(),
                outtake.waitUntilPivotAtSetpoint()));
      }
      if (currentScoringModeState == scoringModeState.Amp && currentNoteState == noteState.Intake) {
        returnCommand.andThen(feedPieceChamberNote());
      } else if (currentScoringModeState == scoringModeState.Speaker
          && currentNoteState == noteState.Outtake) {
        returnCommand.andThen(
            Commands.sequence(
                Commands.runOnce(() -> currentNoteState = noteState.Processing),
                intake.changeRollerSpeed(0.4),
                outtake.changeRPMSetpoint(-300),
                Commands.waitUntil(() -> !outtake.beamBroken()).unless(RobotBase::isSimulation),
                intake.changeRollerSpeed(0),
                outtake.changeRPMSetpoint(0),
                Commands.runOnce(() -> currentNoteState = noteState.Intake)));
      }
      if (readyToShoot) {
        returnCommand.andThen(readyPiece());
      }
    }

    return returnCommand.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command readyPiece() {
    Command returnCommand = Commands.print("Piece and state were not correct");
    if (currentScoringModeState == scoringModeState.Amp && currentNoteState == noteState.Outtake) {
      returnCommand =
          Commands.sequence(
              outtake.changePivotSetpoint(96), Commands.runOnce(() -> readyToShoot = true));
    } else if (currentScoringModeState == scoringModeState.Speaker
        && currentNoteState == noteState.Intake) {
      returnCommand =
          Commands.sequence(
              outtake.changeRPMSetpoint(OuttakeConstants.kMaxFlywheelRpm),
              Commands.runOnce(() -> readyToShoot = true));
    }
    return returnCommand;
  }

  public Command feedPieceChamberNote() {
    return Commands.sequence(
            Commands.runOnce(() -> currentNoteState = noteState.Processing),
            intake.changeRollerSpeed(-0.6),
            outtake.changeRPMSetpoint(300),
            Commands.waitUntil(outtake::beamBroken).unless(RobotBase::isSimulation),
            outtake.changeRPMSetpoint(0),
            intake.changeRollerSpeed(0),
            Commands.runOnce(() -> currentNoteState = noteState.Outtake))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command shootPiece() {
    Command returnCommand = Commands.none();

    if (!readyToShoot) {
      returnCommand.andThen(readyPiece());
    }

    if (currentScoringModeState == scoringModeState.Amp && currentNoteState == noteState.Outtake) {
      returnCommand.andThen(
          Commands.sequence(
              outtake.waitUntilPivotAtSetpoint(), // we might not need this
              outtake.changeRPMSetpoint(600),
              Commands.waitUntil(() -> !outtake.beamBroken()).unless(RobotBase::isSimulation),
              outtake.changeRPMSetpoint(0),
              outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle),
              Commands.runOnce(() -> currentNoteState = noteState.None)));
    } else if (currentScoringModeState == scoringModeState.Speaker
        && currentNoteState == noteState.Intake) {
      returnCommand.andThen(
          Commands.sequence(
              outtake.changeRPMSetpoint(OuttakeConstants.kMaxFlywheelRpm),
              outtake.waitUntilFlywheelAtSetpoint(),
              intake.changeRollerSpeed(-0.6),
              Commands.waitUntil(outtake::beamBroken).unless(RobotBase::isSimulation),
              outtake.changeRPMSetpoint(0),
              intake.changeRollerSpeed(0),
              Commands.runOnce(() -> currentNoteState = noteState.None)));
    }
    return returnCommand.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command speakerMode() {
    return Commands.sequence(
        Commands.runOnce(() -> currentScoringModeState = scoringModeState.Speaker), relocateNote());
  }

  public Command ampMode() {
    return Commands.sequence(
        Commands.runOnce(() -> currentScoringModeState = scoringModeState.Amp), relocateNote());
  }

  // hopefully we can get rid of this later on
  public Command resetEverything() {
    return Commands.sequence(
        intake.changePivotSetpoint(IntakeConstants.kMaxPivotAngle),
        intake.changeRollerSpeed(0),
        outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle),
        outtake.changeRPMSetpoint(0),
        Commands.runOnce(() -> currentScoringModeState = scoringModeState.Amp),
        Commands.runOnce(() -> currentNoteState = noteState.None),
        Commands.runOnce(() -> readyToShoot = false));
  }

  public Command testAuto() {
    return Commands.sequence(
        shootPiece(),
        extendIntake(),
        Commands.waitUntil(() -> intake.atSetpoint()), // would be nice if we could get rid of this
        runPath("testPath.1", true),
        retractIntake(),
        runPath("testPath.2", false),
        Commands.waitUntil(() -> currentNoteState == noteState.Intake),
        shootPiece());
  }

  public Command meterTestAuto() {
    return runPath("1MeterTest", true);
  }

  public Command interuptChoreoTest() {
    return Commands.sequence(extendIntake(), runPath("interuptChoreoTest", true));
  }

  private boolean isRed() {
    boolean isRed = false;
    if (DriverStation.getAlliance().isPresent()) {
      isRed = (DriverStation.getAlliance().get() == Alliance.Red);
    }
    return isRed;
  }

  private Command runPath(String pathName, boolean isFirstPath) {

    ChoreoTrajectory traj = Choreo.getTrajectory(pathName);
    var thetaController = AutoConstants.kThetaController;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Command setPoseCommand =
        Commands.either(
            Commands.runOnce(
                () ->
                    drivebase.setPose(
                        !isRed()
                            ? traj.getInitialPose()
                            : traj.flipped().getInitialPose()),
                drivebase),
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
            this::isRed, // Whether or not to mirror the path based on alliance (this assumes
            // the path is created for the blue alliance)
            drivebase);
    return Commands.sequence(
            Commands.runOnce(() -> Logger.recordOutput("Current Path", traj.getPoses())),
            setPoseCommand,
            swerveCommand,
            Commands.runOnce(() -> drivebase.choreoStop(), drivebase))
        .until(intake::noteDetected)
        .handleInterrupt(() -> drivebase.choreoStop());
  }
}
