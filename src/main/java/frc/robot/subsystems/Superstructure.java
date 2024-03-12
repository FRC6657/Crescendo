// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.MAXSwerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.util.NoteVisualizer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

public class Superstructure {
  MAXSwerve drivebase;
  Intake intake;
  Outtake outtake;
  Climb climb;
  Trigger noteDetector;

  public enum noteState {
    Processing,
    Intake,
    Outtake,
    None
  };

  @AutoLogOutput(key = "Superstructure/Note State")
  private noteState currentNoteState = noteState.None;

  private enum ScoringMode {
    Amp,
    Speaker
  };

  @AutoLogOutput(key = "Superstructure/Scoring Mode")
  private ScoringMode currentScoringMode = ScoringMode.Speaker;

  @AutoLogOutput(key = "Superstructure/Ready")
  private boolean readyToShoot = false;

  private boolean climbersUp = false;

  public Superstructure(MAXSwerve drivebase, Intake intake, Outtake outtake, Climb climb) {

    this.drivebase = drivebase;
    this.intake = intake;
    this.outtake = outtake;
    this.climb = climb;

    noteDetector = new Trigger(() ->false);

    noteDetector.onTrue(
        Commands.sequence(
                Commands.runOnce(() -> currentNoteState = noteState.Processing),
                Commands.waitSeconds(0),
                retractIntake(),
                Commands.waitUntil(intake::atSetpoint),
                chamberNote(),
                relocateNote())
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    Logger.recordOutput("Superstructure/Latest Event", "Superstructure Initialized");
  }

  public Command processNote(){
    return Commands.sequence(
                Commands.runOnce(() -> currentNoteState = noteState.Processing),
                Commands.waitSeconds(0),
                retractIntake(),
                Commands.waitUntil(intake::atSetpoint),
                chamberNote(),
                relocateNote());
  }

  public void update3DPose() {
    Pose3d[] mechanismPoses = new Pose3d[4];
    mechanismPoses[0] = outtake.get3DPose();
    mechanismPoses[1] = intake.get3DPose();
    mechanismPoses[2] = climb.get3DPoses()[0];
    mechanismPoses[3] = climb.get3DPoses()[1];
    Logger.recordOutput("3D Poses", mechanismPoses);
  }

  public Command logEvent(String event) {
    return Commands.runOnce(() -> Logger.recordOutput("Superstructure/Latest Event", event));
  }

  public Command raiseClimbers() {
    return stowOuttake()
        .onlyIf(() -> (readyToShoot && currentScoringMode == ScoringMode.Amp))
        .andThen(Commands.runOnce(() -> readyToShoot = false))
        .beforeStarting(logEvent("Raising Climbers"))
        .andThen(
            Commands.sequence(
                Commands.waitUntil(outtake::atPivotSetpoint),
                climb.changeSetpoint(ClimbConstants.kMaxHeight - 0.5),
                Commands.runOnce(() -> climbersUp = true)));
  }

  public Command lowerClimbers() {
    return climb.changeSetpoint(0.1).beforeStarting(logEvent("Lowering Climbers"));
  }

  public Command stowOuttake() {
    return Commands.sequence(
        logEvent("Stowing Outtake"),
        outtake.changeRPMSetpoint(0),
        outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle));
  }

  public Command raiseOuttake() {
    return Commands.sequence(
        logEvent("Raising Outtake"),
        lowerClimbers().onlyIf(() -> climbersUp).andThen(Commands.waitUntil(() -> !climbersUp)),
        outtake.changePivotSetpoint(OuttakeConstants.kMaxPivotAngle),
        Commands.waitUntil(outtake::atPivotSetpoint));
  }

  public Command extendIntake() {
    return Commands.sequence(
        logEvent("Extending Intake"),
        intake.changePivotSetpoint(IntakeConstants.kMinPivotAngle),
        intake.changeRollerSpeed(IntakeConstants.kGroundIntakeSpeed));
  }

  public Command retractIntake() {
    return Commands.sequence(
        logEvent("Retracting Intake"),
        intake.changeRollerSpeed(0),
        intake.changePivotSetpoint(IntakeConstants.kMaxPivotAngle));
  }

  public Command zeroRobot() {
    return Commands.sequence(
        logEvent("Zeroing Robot"),
        outtake.changeRPMSetpoint(0),
        outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle),
        intake.changeRollerSpeed(0),
        intake.changePivotSetpoint(IntakeConstants.kMaxPivotAngle),
        Commands.waitUntil(outtake::atPivotSetpoint),
        Commands.waitUntil(outtake::atFlywheelSetpoint),
        Commands.waitUntil(intake::atSetpoint));
  }

  public Command relocateNote() {

    Command[] commands = new Command[4];

    commands[0] = zeroRobot().onlyIf(() -> readyToShoot);

    commands[1] =
        Commands.sequence(logEvent("Chambering Note"), chamberNote())
            .onlyIf(
                () ->
                    currentScoringMode == ScoringMode.Amp && currentNoteState == noteState.Intake);

    commands[2] =
        Commands.sequence(
                logEvent("Unchambering Note"),
                Commands.runOnce(() -> currentNoteState = noteState.Processing),
                intake.changeRollerSpeed(-IntakeConstants.kFeedSpeed),
                outtake.changeRPMSetpoint(-OuttakeConstants.kFeedRPM),
                Commands.waitUntil(() -> !outtake.beamBroken()).unless(RobotBase::isSimulation),
                intake.changeRollerSpeed(0),
                outtake.changeRPMSetpoint(0),
                Commands.runOnce(() -> currentNoteState = noteState.Intake))
            .onlyIf(
                () ->
                    currentScoringMode == ScoringMode.Speaker
                        && currentNoteState == noteState.Outtake);

    commands[3] = readyPiece().onlyIf(() -> readyToShoot);

    return Commands.sequence(commands);
  }

  public Command readyPiece() {

    Command[] commands = new Command[2];

    commands[0] =
        Commands.sequence(
                logEvent("Readying Robot for Amp"),
                outtake.changePivotSetpoint(OuttakeConstants.kPivotAmpAngle),
                Commands.runOnce(() -> readyToShoot = true))
            .beforeStarting(
                lowerClimbers()
                    .andThen(Commands.waitUntil(climb::atSetpoint))
                    .onlyIf(() -> climbersUp))
            .onlyIf(
                () ->
                    currentScoringMode == ScoringMode.Amp && currentNoteState == noteState.Outtake);

    commands[1] =
        Commands.sequence(
                logEvent("Readying Robot for Speaker"),
                outtake.changeRPMSetpoint(OuttakeConstants.kSpeakerRPM),
                Commands.runOnce(() -> readyToShoot = true))
            .onlyIf(
                () ->
                    currentScoringMode == ScoringMode.Speaker
                        && currentNoteState == noteState.Intake);

    return Commands.sequence(commands);
  }

  public Command chamberNote() {
    return Commands.sequence(
            logEvent("Chambering Note"),
            Commands.runOnce(() -> currentNoteState = noteState.Processing),
            intake.changeRollerSpeed(IntakeConstants.kFeedSpeed),
            outtake.changeRPMSetpoint(OuttakeConstants.kFeedRPM),
            Commands.waitUntil(outtake::beamBroken).unless(RobotBase::isSimulation),
            outtake.changeRPMSetpoint(0),
            intake.changeRollerSpeed(0),
            Commands.runOnce(() -> currentNoteState = noteState.Outtake))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command shootPiece() {

    Command[] commands = new Command[3];

    commands[0] = readyPiece().onlyIf(() -> !readyToShoot);

    commands[1] =
        Commands.sequence(
                logEvent("Scoring Amp"),
                outtake.changeRPMSetpoint(OuttakeConstants.kAmpRPM),
                Commands.waitUntil(() -> !outtake.beamBroken()).unless(RobotBase::isSimulation),
                outtake.changeRPMSetpoint(0),
                outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle),
                Commands.runOnce(() -> currentNoteState = noteState.None),
                Commands.runOnce(() -> readyToShoot = false))
            .onlyIf(() -> currentScoringMode == ScoringMode.Amp);

    commands[2] =
        Commands.sequence(
                logEvent("Scoring Speaker"),
                outtake.changeRPMSetpoint(OuttakeConstants.kSpeakerRPM),
                Commands.waitUntil(outtake::atFlywheelSetpoint),
                intake.changeRollerSpeed(IntakeConstants.kFeedSpeed),
                Commands.waitUntil(outtake::beamBroken).unless(RobotBase::isSimulation),
                Commands.waitSeconds(0.1),
                outtake.changeRPMSetpoint(0),
                intake.changeRollerSpeed(0),
                Commands.runOnce(() -> readyToShoot = false),
                Commands.runOnce(() -> currentNoteState = noteState.None))
            .onlyIf(() -> currentScoringMode == ScoringMode.Speaker);

    return Commands.sequence(commands).andThen(NoteVisualizer.shoot());
  }

  public Command speakerMode() {
    return Commands.sequence(
            Commands.runOnce(() -> currentScoringMode = ScoringMode.Speaker), relocateNote())
        .unless(() -> currentScoringMode == ScoringMode.Speaker);
  }

  public Command ampMode() {
    return Commands.sequence(
            Commands.runOnce(() -> currentScoringMode = ScoringMode.Amp), relocateNote())
        .unless(() -> currentScoringMode == ScoringMode.Amp);
  }

  public Command firstReset() {
    return Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll());
  }

  public Command secondReset() {
    return Commands.sequence(
        intake.changePivotSetpoint(IntakeConstants.kMaxPivotAngle),
        intake.changeRollerSpeed(0),
        outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle),
        outtake.changeRPMSetpoint(0),
        Commands.runOnce(() -> currentScoringMode = ScoringMode.Speaker),
        Commands.runOnce(() -> currentNoteState = noteState.None),
        Commands.runOnce(() -> readyToShoot = false));
  }

  public void overrideNoteState(noteState state) {
    currentNoteState = state;
  }

  private boolean isRed() {
    boolean isRed = false;
    if (DriverStation.getAlliance().isPresent()) {
      isRed = (DriverStation.getAlliance().get() == Alliance.Red);
    }
    return isRed;
  }

  public Command autoStart(Pose2d bluePos, Pose2d redPos) {
    return Commands.sequence(
        Commands.runOnce(() -> drivebase.setPose(isRed() ? redPos : bluePos)),
        Commands.runOnce(() -> currentNoteState = noteState.Intake),
        drivebase.goToShotPoint().alongWith(readyPiece()),
        shootPiece());
  }

  public Command CenterFenderS0() {
    return Commands.sequence(
        autoStart(AutoConstants.BLUE_CENTER_FENDER, AutoConstants.RED_CENTER_FENDER));
  }

  public Command CenterFenderS02(){
    return Commands.sequence(
      CenterFenderS0(),
      extendIntake(),
      Commands.waitUntil(intake::atSetpoint),
      Commands.print("Before path"),
      AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("CenterFender02")),
      processNote(),
      
      drivebase.goToShotPoint(),
      Commands.print("after shotpoint"),
      Commands.waitUntil(() -> currentNoteState == noteState.Intake),
      Commands.waitSeconds(1),
      shootPiece()
    );
  }
}
