// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  MAXSwerve drivebase;
  Intake intake;
  Outtake outtake;
  Climb climb;
  Trigger noteDetector;

  private enum noteState {
    Processing,
    Intake,
    Outtake,
    None
  };

  @AutoLogOutput(key = "Superstructure/Note State")
  noteState currentNoteState = noteState.None;

  private enum ScoringMode {
    Amp,
    Speaker
  };

  @AutoLogOutput(key = "Superstructure/Scoring Mode")
  private ScoringMode currentScoringMode = ScoringMode.Speaker;

  @AutoLogOutput(key = "Superstructure/Ready")
  private boolean readyToShoot = false;

  @AutoLogOutput(key = "Superstructure/Climbers Up")
  private boolean climbersUp = false;

  public Superstructure(MAXSwerve drivebase, Intake intake, Outtake outtake, Climb climb) {

    this.drivebase = drivebase;
    this.intake = intake;
    this.outtake = outtake;
    this.climb = climb;

    noteDetector = new Trigger(() -> true);

    noteDetector.onTrue(
        Commands.sequence(
                Commands.runOnce(() -> currentNoteState = noteState.Processing),
                Commands.waitSeconds(0.4),
                retractIntake(),
                Commands.waitUntil(intake::atSetpoint),
                chamberNote(),
                relocateNote())
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    Logger.recordOutput("Superstructure/Latest Event", "Superstructure Initialized");

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
    // return stowOuttake()
    //     .onlyIf(() -> (readyToShoot && currentScoringMode == ScoringMode.Amp))
    //     .andThen(Commands.runOnce(() -> readyToShoot = false))
    //     .beforeStarting(logEvent("Raising Climbers"))
    //     .andThen(
    //         Commands.sequence(
    //             outtake.waitUntilPivotAtSetpoint(),
    //             climb.changeSetpoint(ClimbConstants.kMaxHeight),
    //             Commands.runOnce(() -> climbersUp = true)));

    return climb.changeSetpoint(ClimbConstants.kMaxHeight - 0.5);
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
        outtake.waitUntilPivotAtSetpoint());
  }

  public Command extendIntake() {
    return Commands.sequence(
        logEvent("Extending Intake"),
        intake.changePivotSetpoint(IntakeConstants.kMinPivotAngle),
        intake.changeRollerSpeed(0.6));
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
        outtake.waitUntilFlywheelAtSetpoint(),
        outtake.waitUntilPivotAtSetpoint(),
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
                intake.changeRollerSpeed(0.3),
                outtake.changeRPMSetpoint(-300),
                Commands.waitUntil(() -> !outtake.beamBroken()).unless(RobotBase::isSimulation),
                Commands.waitSeconds(0),
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
                outtake.changeRPMSetpoint(OuttakeConstants.kMaxFlywheelRpm),
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
            intake.changeRollerSpeed(-0.8),
            outtake.changeRPMSetpoint(300),
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
                outtake.waitUntilPivotAtSetpoint(), // we might not need this
                outtake.changeRPMSetpoint(600),
                Commands.waitUntil(() -> !outtake.beamBroken()).unless(RobotBase::isSimulation),
                outtake.changeRPMSetpoint(0),
                outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle),
                Commands.runOnce(() -> currentNoteState = noteState.None),
                Commands.runOnce(() -> readyToShoot = false))
            .onlyIf(() -> currentScoringMode == ScoringMode.Amp);

    commands[2] =
        Commands.sequence(
                logEvent("Scoring Speaker"),
                outtake.changeRPMSetpoint(OuttakeConstants.kMaxFlywheelRpm),
                outtake.waitUntilFlywheelAtSetpoint(),
                intake.changeRollerSpeed(-0.6),
                Commands.waitUntil(outtake::beamBroken).unless(RobotBase::isSimulation),
                outtake.changeRPMSetpoint(0),
                intake.changeRollerSpeed(0),
                Commands.runOnce(() -> readyToShoot = false),
                Commands.runOnce(() -> currentNoteState = noteState.None))
            .onlyIf(() -> currentScoringMode == ScoringMode.Speaker);

    return Commands.sequence(commands);
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

  public Command ampStart() {
    return Commands.sequence(
        Commands.runOnce(() -> currentScoringMode = ScoringMode.Amp),
        Commands.runOnce(() -> currentNoteState = noteState.Outtake));
  }

  public Command speakerStart() {
    return Commands.sequence(
        Commands.runOnce(() -> currentScoringMode = ScoringMode.Speaker),
        Commands.runOnce(() -> currentNoteState = noteState.Intake));
  }

  public Command nothingStart() {
    return Commands.sequence(
        Commands.runOnce(() -> currentScoringMode = ScoringMode.Speaker),
        Commands.runOnce(() -> currentNoteState = noteState.None));
  }

  // hopefully we can get rid of this later on
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

  public Command firstReset() {
    return Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll());
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
                        !isRed() ? traj.getInitialPose() : traj.flipped().getInitialPose()),
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
            Commands.runOnce(
                () -> Logger.recordOutput("Superstructure/Current Path", traj.getPoses())),
            setPoseCommand,
            swerveCommand,
            Commands.runOnce(() -> drivebase.choreoStop(), drivebase))
        .until(intake::noteDetected)
        .handleInterrupt(() -> drivebase.choreoStop());
  }

  public Command testPivot() {
    return outtake.changePivotSetpoint(96);
  }

  // Autos

  public Command testAuto() {
    return Commands.sequence(
        speakerStart(),
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
    return Commands.sequence(
        extendIntake(),
        Commands.waitUntil(intake::atSetpoint),
        runPath("interuptChoreoTest", true));
  }

  public Command twoCenter() {
    return Commands.sequence(
        speakerStart(),
        shootPiece(),
        extendIntake(),
        runPath("2CenterTest.1", true),
        retractIntake(),
        Commands
            .parallel( // this is so it will be ready to shoot right when it gets back. Might not be
                // needed
                runPath("2CenterTest.2", false),
                Commands.sequence(
                    Commands.runOnce(() -> currentNoteState = noteState.Intake)
                        .onlyIf(RobotBase::isSimulation),
                    Commands.waitUntil(() -> currentNoteState == noteState.Intake),
                    readyPiece())),
        shootPiece(),
        extendIntake(),
        Commands.waitUntil(intake::atSetpoint),
        runPath("2CenterTest.3", false),
        retractIntake(),
        Commands.runOnce(() -> currentNoteState = noteState.Intake).onlyIf(RobotBase::isSimulation),
        runPath("2CenterTest.4", true),
        Commands.waitUntil(() -> currentNoteState == noteState.Intake),
        shootPiece());
  }

  public Command ifTest() {
    return Commands.sequence(
        Commands.runOnce(() -> currentNoteState = noteState.Outtake),
        Commands.runOnce(() -> readyToShoot = true),
        Commands.runOnce(() -> currentScoringMode = ScoringMode.Amp),
        ampMode());
  }

  public Command choreoAuto(String autoName){
    return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(autoName)).beforeStarting(Commands.runOnce(() -> drivebase.setPose(Choreo.getTrajectory(autoName).getInitialPose())));
  }

}
