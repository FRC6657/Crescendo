// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.MAXSwerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LEDs;
import frc.robot.subsystems.outtake.Outtake;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  MAXSwerve drivebase;
  Intake intake;
  Outtake outtake;
  Climb climb;
  LEDs leds;
  Trigger noteDetector;

  // Enum for tracking the position of the note
  public enum noteState {
    Processing,
    Intake,
    Outtake,
    None
  };

  @AutoLogOutput(key = "Superstructure/Note State")
  private noteState currentNoteState = noteState.None;

  // Enum for tracking the robot scoring mode
  private enum ScoringMode {
    Amp,
    Speaker
  };

  @AutoLogOutput(key = "Superstructure/Scoring Mode")
  private ScoringMode currentScoringMode = ScoringMode.Speaker;

  @AutoLogOutput(key = "Superstructure/Ready")
  private boolean readyToShoot = false;

  private boolean climbersUp = false;

  public Superstructure(
      MAXSwerve drivebase, Intake intake, Outtake outtake, Climb climb, LEDs leds) {

    this.drivebase = drivebase;
    this.intake = intake;
    this.outtake = outtake;
    this.climb = climb;
    this.leds = leds;

    // Automatically run process note when the note is detected, but only in teleop
    noteDetector = new Trigger(() -> ((intake.noteDetected() && DriverStation.isTeleop()) && !intake.tofUnplugged()));
    noteDetector.onTrue(processNote());

    // Seed the latest event key
    Logger.recordOutput("Superstructure/Latest Event", "Superstructure Initialized");
  }

  // Method for updating the 3D poses of the mechanisms from their current positions.
  // This is used to visualize the robot in 3D with Advantage Scope
  public void update3DPose() {
    Pose3d[] mechanismPoses = new Pose3d[4];
    mechanismPoses[0] = outtake.get3DPose();
    mechanismPoses[1] = intake.get3DPose();
    mechanismPoses[2] = climb.get3DPoses()[0];
    mechanismPoses[3] = climb.get3DPoses()[1];
    Logger.recordOutput("3D Poses", mechanismPoses);
  }

  // Shorthand method for keeping track of what the superstructure is doing
  public Command logEvent(String event) {
    return Commands.runOnce(() -> Logger.recordOutput("Superstructure/Latest Event", event));
  }

  // Command to process the note after it has been intook.
  // This command is uninterruptible, as it is a critical process
  public Command processNote() {
    return Commands.sequence(
            leds.changeColorCommand(LEDConstants.kProcessingColor),
            Commands.runOnce(() -> currentNoteState = noteState.Processing),
            retractIntake(),
            Commands.waitUntil(intake::atSetpoint),
            chamberNote(),
            relocateNote(),
            leds.changeColorCommand(LEDConstants.kEnabledColor))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  // Command to raise the climbers.
  // This command will also lower the outtake if it is raised
  // This is to prevent collisions between subsystems
  public Command raiseClimbers() {
    return stowOuttake()
        .onlyIf(() -> (readyToShoot && currentScoringMode == ScoringMode.Amp))
        .andThen(Commands.runOnce(() -> readyToShoot = false))
        .beforeStarting(logEvent("Raising Climbers"))
        .andThen(
            Commands.sequence(
                Commands.waitUntil(outtake::atPivotSetpoint),
                climb.changeSetpoint(ClimbConstants.kMaxHeight - 0.5),
                leds.enableBlinkMode(),
                Commands.runOnce(() -> climbersUp = true)));
  }

  // Command to lower the climbers
  public Command lowerClimbers() {
    return Commands.sequence(
        logEvent("Lowering Climbers"),
        climb.changeSetpoint(0.1),
        Commands.waitUntil(climb::atSetpoint),
        Commands.runOnce(() -> climbersUp = false),
        leds.disableBlinkMode());
  }

  // Command to stow the outtake
  public Command stowOuttake() {
    return Commands.sequence(
        logEvent("Stowing Outtake"),
        outtake.changeRPMSetpoint(0),
        outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle));
  }

  // Command to raise the outtake
  // This command will also lower the climbers if they are raised
  public Command raiseOuttake() {
    return Commands.sequence(
        logEvent("Raising Outtake"),
        lowerClimbers().onlyIf(() -> climbersUp).andThen(Commands.waitUntil(() -> !climbersUp)),
        outtake.changePivotSetpoint(OuttakeConstants.kMaxPivotAngle),
        Commands.waitUntil(outtake::atPivotSetpoint));
  }

  // Command to extend the intake
  public Command extendIntake() {
        return Commands.sequence(
            logEvent("Extending Intake"),
            intake.changePivotSetpoint(IntakeConstants.kMinPivotAngle),
            intake.changeRollerSpeed(IntakeConstants.kGroundIntakeSpeed));
  }

  // Command to retract the intake
  public Command retractIntake() {
    return Commands.sequence(
      logEvent("Retracting Intake"),
      intake.changeRollerSpeed(0),
      intake.changePivotSetpoint(IntakeConstants.kMaxPivotAngle));
  }

  // Command to return the robot to its default position
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

  // Command to move the note to the desired position
  // The desired position is dependant on the current scoring mode
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
                Commands.waitSeconds(0.1),
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

  // Readys the robot to shoot the current piece
  // The behavior of this command is dependant on the current scoring mode
  public Command readyPiece() {

    Command[] commands = new Command[2];

    commands[0] =
        Commands.sequence(
                logEvent("Readying Robot for Amp"),
                outtake.changePivotSetpoint(OuttakeConstants.kPivotAmpAngle),
                leds.enableBlinkMode(),
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
                leds.enableBlinkMode(),
                Commands.runOnce(() -> readyToShoot = true))
            .onlyIf(
                () ->
                    currentScoringMode == ScoringMode.Speaker
                        && currentNoteState == noteState.Intake);

    return Commands.sequence(commands);
  }

  // Command to move the note from the intake to the outtake
  // This command is uninterruptible, as it is a critical process
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
        .onlyIf(() -> intake.hasNote() || intake.tofUnplugged())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  // Command to shoot the current piece
  // The behavior of this command is dependant on the current scoring mode
  public Command shootPiece() {

    Command[] commands = new Command[3];

    commands[0] = readyPiece().onlyIf(() -> !readyToShoot);

    commands[1] =
        Commands.sequence(
                logEvent("Scoring Amp"),
                Commands.waitUntil(outtake::atPivotSetpoint),
                outtake.changeRPMSetpoint(OuttakeConstants.kAmpRPM),
                Commands.waitUntil(() -> !outtake.beamBroken()).unless(RobotBase::isSimulation),
                outtake.changeRPMSetpoint(0),
                outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle),
                Commands.runOnce(() -> currentNoteState = noteState.None),
                Commands.runOnce(() -> readyToShoot = false),
                leds.disableBlinkMode())
            .onlyIf(
                () ->
                    (currentScoringMode == ScoringMode.Amp
                        && currentNoteState == noteState.Outtake));

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
                Commands.runOnce(() -> currentNoteState = noteState.None),
                leds.disableBlinkMode())
            .onlyIf(
                () ->
                    (currentScoringMode == ScoringMode.Speaker
                        && currentNoteState == noteState.Intake));

    return Commands.sequence(commands);
  }

  // Command to switch the robot to speaker mode
  // This command will also relocate the note if it is in the wrong position
  public Command speakerMode() {
    return Commands.sequence(
            Commands.runOnce(() -> currentScoringMode = ScoringMode.Speaker), relocateNote())
        .unless(() -> currentScoringMode == ScoringMode.Speaker);
  }

  // Command to switch the robot to amp mode
  // This command will also relocate the note if it is in the wrong position
  public Command ampMode() {
    return Commands.sequence(
            Commands.runOnce(() -> currentScoringMode = ScoringMode.Amp), relocateNote())
        .unless(() -> currentScoringMode == ScoringMode.Amp);
  }

  // Command to run a path with the intake extended
  // The path will end when the note is detected, or when it reaches the end
  public Command intakePath(String pathName, boolean waitForIntake) {
    return Commands.sequence(
        extendIntake(),
        Commands.waitUntil(intake::atSetpoint).onlyIf(() -> waitForIntake),
        Commands.race(
            Commands.waitUntil(intake::noteDetected),
            Commands.sequence(runChoreoPath(pathName), retractIntake())));
  }

  // Command to run a path with the intake extending and retracting along the way
  // The intake retracts early and processes the note if it is detected
  public Command intakePath(
      String pathName, double intakeExtendSecond, double intakeRetractSecond) {
    return Commands.parallel(
        runChoreoPath(pathName),
        Commands.sequence(
            Commands.waitSeconds(intakeExtendSecond),
            extendIntake(),
            Commands.sequence(
                Commands.sequence(Commands.waitSeconds(intakeRetractSecond), retractIntake())
                    .until(intake::noteDetected),
                processNote().unless(intake::pivotSetpointIsMax))));
  }

  public Command intakePath(String pathName, double intakeRetractSecond) {
    return Commands.sequence(
        extendIntake(),
        Commands.waitUntil(intake::atSetpoint),
        Commands.parallel(
            runChoreoPath(pathName),
            Commands.sequence(
                Commands.sequence(Commands.waitSeconds(intakeRetractSecond), retractIntake())
                    .until(intake::noteDetected),
                processNote().unless(intake::pivotSetpointIsMax))));
  }

  // The first step in fully reseting the robot's current state.
  // This will cancel all commands currently running even if they are "uninterruptible"
  public Command firstReset() {
    return Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll());
  }

  // The second step in fully reseting the robot's current state.
  public Command secondReset() {
    return Commands.sequence(
        zeroRobot(),
        Commands.runOnce(() -> currentScoringMode = ScoringMode.Speaker),
        Commands.runOnce(() -> currentNoteState = noteState.None),
        Commands.runOnce(() -> readyToShoot = false));
  }

  // Command for overriding the current note state
  // Useful for debugging in simulation
  public void overrideNoteState(noteState state) {
    currentNoteState = state;
  }

  // Boolean value for current alliance color
  private boolean isRed() {
    boolean isRed = false;
    if (DriverStation.getAlliance().isPresent()) {
      isRed = (DriverStation.getAlliance().get() == Alliance.Red);
    }
    return isRed;
  }

  public boolean inSpeakerMode() {
    return currentScoringMode == ScoringMode.Speaker;
  }

  // Command for running a choreo trajectory
  private Command runChoreoPath(String pathName, boolean resetPose) {

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
            () -> resetPose);

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
                () ->
                    Logger.recordOutput(
                        "Superstructure/CurrentPath",
                        isRed() ? traj.flipped().getPoses() : traj.getPoses())),
            setPoseCommand,
            swerveCommand,
            Commands.runOnce(() -> drivebase.stop(), drivebase))
        .handleInterrupt(() -> drivebase.stop());
  }

  // Autonomous Stuff
  public Command runChoreoPath(String pathName) {
    return runChoreoPath(pathName, false);
  }

  // Convinience command for starting the autonomous
  // This will seed the robot's pose and shoot the robot's preloaded piece
  public Command autoStart(Pose2d bluePos, Pose2d redPos) {
    return Commands.sequence(
        Commands.runOnce(() -> drivebase.setPose(isRed() ? redPos : bluePos)),
        Commands.runOnce(() -> currentNoteState = noteState.Intake),
        drivebase.goToShotPoint().alongWith(readyPiece()),
        shootPiece());
  }

  public Command SouFS0() {
    return Commands.sequence(
        autoStart(AutoConstants.BLUE_SOURCE_FENDER, AutoConstants.RED_SOURCE_FENDER));
  }

  public Command SouFS0145() {
    return Commands.sequence(
      SouFS0(),
      intakePath("SouF-S0143.1", true),
      Commands.parallel(
            processNote().andThen(readyPiece()).onlyIf(() -> (intake.hasNote() || intake.tofUnplugged())),
            drivebase.goToShotPoint()),
      shootPiece(),
      intakePath("SouF-S0143.2", true),
      Commands.parallel(
            processNote().andThen(readyPiece()).onlyIf(() -> (intake.hasNote() || intake.tofUnplugged())),
            drivebase.goToShotPoint()),
      shootPiece(),
      intakePath("SouF-S0143.3", true),
      Commands.parallel(
            processNote().andThen(readyPiece()).onlyIf(() -> (intake.hasNote() || intake.tofUnplugged())),
            drivebase.goToShotPoint()),
      shootPiece(),
      intakePath("SouF-S0143.4", 1, 1.7),
      Commands.parallel(
            processNote().andThen(readyPiece()).onlyIf(() -> (intake.hasNote() || intake.tofUnplugged())),
            drivebase.goToShotPoint()),
      shootPiece()
      
    );
  }

  public Command CenFS0() {
    return Commands.sequence(
        autoStart(AutoConstants.BLUE_CENTER_FENDER, AutoConstants.RED_CENTER_FENDER));
  }

  public Command CenFS02() {
    return Commands.sequence(
        CenFS0(),
        intakePath("CenF-S02", true),
        Commands.parallel(
            processNote().andThen(readyPiece()).onlyIf(() -> (intake.hasNote() || intake.tofUnplugged())),
            drivebase.goToShotPoint()),
        retractIntake(),
        shootPiece());
  }

  public Command CenFS03() {
    return Commands.sequence(
        CenFS0(),
        intakePath("CenF-S03", true),
        Commands.parallel(processNote().andThen(readyPiece()), drivebase.goToShotPoint()),
        shootPiece());
  }

  public Command CenFS3214() {
    return Commands.sequence(
        CenFS0(),
        intakePath("CenF-S03214.1", true),
        Commands.parallel(processNote().andThen(readyPiece()), drivebase.goToShotPoint()),
        shootPiece(),
        intakePath("CenF-S03214.2", true),
        Commands.parallel(processNote().andThen(readyPiece()), drivebase.goToShotPoint()),
        shootPiece(),
        intakePath("CenF-S03214.3", true),
        Commands.parallel(processNote().andThen(readyPiece()), drivebase.goToShotPoint()),
        shootPiece(),
        intakePath("CenF-S03214.4", 1, 1.7),
        drivebase.goToShotPoint(),
        shootPiece());
  }

  public Command AmpFS0() {
    return Commands.sequence(
        autoStart(AutoConstants.BLUE_AMP_FENDER, AutoConstants.RED_AMP_FENDER));
  }

  public Command AmpFS041() {
    return Commands.sequence(
        AmpFS0(),
        intakePath("AmpF-S041.1", 3, 6),
        drivebase.goToShotPoint(),
        shootPiece(),
        intakePath("AmpF-S041.2", 2),
        drivebase.goToShotPoint(),
        shootPiece());
  }

  // Sysid at home
  double maxVel = 3.5;
  double target = maxVel;
  SlewRateLimiter limiter = new SlewRateLimiter(5);
  Timer timer = new Timer();
  double previousVel = 0;

  public double getAccelCurveValue() {

    double vel = limiter.calculate(target);

    if (vel == maxVel) {
      timer.start();
    }

    if (timer.get() > 0.5) {
      timer.reset();
      target = 0;
    }

    Logger.recordOutput("Sysid/VelTarget", vel);
    Logger.recordOutput("Sysid/Vel", drivebase.getChassisSpeeds().vxMetersPerSecond);
    Logger.recordOutput("Sysid/Accel", (vel - previousVel) / CodeConstants.kMainLoopFrequency);
    Logger.recordOutput("Sysid/Battery Voltage", RobotController.getBatteryVoltage());

    return vel;
  }

  public Command accelSysid() {
    return drivebase
        .runVelocity(() -> new ChassisSpeeds(getAccelCurveValue(), 0, 0))
        .beforeStarting(
            () -> {
              limiter.reset(0);
              timer.reset();
            });
  }
}
