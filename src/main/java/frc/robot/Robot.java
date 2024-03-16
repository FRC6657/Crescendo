package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants.ClimberInformation;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.MAXSwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.noteState;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimberIO;
import frc.robot.subsystems.climb.ClimberIO_Real;
import frc.robot.subsystems.climb.ClimberIO_Sim;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIO_Real;
import frc.robot.subsystems.drive.MAXSwerve;
import frc.robot.subsystems.drive.MAXSwerveIO;
import frc.robot.subsystems.drive.MAXSwerveIO_Real;
import frc.robot.subsystems.drive.MAXSwerveIO_Sim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO_Real;
import frc.robot.subsystems.intake.IntakeIO_Sim;
import frc.robot.subsystems.led.*;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO_Real;
import frc.robot.subsystems.outtake.OuttakeIO_Sim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.NoteVisualizer;
import java.io.IOException;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  Vision vision =
      new Vision(
          VisionConstants.kBackCameraInfo,
          VisionConstants.kSideCameraInfo,
          VisionConstants.kNoteCameraName);

  public static enum RobotMode {
    SIM,
    REPLAY,
    REAL
  }

  // Control the mode of the robo
  public static final RobotMode mode = Robot.isReal() ? RobotMode.REAL : RobotMode.SIM;
  // public static final RobotMode mode = RobotMode.REPLAY;

  // Auto Command
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  private Command autoCommand;

  // Driver Controllers
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandGenericHID operator = new CommandGenericHID(1);
  private CommandGenericHID debug = new CommandGenericHID(5);

  // Subsystems
  private MAXSwerve drivebase =
      new MAXSwerve(
          mode == RobotMode.REAL ? new GyroIO_Real() : new GyroIO() {},
          mode == RobotMode.REAL
              ? new MAXSwerveIO[] {
                new MAXSwerveIO_Real(DriveConstants.kFrontLeftSwerveModule),
                new MAXSwerveIO_Real(DriveConstants.kFrontRightSwerveModule),
                new MAXSwerveIO_Real(DriveConstants.kBackLeftSwerveModule),
                new MAXSwerveIO_Real(DriveConstants.kBackRightSwerveModule)
              }
              : new MAXSwerveIO[] {
                new MAXSwerveIO_Sim(),
                new MAXSwerveIO_Sim(),
                new MAXSwerveIO_Sim(),
                new MAXSwerveIO_Sim()
              });

  private Climb climb =
      new Climb(
          mode == RobotMode.REAL
              ? new ClimberIO[] {
                new ClimberIO_Real(ClimberInformation.kLeftClimber),
                new ClimberIO_Real(ClimberInformation.kRightClimber)
              }
              : new ClimberIO[] {new ClimberIO_Sim(), new ClimberIO_Sim()});

  private Outtake outtake =
      new Outtake(mode == RobotMode.REAL ? new OuttakeIO_Real() : new OuttakeIO_Sim());

  private Intake intake =
      new Intake(mode == RobotMode.REAL ? new IntakeIO_Real() : new IntakeIO_Sim());

  private LEDs led = new LEDs();

  private Superstructure superstructure =
      new Superstructure(drivebase, intake, outtake, climb, led);

  private Alliance currentAlliance = Alliance.Blue;

  @SuppressWarnings(value = "resource")
  @Override
  public void robotInit() {
    Logger.recordMetadata("Codebase", "6657 2024");
    switch (mode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        break;
    }

    Logger.start();

    autoChooser.addDefaultOption("None", null);
    autoChooser.addOption("CenF-S0", superstructure.CenFS0());
    autoChooser.addOption("CenF-S02", superstructure.CenFS02());
    autoChooser.addOption("CenF-S03", superstructure.CenFS03());
    autoChooser.addOption("AmpF-S0", superstructure.AmpFS0());
    // autoChooser.addOption("AmpF-S041", superstructure.AmpFS041());
    autoChooser.addOption("Accel Sysid", superstructure.accelSysid());
    autoChooser.addOption("CenF-S03214", superstructure.CenFS3214());

    NoteVisualizer.setRobotPoseSupplier(drivebase::getPose);

    // Set the default command for the drivebase for TeleOP driving
    drivebase.setDefaultCommand(
        drivebase.runVelocityFieldRelative(
            () ->
                new ChassisSpeeds(
                    -MathUtil.applyDeadband(driver.getLeftY(), 0.05)
                        * MAXSwerveConstants.kMaxDriveSpeed
                        * 0.7,
                    -MathUtil.applyDeadband(driver.getLeftX(), 0.05)
                        * MAXSwerveConstants.kMaxDriveSpeed
                        * 0.7,
                    -MathUtil.applyDeadband(driver.getRightX(), 0.05)
                        * DriveConstants.kMaxAngularVelocity
                        * 0.4)));

    driver
        .a()
        .whileTrue(
            Commands.either(
                drivebase.noteAim(
                    driver::getLeftY, driver::getLeftX, driver::getRightX, vision::getNoteX),
                Commands.either(
                        drivebase.goToShotPoint(),
                        drivebase.goToAmpPoint(),
                        superstructure::inSpeakerMode)
                    .andThen(superstructure.shootPiece()),
                intake::extended));

    driver
        .rightTrigger()
        .onTrue(superstructure.extendIntake())
        .onFalse(superstructure.retractIntake());
    driver.leftTrigger().onTrue(superstructure.shootPiece());

    operator.button(1).onTrue(superstructure.ampMode());
    operator.button(2).onTrue(superstructure.speakerMode());
    operator.button(3).onTrue(superstructure.readyPiece());
    operator.button(6).onTrue(superstructure.raiseClimbers());
    operator.button(7).onTrue(superstructure.lowerClimbers());

    operator.button(9).onTrue(superstructure.firstReset());
    operator.button(9).onFalse(superstructure.secondReset());

    operator.button(4).onTrue(new InstantCommand(led::amplifySignal));

    debug
        .button(1)
        .onTrue(Commands.runOnce(() -> superstructure.overrideNoteState(noteState.Intake)));
    debug
        .button(2)
        .onTrue(Commands.runOnce(() -> superstructure.overrideNoteState(noteState.Outtake)));
    led.changeColor(LEDConstants.kDisabledColor);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    superstructure.update3DPose();

    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() != currentAlliance) {
        currentAlliance = DriverStation.getAlliance().get();
        try {
          vision.setFieldTags(currentAlliance);
        } catch (IOException e) {
          e.printStackTrace();
        }
      }
    }

    var backResult = vision.getBackCameraResult();
    var sideResult = vision.getSideCameraResult();

    if (backResult.timestamp != 0.0) {
      Logger.recordOutput("Vision/BackGlobalEstimate", backResult.estimatedPose);
      if (RobotBase.isReal()) {
        drivebase.addVisionMeasurement(
            backResult.estimatedPose, backResult.timestamp, backResult.stdDevs);
      }
    }

    if (sideResult.timestamp != 0.0) {
      Logger.recordOutput("Vision/SideGlobalEstimate", sideResult.estimatedPose);
      if (RobotBase.isReal()) {
        drivebase.addVisionMeasurement(
            sideResult.estimatedPose, sideResult.timestamp, sideResult.stdDevs);
      }
    }
  }

  @Override
  public void teleopInit() {
    led.changeColor(LEDConstants.kEnabledColor);
  }

  @Override
  public void disabledInit() {
    Commands.sequence(led.disableBlinkMode(), led.changeColorCommand(LEDConstants.kDisabledColor))
        .schedule();
  }

  @Override
  public void autonomousInit() {

    led.changeColor(LEDConstants.kEnabledColor);

    autoCommand = autoChooser.get();

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousExit() {
    if (autoChooser.get() != null) {
      CommandScheduler.getInstance().cancel(autoChooser.get());
      CommandScheduler.getInstance().schedule(drivebase.stop());
    }
  }

  @Override
  public void simulationPeriodic() {
    vision.simulationPeriodic(drivebase.getPose());
  }
}
