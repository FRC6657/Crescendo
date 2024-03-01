package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MAXSwerveConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.ClimbConstants.ClimberInformation;
import frc.robot.subsystems.Superstructure;
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
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO_Real;
import frc.robot.subsystems.outtake.OuttakeIO_Sim;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  Vision vision = new Vision(VisionConstants.kBackCameraInfo, VisionConstants.kSideCameraInfo);

  public static enum RobotMode {
    SIM,
    REPLAY,
    REAL
  }

  // Control the mode of the robot
  public static final RobotMode mode = Robot.isReal() ? RobotMode.REAL : RobotMode.SIM;
  // public static final RobotMode mode = RobotMode.REPLAY;

  // Auto Command
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  private Command autoCommand;

  // Driver Controllers
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandGenericHID operator = new CommandGenericHID(1);

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

  // private Led led = new Led();

  private Superstructure superstructure = new Superstructure(drivebase, intake, outtake, climb);

  Trigger stopTrigger = new Trigger(outtake::beamBroken).onTrue(outtake.changeRPMSetpoint(0));

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

    // Set the default command for the drivebase for TeleOP driving
    drivebase.setDefaultCommand(
        drivebase.runVelocityFieldRelative(
            () ->
                new ChassisSpeeds(
                    -MathUtil.applyDeadband(driver.getLeftY(), 0.05)
                        * MAXSwerveConstants.kMaxDriveSpeed
                        * 0.5,
                    -MathUtil.applyDeadband(driver.getLeftX(), 0.15)
                        * MAXSwerveConstants.kMaxDriveSpeed
                        * 0.5,
                    -MathUtil.applyDeadband(driver.getRightX(), 0.15)
                        * DriveConstants.kMaxAngularVelocity
                        * 0.25)));

    autoChooser.addDefaultOption("None", null);
    autoChooser.addDefaultOption("test", superstructure.testAuto());

    driver.povUp().whileTrue(drivebase.goToShotPoint());

    // Floor Pickup
    // driver
    //     .a()
    //     .whileTrue(
    //         new SequentialCommandGroup(
    //             intake.changePivotSetpoint(IntakeConstants.kMinPivotAngle),
    //             intake.changeRollerSpeed(IntakeConstants.kFloorInSpeed),
    //             Commands.waitUntil(() -> intake.noteDetected()),
    //             intake.changeRollerSpeed(IntakeConstants.kFloorInSpeed),
    //             intake.changePivotSetpoint(IntakeConstants.kMaxPivotAngle)))
    //     .whileFalse(
    //         new SequentialCommandGroup(
    //             intake.changePivotSetpoint(IntakeConstants.kMaxPivotAngle),
    //             intake.changeRollerSpeed(0)));

    driver.x().whileTrue(
      intake.changeRollerSpeed(IntakeConstants.kFloorInSpeed)
    ).onFalse(
      intake.changeRollerSpeed(0)
    );

    driver.y().whileTrue(
      intake.changeRollerSpeed(-0.5)
    ).onFalse(
      intake.changeRollerSpeed(0)
    );

    // Fire Amp
    // driver
    //     .x()
    //     .whileTrue(
    //         new SequentialCommandGroup(
    //             outtake.changeRPMSetpoint(300),
    //             new WaitUntilCommand(outtake::beamBroken),
    //             outtake.changeRPMSetpoint(0),
    //             outtake.changePivotSetpoint(96),
    //             outtake.waitUntilPivotAtSetpoint(),
    //             outtake.changeRPMSetpoint(1000),
    //             new WaitUntilCommand(() -> !outtake.beamBroken()),
    //             outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle),
    //             outtake.changeRPMSetpoint(0)));

    // Fire Speaker
    driver
        .b()
        .whileTrue(
            new SequentialCommandGroup(
                outtake.changeRPMSetpoint(OuttakeConstants.kMaxFlywheelRpm),
                outtake.waitUntilFlywheelAtSetpoint(),
                intake.changeRollerSpeed(-0.6)))
        .whileFalse(
            new SequentialCommandGroup(intake.changeRollerSpeed(0), outtake.changeRPMSetpoint(0)));

    // Ready Amp
    operator
        .button(1)
        .onTrue(
            new SequentialCommandGroup(
                intake.changeRollerSpeed(-0.6),
                outtake.changeRPMSetpoint(300),
                new WaitUntilCommand(outtake::beamBroken),
                outtake.changeRPMSetpoint(0),
                intake.changeRollerSpeed(0)));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    superstructure.update3DPose();

    var backResult = vision.getBackCameraResult();
    var sideResult = vision.getSideCameraResult();

    if (backResult.timestamp != 0.0) {
      Logger.recordOutput("Vision/BackGlobalEstimate", backResult.estimatedPose);
      if (RobotBase.isReal()) {
        // drivebase.addVisionMeasurement(backResult.estimatedPose, backResult.timestamp,
        // backResult.stdDevs);
      }
    }

    if (sideResult.timestamp != 0.0) {
      Logger.recordOutput("Vision/SideGlobalEstimate", sideResult.estimatedPose);
      if (RobotBase.isReal()) {
        // drivebase.addVisionMeasurement(sideResult.estimatedPose, sideResult.timestamp,
        // sideResult.stdDevs);
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    autoCommand = autoChooser.get();

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autoChooser.get() != null) {
      CommandScheduler.getInstance().cancel(autoChooser.get());
      CommandScheduler.getInstance().schedule(drivebase.stop());
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void simulationPeriodic() {
    vision.simulationPeriodic(drivebase.getPose());
  }
}
