package frc.robot;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.ClimberInformation;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MAXSwerveConstants;
import frc.robot.Constants.OuttakeConstants;
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
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

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

  // Driver Controllers
  private CommandXboxController controller = new CommandXboxController(0);
  private CommandGenericHID operator = new CommandGenericHID(1);

  private Trigger noteDetected;
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

  ChoreoTrajectory TestPath1;

  // Trigger stopTrigger = new Trigger(outtake::beamBroken).onTrue(outtake.changeRPMSetpoint(0));

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

    noteDetected = new Trigger(intake::noteDetected);
    noteDetected.onTrue(intake.changeRollerSpeed(0));

    // Driver Controls
    drivebase.setDefaultCommand(
        drivebase.runVelocityFieldRelative(
            () ->
                new ChassisSpeeds(
                    -MathUtil.applyDeadband(controller.getLeftY(), 0.05)
                        * MAXSwerveConstants.kMaxDriveSpeed
                        * 0.5,
                    -MathUtil.applyDeadband(controller.getLeftX(), 0.15)
                        * MAXSwerveConstants.kMaxDriveSpeed
                        * 0.5,
                    -MathUtil.applyDeadband(controller.getRightX(), 0.15)
                        * DriveConstants.kMaxAngularVelocity
                        * 0.25)));

    // Floor Pickup
    controller
        .a()
        .onTrue(
            new SequentialCommandGroup(
                intake.changePivotSetpoint(IntakeConstants.kMinPivotAngle),
                intake.changeRollerSpeed(IntakeConstants.kFloorInSpeed)))
        .onFalse(
            new SequentialCommandGroup(
                intake.changePivotSetpoint(IntakeConstants.kMaxPivotAngle),
                intake.changeRollerSpeed(0)));

    // Fire Amp
    controller
        .x()
        .onTrue(
            new SequentialCommandGroup(
                outtake.changeRPMSetpoint(300),
                new WaitUntilCommand(outtake::beamBroken),
                outtake.changeRPMSetpoint(0),
                outtake.changePivotSetpoint(96),
                new WaitCommand(1),
                outtake.changeRPMSetpoint(1000),
                new WaitUntilCommand(() -> !outtake.beamBroken()),
                outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle),
                outtake.changeRPMSetpoint(0)));

    // Fire Speaker
    controller
        .y()
        .onTrue(outtake.changeRPMSetpoint(OuttakeConstants.kMaxFlywheelRpm))
        .onFalse(outtake.changeRPMSetpoint(0));

    controller.b().onTrue(intake.changeRollerSpeed(-0.6)).onFalse(intake.changeRollerSpeed(0));

    controller.leftTrigger().onTrue(outtake.changeRPMSetpoint(600));
    controller.leftTrigger().onFalse(outtake.changeRPMSetpoint(0));

    controller
        .rightTrigger()
        .onTrue(intake.changeRollerSpeed(-0.4))
        .onFalse(intake.changeRollerSpeed(0));

    controller.rightBumper().onTrue(intake.changePivotSetpoint(IntakeConstants.kMaxPivotAngle));
    controller.leftBumper().onTrue(intake.changePivotSetpoint(IntakeConstants.kMinPivotAngle));

    // Operator Controls

    // Load Shooter for Amp
    operator
        .button(1)
        .onTrue(
            new SequentialCommandGroup(
                intake.changeRollerSpeed(-0.6),
                outtake.changeRPMSetpoint(300),
                new WaitUntilCommand(outtake::beamBroken),
                intake.changeRollerSpeed(0),
                outtake.changeRPMSetpoint(0),
                outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle)));

    // Ready Amp
    operator.button(2).onTrue(outtake.changePivotSetpoint(96));

    // Un-Ready Amp or Pivot Home
    operator.button(3).onTrue(outtake.changePivotSetpoint(OuttakeConstants.kMinPivotAngle));

    // Man intake
    operator.button(4).onTrue(intake.changeRollerSpeed(IntakeConstants.kFloorInSpeed));

    // Climbers Up
    operator.button(5).onTrue(climb.changeSetpoint(ClimbConstants.kMaxHeight));

    // Climbers Down
    operator.button(6).onTrue(climb.changeSetpoint(ClimbConstants.kMinHeight + 0.1));

    // Man feed
    operator.button(7).onTrue(intake.changeRollerSpeed(-0.8)).onFalse(intake.changeRollerSpeed(0));

    autoChooser.addDefaultOption("None", null);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    superstructure.update3DPose();
    superstructure.processQueue();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (autoChooser.get() != null) {
      CommandScheduler.getInstance().schedule(autoChooser.get());
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
    // noteVisulaizer.updateNotes();
  }
}
