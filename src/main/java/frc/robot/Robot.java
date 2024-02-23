package frc.robot;

import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MAXSwerveConstants;
import frc.robot.Constants.OuttakeConstants;
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
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  Vision vision = new Vision();


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

    // Set the default command for the drivebase for TeleOP driving
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

    autoChooser.addDefaultOption("None", null);

    // controller.b().onTrue(outtake.changePivotSetpoint(OuttakeConstants.kMaxAngle));
    // controller.b().onFalse(outtake.changePivotSetpoint(OuttakeConstants.kMinAngle));

    controller.a().onTrue(outtake.changeRPMSetpoint(2300));
    controller.a().onFalse(outtake.changeRPMSetpoint(0));
    // controller.a().onTrue(
    //   new SequentialCommandGroup(
    //     outtake.changeRPMSetpoint(300),
    //     new WaitUntilCommand(outtake::beamBroken),
    //     outtake.changeRPMSetpoint(0),
    //     outtake.changePivotSetpoint(96),
    //     new WaitCommand(1),
    //     outtake.changeRPMSetpoint(1000),
    //     new WaitUntilCommand(() -> !outtake.beamBroken()),
    //     outtake.changePivotSetpoint(OuttakeConstants.kMinAngle),
    //     outtake.changeRPMSetpoint(0)
    //   )
    //   );

    // controller.a().onFalse(
    // new SequentialCommandGroup(

    // )
    // );
    
    controller.leftTrigger().onTrue(outtake.changeRPMSetpoint(600));
    controller.leftTrigger().onFalse(outtake.changeRPMSetpoint(0));


    controller.b().onTrue(intake.changePivotSetpoint(IntakeConstants.kMinPivotAngle));
    controller.x().onTrue(intake.changePivotSetpoint(IntakeConstants.kMaxPivotAngle));
    controller.y().onTrue(outtake.changePivotSetpoint(OuttakeConstants.kMaxPivotAngle));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    superstructure.update3DPose();
    superstructure.processQueue();
   
    var backVisionEst = vision.getBackEstimatedGlobalPose();
    var sideVisionEst = vision.getSideEstimatedGlobalPose();

    backVisionEst.ifPresent(
      est -> {
        var estPose = est.estimatedPose.toPose2d();
        Logger.recordOutput("Vision/BackGlobalEstimate", estPose);

        // var estStdDevs = vision.getBackEstimationStdDevs(estPose);
        // drivebase.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);

      }
    );

    sideVisionEst.ifPresent(
      est -> {
        var estPose = est.estimatedPose.toPose2d();
        Logger.recordOutput("Vision/SideGlobalEstimate", estPose);

        // var estStdDevs = vision.getSideEstimationStdDevs(estPose);
        // drivebase.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);
      }
    );

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
    vision.simulationPeriodic(drivebase.getPose());
  }
}
