package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MAXSwerveConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class MAXSwerve extends SubsystemBase {

  // Gryo IO
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // Swerve Kinematics
  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.kModuleLocations);

  // Odometry
  private SwerveDrivePoseEstimator poseEstimator;

  // Array of swerve modules
  private final MAXSwerveModule[] modules;

  // Track the last heading for when the gyro is disconnected/simulated
  private Rotation2d lastHeading = new Rotation2d();

  /**
   * Creates a new MAXSwerve drivebase
   *
   * @param gyroIO GyroIO
   * @param MAXSwerveIOs Array of MAXSwerveIOs
   */
  public MAXSwerve(GyroIO gyroIO, MAXSwerveIO[] MAXSwerveIOs) {

    // Set gyroIO
    this.gyroIO = gyroIO;

    // Create array of swerve modules
    modules = new MAXSwerveModule[MAXSwerveIOs.length];

    // Create a swerve module for each MAXSwerveIO
    for (int i = 0; i < MAXSwerveIOs.length; i++) {
      modules[i] =
          new MAXSwerveModule(
              MAXSwerveIOs[i], DriveConstants.kIndexedSwerveModuleInformation[i].name + " Module");
    }

    // Create the pose estimator
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, gyroInputs.yawPosition, getModulePositions(), new Pose2d());
  }

  /** This code runs at 50hz and is responsible for updating the IO and pose estimator */
  @Override
  public void periodic() {

    lastHeading = getPose().getRotation();

    // Update Swerve Module Inputs
    for (var module : modules) {
      module.updateInputs();
    }
    gyroIO.updateInputs(gyroInputs);

    Logger.processInputs("Gyro", gyroInputs);

    var gyroDelta =
        new Rotation2d(
            kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
                * 1
                / CodeConstants.kMainLoopFrequency);

    lastHeading = lastHeading.plus(gyroDelta);

    if (gyroInputs.connected) {
      poseEstimator.update(gyroInputs.yawPosition, getModulePositions());
    } else {
      poseEstimator.update(lastHeading, getModulePositions());
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }
  }

  /**
   * Runs the drivebase using a continuous Chassis Speed Input
   *
   * @param speeds Continuous Chassis Speed Input
   * @return Command that runs the drivebase
   */
  public Command runVelocity(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          this.runChassisSpeeds(speeds.get());
        });
  }

  /*
   * Stops the drivebase
   */
  public Command stop() {
    return runVelocity(() -> new ChassisSpeeds());
  }

  public void choreoStop() {
    for (int i = 0; i < 4; i++) {
      modules[i].stop();
    }
  }

  /**
   * Runs the drivebase using a continuous Chassis Speed Input in field relative mode
   *
   * @param speeds Continuous Chassis Speed Input
   * @return Command that runs the drivebase in field relative mode
   */
  public Command runVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.runVelocity(
        () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getPose().getRotation()));
  }

  /**
   * Runs the drivebase directly using a chassis speed input
   *
   * @param speeds desired chassis speed
   */
  public void runChassisSpeeds(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, 1 / CodeConstants.kMainLoopFrequency);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAXSwerveConstants.kMaxDriveSpeed);

    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

    for (int i = 0; i < modules.length; i++) {
      optimizedSetpointStates[i] = modules[i].run(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getSwerveModuleState();
    }
    return states;
  }

  // Returns the distance and angle of each module
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getSwerveModulePosition();
    }
    return positions;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    if (RobotBase.isReal()) {
      poseEstimator.resetPosition(gyroInputs.yawPosition, getModulePositions(), pose);
    } else {
      poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
    }
  }

  public Command goToShotPoint() {

    var xController = new PIDController(AutoConstants.kAA_P_X, 0, 0);
    var yController = new PIDController(AutoConstants.kAA_P_Y, 0, 0);
    var thetaController = new PIDController(AutoConstants.kAA_P_Theta, 0, 0);

    xController.setTolerance(0.02);
    yController.setTolerance(0.02);
    thetaController.setTolerance(Units.degreesToRadians(1));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return this.run(
            () -> {

              boolean isBlue = true;

              if(DriverStation.getAlliance().isPresent()){
                isBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
              }

              // Translation2d speakerCenter = (DriverStation.getAlliance().get() == Alliance.Blue)
              // ?
              // new Translation2d(0.2167, 5.549) : new Translation2d(16.3, 5.549);
              Translation2d speakerCenter =
                  (isBlue) ? new Translation2d(0.2167, 5.549) : new Translation2d(16.3, 5.549);

              double shotDistance = 1.75; // meters

              // Draw a circle of points around the speaker to visualize the shot line
              Pose2d[] speakerCirclePoints = new Pose2d[360];
              for (int i = 0; i < 360; i++) {
                double angle = Math.toRadians(i / 360d) * 360;
                speakerCirclePoints[i] =
                    new Pose2d(
                        new Translation2d(
                                shotDistance * Math.cos(angle), shotDistance * Math.sin(angle))
                            .plus(speakerCenter),
                        new Rotation2d());
              }

              double distanceFromShotline =
                  getPose().getTranslation().getDistance(speakerCenter) - shotDistance;

              // Draw circle of points around the robot
              Pose2d[] robotCirclePoints = new Pose2d[360];
              for (int i = 0; i < 360; i++) {
                double angle = Math.toRadians(i / 360d) * 360;
                robotCirclePoints[i] =
                    new Pose2d(
                        new Translation2d(
                                distanceFromShotline * Math.cos(angle),
                                distanceFromShotline * Math.sin(angle))
                            .plus(getPose().getTranslation()),
                        new Rotation2d());
              }

              Logger.recordOutput("AutoAim/ShotLine", speakerCirclePoints);
              Logger.recordOutput("AutoAim/RobotCircle", robotCirclePoints);
              Logger.recordOutput(
                  "AutoAim/Directline",
                  new Pose2d[] {getPose(), new Pose2d(speakerCenter, new Rotation2d())});

              double vX = getPose().getX() - speakerCenter.getX();
              double vY = getPose().getY() - speakerCenter.getY();
              double magV = Math.sqrt(vX * vX + vY * vY);
              double aX = speakerCenter.getX() + vX / magV * shotDistance;
              double aY = speakerCenter.getY() + vY / magV * shotDistance;

              double m = (aY - getPose().getY()) / (aX - getPose().getX());
              double angle = Math.atan(m);

              Pose2d desiredPos = new Pose2d(aX, aY, new Rotation2d(angle).plus(new Rotation2d(isBlue ? 0 : Math.PI)));

              Logger.recordOutput("AutoAim/DesiredShotPos", desiredPos);

              var xSpeed =
                  xController.calculate(
                      getPose().getTranslation().getX(), desiredPos.getTranslation().getX());
              var ySpeed =
                  yController.calculate(
                      getPose().getTranslation().getY(), desiredPos.getTranslation().getY());
              var thetaSpeed =
                  thetaController.calculate(
                      getPose().getRotation().getRadians(), desiredPos.getRotation().getRadians());

              this.runChassisSpeeds(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      new ChassisSpeeds(
                          MathUtil.clamp(xSpeed, -2, 2), MathUtil.clamp(ySpeed, -2, 2), thetaSpeed),
                      getPose().getRotation()));
            })
        .beforeStarting(
            () -> {
              xController.reset();
              yController.reset();
              thetaController.reset();
            })
        .until(
            () ->
                (xController.atSetpoint()
                    && yController.atSetpoint()
                    && thetaController.atSetpoint()));
  }

  @SuppressWarnings("resource")
  public Command goToPose(Pose2d targetPose) {

    var xController = new PIDController(AutoConstants.kAA_P_X, 0, 0);
    var yController = new PIDController(AutoConstants.kAA_P_Y, 0, 0);
    var thetaController = new PIDController(AutoConstants.kAA_P_Theta, 0, 0);

    xController.setTolerance(0.02);
    yController.setTolerance(0.02);
    thetaController.setTolerance(Units.degreesToRadians(1));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return this.run(
            () -> {
              Logger.recordOutput("GoToLocation", targetPose);

              var xSpeed =
                  xController.calculate(
                      getPose().getTranslation().getX(), targetPose.getTranslation().getX());
              var ySpeed =
                  yController.calculate(
                      getPose().getTranslation().getY(), targetPose.getTranslation().getY());
              var thetaSpeed =
                  thetaController.calculate(
                      getPose().getRotation().getRadians(), targetPose.getRotation().getRadians());

              this.runChassisSpeeds(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      new ChassisSpeeds(
                          MathUtil.clamp(xSpeed, -2, 2), MathUtil.clamp(ySpeed, -2, 2), thetaSpeed),
                      getPose().getRotation()));
            })
        .beforeStarting(
            () -> {
              xController.reset();
              yController.reset();
              thetaController.reset();
            })
        .until(
            () ->
                (xController.atSetpoint()
                    && yController.atSetpoint()
                    && thetaController.atSetpoint()));
  }

  public void runTowardsPose(Pose2d pose) {}
}
