package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static final class CodeConstants {
    public static final double kMainLoopFrequency = 50; // Hz
  }

  public static final class NotePositions {
    public static final Pose3d[] kNotesStartingMidline = {
      new Pose3d(8.258, 7.462, 0.03018, new Rotation3d()),
      new Pose3d(8.258, 5.785, 0.03018, new Rotation3d()),
      new Pose3d(8.258, 4.109, 0.03018, new Rotation3d()),
      new Pose3d(8.258, 2.432, 0.03018, new Rotation3d()),
      new Pose3d(8.258, 0.756, 0.03018, new Rotation3d()),
    };

    public static final Pose3d[] kNotesStartingBlueWing = {
      new Pose3d(2.884, 4.109, 0.03018, new Rotation3d()),
      new Pose3d(2.884, 5.557, 0.03018, new Rotation3d()),
      new Pose3d(2.884, 7.004, 0.03018, new Rotation3d()),
    };

    public static final Pose3d[] kNotesStartingRedWing = {
      new Pose3d(13.63, 4.109, 0.03018, new Rotation3d()),
      new Pose3d(13.63, 5.557, 0.03018, new Rotation3d()),
      new Pose3d(13.63, 7.004, 0.03018, new Rotation3d()),
    };
  }

  public static final class VisionConstants {
    public static final Transform3d kBackCameraPose =
        new Transform3d(
            new Translation3d(-0.343236, 0, 0.531201), new Rotation3d(0, -0.485314, Math.PI));
    public static final Transform3d kSideCameraPose =
        new Transform3d(
            new Translation3d(-0.246587, 0.172519, 0.509174),
            new Rotation3d(Math.PI, 0.122173 + Math.PI, -1.28248701081));

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.4, 0.4, 0.4);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.2);

    public static final class CameraInformation {
      public final String name;
      public final Transform3d cameraPose;

      public CameraInformation(String name, Transform3d cameraPose) {
        this.name = name;
        this.cameraPose = cameraPose;
      }
    }

    public static final CameraInformation kBackCameraInfo =
        new CameraInformation("Camera_Back", kBackCameraPose);
    public static final CameraInformation kSideCameraInfo =
        new CameraInformation("Camera_Side", kSideCameraPose);

    public static final String kNoteCameraName = "NoteCamera";

    public static final class CameraResult {
      public final Pose3d estimatedPose;
      public final Matrix<N3, N1> stdDevs;
      public final double timestamp;

      public CameraResult(Pose3d estPose, Matrix<N3, N1> stdDevs, double timestamp) {
        this.estimatedPose = estPose;
        this.stdDevs = stdDevs;
        this.timestamp = timestamp;
      }
    }
  }

  public static final class CANID {

    public static final int kPDH = 1;

    public static final int kFrontLeftDrive = 2;
    public static final int kBackLeftDrive = 4;
    public static final int kFrontRightDrive = 3;
    public static final int kBackRightDrive = 5;

    public static final int kFrontLeftTurn = 6;
    public static final int kBackLeftTurn = 8;
    public static final int kFrontRightTurn = 7;
    public static final int kBackRightTurn = 9;

    public static final int kPigeon = 10;

    public static final int kLeftFlywheel = 11;
    public static final int kRightFlywheel = 12;
    public static final int kShooterPivot = 13;

    public static final int kLeftClimber = 14;
    public static final int kRightClimber = 15;

    public static final int kIntakeRollers = 16;
    public static final int kIntakePivot = 17;
  }

  public static final class AutoConstants {

    // Choreo
    public static final PIDController kXController = new PIDController(9, 0, 0);
    public static final PIDController kYController = new PIDController(9, 0, 0);
    public static final PIDController kThetaController = new PIDController(8, 0, 0);

    // Auto Align
    public static final double kAA_P_X = 3.5;
    public static final double kAA_P_Y = 3.5;
    public static final double kAA_P_Theta = 8;

    public static final double kAA_T_Clamp = 3; // m/s
    public static final double kAA_R_Clamp = 2 * Math.PI; // rad/s

    public static final double kAA_T_Tolerance = Units.inchesToMeters(2); // m
    public static final double kAA_R_Tolerance = Units.degreesToRadians(2); // rad

    // Note Aim
    public static final double kNA_P = 1d/20;

    // Starting Positions
    public static final Pose2d BLUE_CENTER_FENDER =
        new Pose2d(new Translation2d(1.375, 5.55), new Rotation2d(0));
    public static final Pose2d RED_CENTER_FENDER =
        new Pose2d(new Translation2d(15.175, 5.55), new Rotation2d(Math.PI));
    public static final Pose2d BLUE_AMP_FENDER = new Pose2d(0.695, 6.735, new Rotation2d(1.05));
    public static final Pose2d RED_AMP_FENDER = new Pose2d(15.875, 6.735, new Rotation2d(2.091));
    public static final Pose2d BLUE_SOURCE_FENDER = new Pose2d(0.695, 4.385, new Rotation2d(-1.05));
    public static final Pose2d RED_SOURCE_FENDER =
        new Pose2d(15.875, 4.385, new Rotation2d(-2.091));
  }

  public static final class DriveConstants {

    public static final class SwerveModuleInformation {
      public final String name;
      public final int driveCAN_ID;
      public final int turnCAN_ID;
      public final Rotation2d moduleOffset;

      public SwerveModuleInformation(
          String name, int driveCAN_ID, int turnCAN_ID, Rotation2d moduleOffset) {
        this.name = name;
        this.driveCAN_ID = driveCAN_ID;
        this.turnCAN_ID = turnCAN_ID;
        this.moduleOffset = moduleOffset;
      }
    }

    // Define Swerve Modules
    public static final SwerveModuleInformation kFrontLeftSwerveModule =
        new SwerveModuleInformation(
            "Front Left", CANID.kFrontLeftDrive, CANID.kFrontLeftTurn, new Rotation2d(Math.PI / 2));
    public static final SwerveModuleInformation kBackLeftSwerveModule =
        new SwerveModuleInformation(
            "Back Left", CANID.kBackLeftDrive, CANID.kBackLeftTurn, new Rotation2d(Math.PI));
    public static final SwerveModuleInformation kFrontRightSwerveModule =
        new SwerveModuleInformation(
            "Front Right", CANID.kFrontRightDrive, CANID.kFrontRightTurn, new Rotation2d(0));
    public static final SwerveModuleInformation kBackRightSwerveModule =
        new SwerveModuleInformation(
            "Back Right",
            CANID.kBackRightDrive,
            CANID.kBackRightTurn,
            new Rotation2d(-Math.PI / 2));

    public static final SwerveModuleInformation[] kIndexedSwerveModuleInformation = {
      kFrontLeftSwerveModule, kBackLeftSwerveModule, kFrontRightSwerveModule, kBackRightSwerveModule
    };

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);

    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);

    public static final Translation2d[] kModuleLocations = {
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };

    public static double kDrivebaseRadius = Math.hypot(kWheelBase, kTrackWidth) / 2;

    public static final double kMaxAngularVelocity =
        MAXSwerveConstants.kMaxDriveSpeed / kDrivebaseRadius;

    public static final double kSlowSpeed = 0.5;

    public static final double kS = 0.1;
    public static final double kV = 12d / MAXSwerveConstants.kMaxDriveSpeed; // v/m/s
    public static final double kA = 0.45; // Accel/V
  }

  public static final class MAXSwerveConstants {

    public static enum DriveRatio {
      LOW(12),
      MEDIUM(13),
      HIGH(14);

      public final int pinionTeeth;

      DriveRatio(int pinionTeeth) {
        this.pinionTeeth = pinionTeeth;
      }
    }

    public static final int kDriveMotorPinionTeeth = DriveRatio.HIGH.pinionTeeth;

    // Invert the Turn encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurnEncoderInverted = true;

    // Calculations required for Drive motor conversion factors and feed forward
    public static final double kDriveMotorFreeSpeedRps =
        Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNEO(1).freeSpeedRadPerSec) / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the
    // bevel pinion
    public static final double kDriveMotorReduction = (45.0 * 22) / (kDriveMotorPinionTeeth * 15);
    public static final double kMaxDriveSpeed =
        (kDriveMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDriveMotorReduction;

    public static final double kDriveEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDriveMotorReduction; // meters
    public static final double kDriveEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDriveMotorReduction) / 60.0; // meters per second

    public static final double kTurnMotorReduction = 9424d / 203;

    public static final double kTurnEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurnEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurnEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurnEncoderPositionPIDMaxInput =
        kTurnEncoderPositionFactor; // radians

    public static final double kDriveP = 0.04;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveFF = 1 / kMaxDriveSpeed;
    public static final double kDriveMinOutput = -1;
    public static final double kDriveMaxOutput = 1;

    public static final double kTurnP = 1.1;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnFF = 0;
    public static final double kTurnMinOutput = -1;
    public static final double kTurnMaxOutput = 1;

    public static final IdleMode kDriveIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurnIdleMode = IdleMode.kBrake;

    public static final int kDriveCurrentLimit = 40; // amps
    public static final int kTurnCurrentLimit = 20; // amps
  }

  public static final class ClimbConstants {

    public static final double kMinHeight = 0;
    public static final double kMaxHeight = 15.25; // Inches
    public static final double kGearing = 1 / 60d;
    public static final double kSprocketPD = 1.790; // Inches
    public static final PIDController kClimbUpPID = new PIDController(2, 0, 0);
    public static final PIDController kClimbDownPID = new PIDController(3, 0, 0);

    public static final double kSensorToVerticalMeters =
        (kGearing * kSprocketPD * Math.PI); // Motor Rotations to Climber Inches

    public static final double kCurrentLimit = 40; // Amps

    public static final double kP_U = 12d / 15.25;
    public static final double kP_D = 24d / 15.25;

    public static final CurrentLimitsConfigs kCurrentConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kCurrentLimit)
            .withSupplyCurrentLimit(kCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(kCurrentLimit)
            .withSupplyTimeThreshold(0);

    public static final class ClimberInformation {
      public final String name;
      public final int id;
      public final boolean inverted;

      public ClimberInformation(String name, int id, Boolean inverted) {
        this.name = name;
        this.id = id;
        this.inverted = inverted;
      }

      public static final ClimberInformation kLeftClimber =
          new ClimberInformation("Left", CANID.kLeftClimber, false);
      public static final ClimberInformation kRightClimber =
          new ClimberInformation("Right", CANID.kRightClimber, true);
    }
  }

  public static final class IntakeConstants {

    public static final double kGearingPivot = (1d / 12) * (16d / 36);
    public static final double kGearingRollers = (11d / 24);

    public static final double kMinPivotAngle = -17;
    public static final double kMaxPivotAngle = 152.25;

    public static final double kPivotCurrentLimit = 50;
    public static final double kRollersCurrentLimit = 30;

    public static final double kGroundIntakeSpeed = 0.9;
    public static final double kFeedSpeed = -0.25;

    public static Slot0Configs kPivotSlot0 =
        new Slot0Configs()
            .withKS(0)
            .withKV(12d / ((6380d / 60) * kGearingPivot)) // Volts/Mechanism RPS
            .withKP(300)
            .withKI(0)
            .withKD(0);

    public static MotionMagicConfigs kPivotMotionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Units.degreesToRotations(600))
            .withMotionMagicAcceleration(Units.degreesToRotations(1000));

    public static final CurrentLimitsConfigs kPivotCurrentConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kPivotCurrentLimit)
            .withSupplyCurrentLimit(kPivotCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(kPivotCurrentLimit)
            .withSupplyTimeThreshold(0);

    public static final CurrentLimitsConfigs kRollersCurrentConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kRollersCurrentLimit)
            .withSupplyCurrentLimit(kRollersCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(kRollersCurrentLimit)
            .withSupplyTimeThreshold(0);
  }

  public static final class OuttakeConstants {

    public static final double kGearingPivot = (1d / 20) * (16d / 36);
    public static final double kGearingFlywheel = (1d / 2);

    public static final double kMinPivotAngle = -10;
    public static final double kPivotAmpAngle = 96;
    public static final double kMaxPivotAngle = 133;
    public static final double kMinFlywheelRpm = -3190;
    public static final double kMaxFlywheelRpm = 3190;

    public static final double kPivotCurrentLimit = 30;
    public static final double kFlywheelCurrentLimit = 30;

    public static final double kFeedRPM = 400;
    public static final double kSpeakerRPM = 2750;
    public static final double kAmpRPM = 1000;

    public static Slot0Configs kPivotSlot0 =
        new Slot0Configs()
            .withKS(0)
            .withKV(12d / ((6380d / 60) * kGearingPivot)) // Volts/Mechanism RPS
            .withKP(150)
            .withKI(0)
            .withKD(0);

    public static MotionMagicConfigs kPivotMotionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Units.degreesToRotations(600))
            .withMotionMagicAcceleration(Units.degreesToRotations(800));

    public static Slot0Configs kFlyWheelSlot0 =
        new Slot0Configs()
            .withKS(.05)
            .withKV(12d / ((6380d / 60) * kGearingFlywheel)) // Volts/Mechanism RPS
            .withKP(0)
            .withKI(0)
            .withKD(0);

    public static final CurrentLimitsConfigs kPivotCurrentConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kPivotCurrentLimit)
            .withSupplyCurrentLimit(kPivotCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(kPivotCurrentLimit)
            .withSupplyTimeThreshold(0);

    public static final CurrentLimitsConfigs kFlywheelCurrentConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kFlywheelCurrentLimit)
            .withSupplyCurrentLimit(kFlywheelCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(kFlywheelCurrentLimit)
            .withSupplyTimeThreshold(0);
  }

  public static final class LEDConstants {
    public static class Color {
      public int red;
      public int green;
      public int blue;

      public Color(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
      }
    }

    public static final Color kDisabledColor = new Color(255, 10, 0); // Orange
    public static final Color kEnabledColor = new Color(0, 255, 0); // Green
    public static final Color kProcessingColor = new Color(255, 0, 0); // Red
    public static final Color kBlinkModeColor = new Color(8, 255, 8); // Also Green
    public static final Color kAmpSignalColor = new Color(128, 16, 255); // Purple
  }
}
