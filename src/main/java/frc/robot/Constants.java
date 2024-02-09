package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static final class CodeConstants {
    public static final double kMainLoopFrequency = 50; // Hz
  }

  public static final class VisionConstants {
    public static final Transform3d kFrontCameraPose =
        new Transform3d(new Translation3d(-0.336886, 0, 0.531201), new Rotation3d(0, 0.485314, 0));
    public static final Transform3d kSideCameraPose =
        new Transform3d(
            new Translation3d(-0.242086, 0.170791, 0.511322),
            new Rotation3d(0, 0.122173, -1.28248701081));
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

    public static final int kLeftShooter = 11;
    public static final int kRightShooter = 12;

    public static final int kShooterPivot = 13;

    public static final int kLeftClimber = 14;
    public static final int kRightClimber = 15;

    public static final int kIntakeMotor = 16;
    public static final int kIntakePivot = 17;
  }

  public static final class AutoConstants {
    // Choreo
    public static final double kChor_P_X = 1;
    public static final double kChor_P_Y = 1;
    public static final double kChor_P_Theta = 1;

    // Auto Align
    public static final double kAA_P_X = 3;
    public static final double kAA_P_Y = 3;
    public static final double kAA_P_Theta = 3;
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

    public static final double kDrivebaseRadius = Math.hypot(kWheelBase, kTrackWidth) / 2;

    public static final double kMaxAngularVelocity =
        MAXSwerveConstants.kMaxDriveSpeed / kDrivebaseRadius;
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

    public static final double kTurnP = 1;
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

  public static final class IntakeConstants {

    // Pivot
    public static final double kPivotGearing = 1d / 12 * 16d / 36;
    public static final double kPivotMinAngle = -27.5;
    public static final double kPivotMaxAngle = 152.25;

    //Rollers
    public static final double kMotorCurrentLimit = 20; // Amps
  }

  public static final class ClimbConstants {

    public static final double kMinHeight = 0;
    public static final double kMaxHeight = 15.25; // Inches
    public static final double kGearing = 1 / 20d;
    public static final double kSprocketPD = 1.790; // Inches
    public static final PIDController kClimbUpPID = new PIDController(0.1, 0, 0);
    public static final PIDController kClimbDownPID = new PIDController(0.15, 0, 0);

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

      public ClimberInformation(String name, int id) {
        this.name = name;
        this.id = id;
      }

      public static final ClimberInformation kLeftClimber =
          new ClimberInformation("Left", CANID.kLeftClimber);
      public static final ClimberInformation kRightClimber =
          new ClimberInformation("Right", CANID.kRightClimber);
    }
  }

  public static final class OuttakeConstants {

    public static final double kGearingPivot = (1d / 12) * (16d / 36);
    public static final double kGearingFlyWheel = (1d / 12) * (16d / 36);
    public static final double kMinAngle = -10;
    public static final double kMaxAngle = 133;
    public static final double kMinRpm = 0;
    public static final double kMaxRpm = 3190;
    public static final double kCurrentLimit = 40;

    public static Slot0Configs kPivotSlot0 = new Slot0Configs()
    .withKS(0.25) // Add 0.25 V output to overcome static friction
    .withKV(0.12) // A velocity target of 1 rps results in 0.12 V output
    .withKP(2) // A position error of 2.5 rotations results in 12 V output
    .withKI(0) // no output for integrated error
    .withKD(0)
    .withGravityType(GravityTypeValue.Arm_Cosine); // no d

    public static Slot0Configs kFlyWheelSlot0 = new Slot0Configs()
    .withKS(0.25) // Add 0.25 V output to overcome static friction
    .withKV(0.12) // A velocity target of 1 rps results in 0.12 V output
    .withKP(2) // A position error of 2.5 rotations results in 12 V output
    .withKI(0) // no output for integrated error
    .withKD(0); // no d

    public static final MotionMagicConfigs kMotionMagicConfigsFlyWheel = new MotionMagicConfigs()
    .withMotionMagicAcceleration(400)
    .withMotionMagicJerk(4000);

    public static final CurrentLimitsConfigs kCurrentConfigs = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(kCurrentLimit)
    .withSupplyCurrentLimit(kCurrentLimit)
    .withStatorCurrentLimitEnable(true)
    .withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentThreshold(kCurrentLimit)
    .withSupplyTimeThreshold(0);

  
  }
}
