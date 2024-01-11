package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants.IntakeDirection;

public class Constants {

  public static final class CodeConstants {
    public static final double kMainLoopFrequency = 50; // Hz
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

    // public static final int kElevator = 10;
    // public static final int kIntake = 11;

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
            "Front Left",
            CANID.kFrontLeftDrive,
            CANID.kFrontLeftTurn,
            new Rotation2d(Math.PI / 2));
    public static final SwerveModuleInformation kBackLeftSwerveModule =
        new SwerveModuleInformation(
            "Back Left", CANID.kBackLeftDrive, CANID.kBackLeftTurn, new Rotation2d(Math.PI));
    public static final SwerveModuleInformation kFrontRightSwerveModule =
        new SwerveModuleInformation(
            "Front Right", CANID.kFrontRightDrive, CANID.kFrontRightTurn, new Rotation2d(0));
    public static final SwerveModuleInformation kBackRightSwerveModule =
        new SwerveModuleInformation(
            "Back Right", CANID.kBackRightDrive, CANID.kBackRightTurn, new Rotation2d(-Math.PI / 2));

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

    public static enum IntakeDirection {
      IN,
      OUT
    }

    public static final double kGearing = 1 / 2d;
    public static final double kHoldingVoltage = 1;
    public static final double kCurrentLimit = 20; // Amps
  }

  public static final class ElevatorConstants {

    public static enum GamePiece {
      CONE,
      CUBE
    }

    public static final double kMinHeight = 9.6;
    public static final double kMaxHeight = 54;
    public static final int kStages = 2;
    public static final double kGearing = 1 / 12d;
    public static final double kAngle = Units.degreesToRadians(50);
    public static final double kSprocketPD = 1.751; // Inches

    public static final double kSensorToVerticalInches =
        kGearing * (1.751 * Math.PI) * kStages * Math.sin(kAngle);

    public static final double kStartingHeight = 9.6;

    public static final double kCurrentLimit = 40; // Amps

    public static final double kP = 1 / 5d;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;

    // Control Constants
    public static final Slot0Configs kPIDConfigs =
        new Slot0Configs()
            .withKS(0.05)
            .withKV(0.12)
            .withKP(0.11)
            .withGravityType(GravityTypeValue.Elevator_Static);

    public static final CurrentLimitsConfigs kCurrentConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(40)
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(40)
            .withSupplyTimeThreshold(40);
  }

  public static final record ScoringSetpoint(
      String name,
      double coneHeight,
      double cubeHeight,
      IntakeDirection intakeDirection,
      double coneSpeed,
      double cubeSpeed) {}

  public static final class ScoringSetpoints {
    public static final ScoringSetpoint kZero =
        new ScoringSetpoint("Zero", 9.6, 9.6, IntakeDirection.IN, 0, 0);
    public static final ScoringSetpoint kCarry =
        new ScoringSetpoint("Carry", 9.6, 10, IntakeDirection.IN, 0.2, 0.4);
    public static final ScoringSetpoint kUp =
        new ScoringSetpoint("Up", 54, 21, IntakeDirection.OUT, 0.3, 0.5);
  }
}
