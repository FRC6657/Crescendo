package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SwerveModuleInformation;
import frc.robot.Constants.MAXSwerveConstants;
import org.littletonrobotics.junction.Logger;

public class MAXSwerveIO_Real implements MAXSwerveIO {

  CANSparkMax driveMotor;
  CANSparkMax turnMotor;

  RelativeEncoder driveEncoder;
  AbsoluteEncoder turnEncoder;

  // Onboard PID controller
  SparkPIDController drivePID;
  SparkPIDController turnPID;

  SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

  SwerveModuleInformation moduleInformation;

  double turnSetpoint = 0.0;
  double driveSetpoint = 0.0;

  double arbFF = 0.0;

  double lastDriveSetpoint = 0.0;

  public MAXSwerveIO_Real(SwerveModuleInformation moduleInformation) {

    this.moduleInformation = moduleInformation;

    driveMotor = new CANSparkMax(moduleInformation.driveCAN_ID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(moduleInformation.turnCAN_ID, MotorType.kBrushless);

    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    drivePID = driveMotor.getPIDController();
    turnPID = turnMotor.getPIDController();

    drivePID.setFeedbackDevice(driveEncoder);
    turnPID.setFeedbackDevice(turnEncoder);

    driveEncoder.setPositionConversionFactor(MAXSwerveConstants.kDriveEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(MAXSwerveConstants.kDriveEncoderVelocityFactor);
    driveEncoder.setMeasurementPeriod((int) (1000 / CodeConstants.kMainLoopFrequency));
    driveEncoder.setAverageDepth(2);

    turnEncoder.setPositionConversionFactor(MAXSwerveConstants.kTurnEncoderPositionFactor);
    turnEncoder.setVelocityConversionFactor(MAXSwerveConstants.kTurnEncoderVelocityFactor);
    turnEncoder.setInverted(MAXSwerveConstants.kTurnEncoderInverted);

    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(MAXSwerveConstants.kTurnEncoderPositionPIDMinInput);
    turnPID.setPositionPIDWrappingMaxInput(MAXSwerveConstants.kTurnEncoderPositionPIDMaxInput);

    drivePID.setP(MAXSwerveConstants.kDriveP);
    drivePID.setI(MAXSwerveConstants.kDriveI);
    drivePID.setD(MAXSwerveConstants.kDriveD);
    drivePID.setOutputRange(MAXSwerveConstants.kDriveMinOutput, MAXSwerveConstants.kDriveMaxOutput);

    turnPID.setP(MAXSwerveConstants.kTurnP);
    turnPID.setI(MAXSwerveConstants.kTurnI);
    turnPID.setD(MAXSwerveConstants.kTurnD);
    turnPID.setFF(MAXSwerveConstants.kTurnFF);
    turnPID.setOutputRange(MAXSwerveConstants.kTurnMinOutput, MAXSwerveConstants.kTurnMaxOutput);

    driveMotor.setIdleMode(MAXSwerveConstants.kDriveIdleMode);
    turnMotor.setIdleMode(MAXSwerveConstants.kTurnIdleMode);

    driveMotor.setSmartCurrentLimit(MAXSwerveConstants.kDriveCurrentLimit);
    turnMotor.setSmartCurrentLimit(MAXSwerveConstants.kTurnCurrentLimit);

    driveMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000 / CodeConstants.kMainLoopFrequency));
    turnMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000 / CodeConstants.kMainLoopFrequency));

    driveMotor.burnFlash();
    turnMotor.burnFlash();
  }

  /** Updates the IO */
  @Override
  public void updateInputs(MAXSwerveIOInputs inputs) {

    arbFF =
        driveFeedforward.calculate(
            driveSetpoint, (driveSetpoint - lastDriveSetpoint) * CodeConstants.kMainLoopFrequency);
    lastDriveSetpoint = driveSetpoint;

    Logger.recordOutput("DriveFFs/ " + moduleInformation.name, arbFF);

    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMPS = driveEncoder.getVelocity();

    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

    inputs.turnPositionRad = getTurnAngle();
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.turnCurrentAmps = turnMotor.getOutputCurrent();

    inputs.turnError = turnSetpoint - inputs.turnPositionRad.getRadians();
  }

  /** Sets the drive MPS Setpoint */
  @Override
  public void setDriveMPS(double mps) {
    driveSetpoint = mps;
    drivePID.setReference(mps, ControlType.kVelocity, 0, arbFF);
  }

  /** Sets the turn angle setpoint */
  @Override
  public void setTurnAngle(Rotation2d angle) {

    turnSetpoint = angle.getRadians() - moduleInformation.moduleOffset.getRadians();

    turnPID.setReference(
        angle.getRadians() - moduleInformation.moduleOffset.getRadians(), ControlType.kPosition);
  }

  /** Get the turn angle */
  @Override
  public Rotation2d getTurnAngle() {
    return new Rotation2d(turnEncoder.getPosition() + moduleInformation.moduleOffset.getRadians());
  }
}
