package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.DriveConstants.SwerveModuleInformation;
import frc.robot.Constants.MAXSwerveConstants;

public class MAXSwerveIO_Real implements MAXSwerveIO {

  CANSparkMax driveMotor;
  CANSparkMax turnMotor;

  RelativeEncoder driveEncoder;
  AbsoluteEncoder turnEncoder;

  // Onboard PID controller
  SparkPIDController drivePID;
  SparkPIDController turnPID;

  SwerveModuleInformation moduleInformation;

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

    driveEncoder.setPositionConversionFactor(MAXSwerveConstants.kDriveEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(MAXSwerveConstants.kDriveEncoderVelocityFactor);
    driveEncoder.setMeasurementPeriod((int) (1000 / CodeConstants.kMainLoopFrequency));
    driveEncoder.setAverageDepth(2);

    turnEncoder.setPositionConversionFactor(MAXSwerveConstants.kTurnEncoderPositionFactor);
    turnEncoder.setVelocityConversionFactor(MAXSwerveConstants.kTurnEncoderVelocityFactor);
    turnEncoder.setInverted(MAXSwerveConstants.kTurnEncoderInverted);
    turnEncoder.setAverageDepth(2);

    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(MAXSwerveConstants.kTurnEncoderPositionPIDMinInput);
    turnPID.setPositionPIDWrappingMaxInput(MAXSwerveConstants.kTurnEncoderPositionPIDMaxInput);

    drivePID.setP(MAXSwerveConstants.kDriveP);
    drivePID.setI(MAXSwerveConstants.kDriveI);
    drivePID.setD(MAXSwerveConstants.kDriveD);
    drivePID.setFF(MAXSwerveConstants.kDriveFF);
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

    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMPS = driveEncoder.getVelocity();

    inputs.driveAppliedVolts = driveMotor.getAppliedOutput();
    inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

    inputs.turnPositionRad = getTurnAngle();
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput();
    inputs.turnCurrentAmps = turnMotor.getOutputCurrent();

    Logger.recordOutput("Turn Voltage", turnMotor.getAppliedOutput());
  }


  /** Sets the drive MPS Setpoint */
  @Override
  public void setDriveMPS(double mps) {
    drivePID.setReference(mps, ControlType.kVelocity);
  }

  /** Sets the turn angle setpoint */
  @Override
  public void setTurnAngle(Rotation2d angle) {
    turnPID.setReference(angle.getRadians() - moduleInformation.moduleOffset.getRadians(), ControlType.kPosition);
  }

  /** Get the turn angle */
  @Override
  public Rotation2d getTurnAngle() {
    return new Rotation2d(turnEncoder.getPosition() + moduleInformation.moduleOffset.getRadians());
  }
}
