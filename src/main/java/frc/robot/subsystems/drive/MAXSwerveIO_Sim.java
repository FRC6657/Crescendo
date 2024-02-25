package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.MAXSwerveConstants;

public class MAXSwerveIO_Sim implements MAXSwerveIO {

  private DCMotorSim driveMotor =
      new DCMotorSim(DCMotor.getNEO(1), MAXSwerveConstants.kDriveMotorReduction, 0.025);

  private DCMotorSim turnMotor =
      new DCMotorSim(DCMotor.getNeo550(1), MAXSwerveConstants.kTurnMotorReduction, 0.025);

  private final Rotation2d turnAbsoluteInitialPosition =
      new Rotation2d(Math.random() * 2 * Math.PI);

  private double driveVolts = 0.0;
  private double turnVolts = 0.0;

  private double mpsSetpoint = 0.0;
  private Rotation2d turnAngleSetpoint = new Rotation2d();

  private PIDController turnController = new PIDController(15, 0.0, 0.0);
  private PIDController driveController = new PIDController(5, 0.0, 0.0);
  private SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(0.0, MAXSwerveConstants.kDriveFF * 12);

  public MAXSwerveIO_Sim() {
    // Set the range of the turn motor
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    // Set the initial position of the turn motor
    turnMotor.setState(turnAbsoluteInitialPosition.getRadians(), 0);
  }

  @Override
  public void updateInputs(MAXSwerveIOInputs inputs) {

    setDriveVoltage(
        driveFeedforward.calculate(mpsSetpoint)
            + driveController.calculate(
                (driveMotor.getAngularVelocityRPM() * MAXSwerveConstants.kWheelCircumferenceMeters)
                    / 60,
                mpsSetpoint));
    setTurnVoltage(
        turnController.calculate(getTurnAngle().getRadians(), turnAngleSetpoint.getRadians()));

    // Step the simulation forward
    driveMotor.update(1 / CodeConstants.kMainLoopFrequency);
    turnMotor.update(1 / CodeConstants.kMainLoopFrequency);

    // Update the inputs
    inputs.drivePositionMeters =
        driveMotor.getAngularPositionRotations() * MAXSwerveConstants.kWheelCircumferenceMeters;
    inputs.driveVelocityMPS =
        (driveMotor.getAngularVelocityRPM() * MAXSwerveConstants.kWheelCircumferenceMeters) / 60;
    inputs.driveAppliedVolts = driveVolts;
    inputs.driveCurrentAmps = driveMotor.getCurrentDrawAmps();

    inputs.turnPositionRad = new Rotation2d(turnMotor.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnMotor.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnVolts;
    inputs.turnCurrentAmps = turnMotor.getCurrentDrawAmps();
  }

  /** Sets the drive motor voltage */
  public void setDriveVoltage(double volts) {
    driveVolts = MathUtil.clamp(volts, -12, 12);
    driveMotor.setInputVoltage(volts);
  }

  /** Sets the turn motor voltage */
  public void setTurnVoltage(double volts) {
    turnVolts = MathUtil.clamp(volts, -12, 12);
    turnMotor.setInputVoltage(turnVolts);
  }

  /** Sets the drive MPS setpoint */
  @Override
  public void setDriveMPS(double mps) {
    mpsSetpoint = mps;
  }

  /** Sets the turn angle setpoint */
  @Override
  public void setTurnAngle(Rotation2d angle) {

    turnAngleSetpoint = angle;
  }

  /** Gets the turn angle */
  @Override
  public Rotation2d getTurnAngle() {
    return Rotation2d.fromRadians(turnMotor.getAngularPositionRad());
  }
}
