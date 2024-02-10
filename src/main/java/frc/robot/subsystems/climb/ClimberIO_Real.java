package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.ClimberInformation;

public class ClimberIO_Real implements ClimberIO {
  private TalonFX mMotor;
  private final PIDController mPID = ClimbConstants.kClimbUpPID;
  private double setpoint = 0.0;
  private double voltage = 0.0;

  public ClimberIO_Real(ClimberInformation info) {
    mMotor = new TalonFX(info.id);

    var motorConfigurator = mMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();

    motorConfigs.CurrentLimits = ClimbConstants.kCurrentConfigs;
    motorConfigs.Feedback.SensorToMechanismRatio = ClimbConstants.kGearing;

    motorConfigurator.apply(motorConfigs);
  }

  private double getHeight() {
    return mMotor.getPosition().getValueAsDouble() * ClimbConstants.kSprocketPD;
  }

  @Override
  public void changeSetpoint(double height) {
    setpoint = height;
  }

  @Override
  public void run() {

    if (setpoint >= getHeight()) {
      mPID.setP(ClimbConstants.kClimbDownPID.getP());
    } else {
      mPID.setP(ClimbConstants.kClimbUpPID.getP());
    }

    voltage = MathUtil.clamp(mPID.calculate(getHeight(), setpoint), -12, 12);
    mMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVoltage = mMotor.getMotorVoltage().getValueAsDouble();
    inputs.position = getHeight();
    inputs.current = mMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocity = mMotor.getVelocity().getValueAsDouble();
  }
}
