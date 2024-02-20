package frc.robot.subsystems.climb;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.ClimberInformation;

public class ClimberIO_Real implements ClimberIO {
  private CANSparkMax mMotor;
  private AbsoluteEncoder mEncoder;
  private final PIDController mPID = ClimbConstants.kClimbUpPID;
  private double setpoint = 0.0;
  private double voltage = 0.0;

  public ClimberIO_Real(ClimberInformation info) {
    mMotor = new CANSparkMax(info.id, MotorType.kBrushless);
    mEncoder = mMotor.getAbsoluteEncoder(Type.kDutyCycle);

    mMotor.setSmartCurrentLimit(40);

    if (info.name == "rightClimber") {
        mMotor.setInverted(true);
      } else {
        mMotor.setInverted(false);
      }

    mEncoder.setPositionConversionFactor(ClimbConstants.kGearing);

  }

  private double getHeight() {
    return mEncoder.getPosition() * ClimbConstants.kSprocketPD;
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
    inputs.appliedVoltage = mMotor.getBusVoltage();
    inputs.position = getHeight();
    inputs.current = mMotor.getOutputCurrent();
    inputs.velocity = mEncoder.getVelocity();
  }
}
