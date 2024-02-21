package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.ClimberInformation;

public class ClimberIO_Real implements ClimberIO {
  private CANSparkMax mMotor;
  private RelativeEncoder mEncoder;
  private final PIDController mPID = ClimbConstants.kClimbUpPID;
  private double setpoint = 0.0;
  private double voltage = 0.0;

  public ClimberIO_Real(ClimberInformation info) {
    mMotor = new CANSparkMax(info.id, MotorType.kBrushless);
    mEncoder = mMotor.getEncoder();

    mMotor.setSmartCurrentLimit(40);

    if (info.name == "rightClimber") {
        mMotor.setInverted(true);
      } else {
        mMotor.setInverted(false);
      }

    mEncoder.setPositionConversionFactor(ClimbConstants.kGearing * ClimbConstants.kSprocketPD);

  }

  @Override
  public void changeSetpoint(double height) {
    setpoint = height;
  }

  @Override
  public void run() {

    if (setpoint <= mEncoder.getPosition()) {
      mPID.setP(ClimbConstants.kClimbDownPID.getP());
    } else {
      mPID.setP(ClimbConstants.kClimbUpPID.getP());
    }

    voltage = MathUtil.clamp(mPID.calculate(mEncoder.getPosition(), setpoint), -12, 12);
    mMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVoltage = mMotor.getBusVoltage() * mMotor.getAppliedOutput();
    inputs.position = mEncoder.getPosition();
    inputs.current = mMotor.getOutputCurrent();
    inputs.velocity = mEncoder.getVelocity();
  }
}
