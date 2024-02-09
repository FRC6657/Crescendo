package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIO_Real implements IntakeIO {

  private TalonFX intakeMotor;
  private double intakeMotorSpeed;
  double lastVelocity = 0;
  double currentVelocity = 0;
  private TalonFX pivot;

  private double angle = 0.0;

  private MotionMagicVoltage pivotRequest =
      new MotionMagicVoltage(
          Units.degreesToRotations(-10));
  // sets degrees to aim for right away

  public IntakeIO_Real() {
    intakeMotor = new TalonFX(Constants.CANID.kIntakeMotor);
    pivot = new TalonFX(Constants.CANID.kIntakePivot);

    var intakeMotorConfigurator = intakeMotor.getConfigurator();
    var intakeMotorConfigs = new TalonFXConfiguration();

    intakeMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeMotorConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.kMotorCurrentLimit;
    intakeMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeMotorConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kMotorCurrentLimit;
    intakeMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    var dutyCycleSignal = intakeMotor.getDutyCycle();
    var tempSignal = intakeMotor.getDeviceTemp();
    var currentSignal = intakeMotor.getSupplyCurrent();

    dutyCycleSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
    tempSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency / 4);
    currentSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);

    intakeMotorConfigurator.apply(intakeMotorConfigs);
    intakeMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.currentSpeed = intakeMotorSpeed;
    inputs.rollerMotorTemp = intakeMotor.getDeviceTemp().getValueAsDouble();
    inputs.rollerMotorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerMotorCurrent = intakeMotor.getSupplyCurrent().getValueAsDouble();

    currentVelocity = intakeMotor.getVelocity().getValueAsDouble();
    inputs.rollerMotorAcceleration =
        (currentVelocity - lastVelocity) * CodeConstants.kMainLoopFrequency;
    lastVelocity = currentVelocity;
  }

  @Override
  public void runRollers(double speed) {
    intakeMotorSpeed = MathUtil.clamp(speed, -1, 1);
    intakeMotor.set(intakeMotorSpeed);
  }
}
