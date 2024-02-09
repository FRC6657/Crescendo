package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANID;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIO_Real implements IntakeIO {

  private TalonFX intake;
  private TalonFX pivot;
  private double intakeMotorSpeed;
  double lastVelocity = 0;
  double currentVelocity = 0;

  private MotionMagicVoltage pivotAngle = new MotionMagicVoltage(Units.degreesToRotations(IntakeConstants.kPivotMinAngle));


  public IntakeIO_Real() {
    intake = new TalonFX(CANID.kIntakePivot);
    pivot = new TalonFX(CANID.kIntakePivot);

    var pivotConfigs = new TalonFXConfiguration();
    var intakeConfigs = new TalonFXConfiguration();

    pivotConfigs.Feedback.SensorToMechanismRatio = 1.0/IntakeConstants.kPivotGearing;
    pivotConfigs.Slot0 = IntakeConstants.kPivotSlot0;

    pivotConfigs.CurrentLimits = IntakeConstants.kCurrentConfigs;
    intakeConfigs.CurrentLimits = IntakeConstants.kCurrentConfigs;

    var tempSignalS = intake.getDeviceTemp();
    var currentSignalS = pivot.getSupplyCurrent();

    var tempSignalP = pivot.getDeviceTemp();
    var currentSignalP = pivot.getSupplyCurrent();

    tempSignalS.setUpdateFrequency(CodeConstants.kMainLoopFrequency / 4);
    currentSignalS.setUpdateFrequency(CodeConstants.kMainLoopFrequency);

    tempSignalP.setUpdateFrequency(CodeConstants.kMainLoopFrequency / 4);
    currentSignalP.setUpdateFrequency(CodeConstants.kMainLoopFrequency);

    intake.optimizeBusUtilization();
    pivot.optimizeBusUtilization();

    intake.getConfigurator().apply(intakeConfigs);
    pivot.getConfigurator().apply(pivotConfigs);

    pivot.setPosition(Units.degreesToRotations(IntakeConstants.kPivotMinAngle));
    pivot.setControl(pivotAngle);


  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.currentSpeed = intakeMotorSpeed;
    inputs.rollerMotorTemp = intake.getDeviceTemp().getValueAsDouble();
    inputs.rollerMotorVoltage = intake.getMotorVoltage().getValueAsDouble();
    inputs.rollerMotorCurrent = intake.getSupplyCurrent().getValueAsDouble();
    inputs.rollerMotorAcceleration = intake.getAcceleration().getValueAsDouble();

    inputs.pivotMotorPosition = Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble());
    inputs.pivotMotorTemp = pivot.getDeviceTemp().getValueAsDouble();
    inputs.pivotMotorVoltage = pivot.getMotorVoltage().getValueAsDouble();
    inputs.pivotMotorCurrent = pivot.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runRollers(double speed) {
    intakeMotorSpeed = MathUtil.clamp(speed, -1, 1);
    intake.set(intakeMotorSpeed);
  }

  @Override
  public void changePivot(double angle){
    pivotAngle.withPosition(angle);
    pivot.setControl(pivotAngle);
  }
}
