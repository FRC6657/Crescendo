package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeIO_Real implements OuttakeIO {

  TalonFX rightShooter = new TalonFX(Constants.CANID.kRightShooter);
  TalonFX leftShooter = new TalonFX(Constants.CANID.kLeftShooter);

  TalonFX pivot = new TalonFX(Constants.CANID.kShooterPivot);
  DigitalInput beambreak = new DigitalInput(0);

  private double angle = 0.0;

  private MotionMagicVoltage pivotAngle = new MotionMagicVoltage(Units.degreesToRotations(OuttakeConstants.kMinAngle));
  private MotionMagicVelocityVoltage flywheelVelocity = new MotionMagicVelocityVoltage(0);
  private final VelocityVoltage velocity = new VelocityVoltage(0).withSlot(0);

  public OuttakeIO_Real() {

    rightShooter.setControl(new StrictFollower(Constants.CANID.kLeftShooter));

    var pivotConfigurator = pivot.getConfigurator();
    var pivotConfigs = new TalonFXConfiguration();

    var flywheelConfigs = new TalonFXConfiguration();

    pivotConfigs.Feedback.SensorToMechanismRatio = 1.0/OuttakeConstants.kGearingPivot;

    flywheelConfigs.Feedback.SensorToMechanismRatio = 1.0/(OuttakeConstants.kGearingFlyWheel*OuttakeConstants.kRpsToRpm);

    pivotConfigs.Slot0 = OuttakeConstants.kPivotSlot0;

    flywheelConfigs.Slot0 = OuttakeConstants.kFlyWheelSlot0;

    flywheelConfigs.MotionMagic = OuttakeConstants.kMotionMagicConfigsFlyWheel;

    

    // Motor Configurations
    pivotConfigs.CurrentLimits = OuttakeConstants.kCurrentConfigs;
    flywheelConfigs.CurrentLimits = OuttakeConstants.kCurrentConfigs;

    var tempSignalS = leftShooter.getDeviceTemp();
    var currentSignalS = leftShooter.getSupplyCurrent();

    var tempSignalP = pivot.getDeviceTemp();
    var currentSignalP = pivot.getSupplyCurrent();

    tempSignalS.setUpdateFrequency(CodeConstants.kMainLoopFrequency / 4);
    currentSignalS.setUpdateFrequency(CodeConstants.kMainLoopFrequency);

    tempSignalP.setUpdateFrequency(CodeConstants.kMainLoopFrequency / 4);
    currentSignalP.setUpdateFrequency(CodeConstants.kMainLoopFrequency);

    leftShooter.optimizeBusUtilization();
    pivot.optimizeBusUtilization();

    leftShooter.getConfigurator().apply(flywheelConfigs);
    pivotConfigurator.apply(pivotConfigs);


    pivot.setPosition(Units.degreesToRotations(OuttakeConstants.kMinAngle));

    pivot.setControl(pivotAngle);
    leftShooter.setControl(flywheelVelocity);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {

    inputs.pivotMotorPosition = angle;

    inputs.pivotMotorVoltage = pivot.getSupplyVoltage().getValueAsDouble();
    inputs.flywheelMotorVoltage = leftShooter.getSupplyVoltage().getValueAsDouble();

    inputs.pivotMotorTemp = pivot.getDeviceTemp().getValueAsDouble();
    inputs.flywheelMotorTemp = pivot.getDeviceTemp().getValueAsDouble();

    inputs.pivotMotorCurrent = pivot.getSupplyCurrent().getValueAsDouble();
    inputs.flywheelMotorCurrent = leftShooter.getSupplyCurrent().getValueAsDouble();

    pivot.getPosition();
  }

  public boolean beambreak(){
    return !beambreak.get();
  }


  @Override
  public void changeFlywheel(double rpm) {
    rpm = MathUtil.clamp(rpm, OuttakeConstants.kMinRpm, OuttakeConstants.kMaxRpm);
    leftShooter.setControl(velocity.withVelocity(rpm).withFeedForward(OuttakeConstants.kFeedForwardMulitplier*rpm));
  }

  @Override
  public void changePivot(double angle) {
    angle = MathUtil.clamp(angle, OuttakeConstants.kMinAngle, OuttakeConstants.kMaxAngle);
    pivot.setControl(pivotAngle.withPosition(angle));
  }


}
