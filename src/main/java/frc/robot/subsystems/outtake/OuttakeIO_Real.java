package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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

  private MotionMagicVoltage pivotAngle = new MotionMagicVoltage(Units.degreesToRotations(-10));
  private MotionMagicVelocityVoltage flywheelVelocity = new MotionMagicVelocityVoltage(0);

  public OuttakeIO_Real() {

    rightShooter.setControl(new StrictFollower(Constants.CANID.kLeftShooter));

    var pivotConfigurator = pivot.getConfigurator();
    var pivotConfigs = new TalonFXConfiguration();

    var flywheelConfigs = new TalonFXConfiguration();

    pivotConfigs.Feedback.SensorToMechanismRatio =
        1.0/Constants.OuttakeConstants.kGearingPivot;

    flywheelConfigs.Feedback.SensorToMechanismRatio =
        1.0/Constants.OuttakeConstants.kGearingFlyWheel;

    var pivotSlot0 = pivotConfigs.Slot0;
    pivotSlot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    pivotSlot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    pivotSlot0.kP = 2; // A position error of 2.5 rotations results in 12 V output
    pivotSlot0.kI = 0; // no output for integrated error
    pivotSlot0.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    pivotSlot0.GravityType = GravityTypeValue.Arm_Cosine;

    var flywheelSlot0 = flywheelConfigs.Slot0;
    flywheelSlot0.kS = 0.25;
    flywheelSlot0.kV = 0.12;
    flywheelSlot0.kP = 2;
    flywheelSlot0.kI = 0;
    flywheelSlot0.kD = 0;

    var motionMagicConfigs = flywheelConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400;
    motionMagicConfigs.MotionMagicJerk = 4000; // 4000 rps/s/s or 0.1 seconds

    

    // Motor Configurations
    pivotConfigs.CurrentLimits.StatorCurrentLimit = OuttakeConstants.kCurrentLimit;
    pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    flywheelConfigs.CurrentLimits.StatorCurrentLimit = OuttakeConstants.kCurrentLimit;
    flywheelConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    var tempSignalS = leftShooter.getDeviceTemp();
    var currentSignalS = leftShooter.getSupplyCurrent();

    var tempSignalP = leftShooter.getDeviceTemp();
    var currentSignalP = leftShooter.getSupplyCurrent();

    tempSignalS.setUpdateFrequency(CodeConstants.kMainLoopFrequency / 4);
    currentSignalS.setUpdateFrequency(CodeConstants.kMainLoopFrequency);

    tempSignalP.setUpdateFrequency(CodeConstants.kMainLoopFrequency / 4);
    currentSignalP.setUpdateFrequency(CodeConstants.kMainLoopFrequency);

    leftShooter.optimizeBusUtilization();
    pivot.optimizeBusUtilization();

    leftShooter.getConfigurator().apply(flywheelConfigs);
    pivotConfigurator.apply(pivotConfigs);


    pivot.setPosition(Units.degreesToRotations(-10));

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
    flywheelVelocity.withVelocity(rpm);
    leftShooter.setControl(flywheelVelocity);
  }

  @Override
  public void changePivot(double angle) {
    pivotAngle.withPosition(angle);
    pivot.setControl(pivotAngle);
  }


}
