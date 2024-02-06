package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeIO_Real implements OuttakeIO {

  TalonFX rightShooter = new TalonFX(Constants.CANID.kRightShooter);
  TalonFX leftShooter = new TalonFX(Constants.CANID.kLeftShooter);

  TalonFX pivot = new TalonFX(Constants.CANID.kShooterPivot);

  private double angle = 0.0;

  private MotionMagicVoltage pivotRequest = new MotionMagicVoltage(Units.degreesToRotations(-10));
  private MotionMagicVelocityVoltage flywheelRequest = new MotionMagicVelocityVoltage(0);

  public OuttakeIO_Real() {

    rightShooter.setControl(new StrictFollower(Constants.CANID.kLeftShooter));

    var shooterPivotConfigurator = pivot.getConfigurator();
    var shooterPivotConfigs = new TalonFXConfiguration();

    var flywheelConfigs = new TalonFXConfiguration();

    // check this
    shooterPivotConfigs.Feedback.SensorToMechanismRatio =
        Constants.OuttakeConstants.kSensorToRotations;

    var pivotSlot0 = new Slot0Configs();
    pivotSlot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    pivotSlot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    pivotSlot0.kP = 2; // A position error of 2.5 rotations results in 12 V output
    pivotSlot0.kI = 0; // no output for integrated error
    pivotSlot0.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    pivotSlot0.GravityType = GravityTypeValue.Arm_Cosine;

    var flywheelSlot0 = new Slot0Configs();
    flywheelSlot0.kS = 0.25;
    flywheelSlot0.kV = 0.12;
    flywheelSlot0.kP = 2;
    flywheelSlot0.kI = 0;
    flywheelSlot0.kD = 0;

    var motionMagicConfigs = flywheelConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400;
    motionMagicConfigs.MotionMagicJerk = 4000; // 4000 rps/s/s or 0.1 seconds

    leftShooter.getConfigurator().apply(flywheelConfigs);

    pivot.getConfigurator().apply(pivotSlot0);

    pivot.setPosition(Units.degreesToRotations(-10));

    shooterPivotConfigurator.apply(shooterPivotConfigs);

    // Motor Configurations
    shooterPivotConfigs.CurrentLimits.StatorCurrentLimit = OuttakeConstants.kCurrentLimit;
    shooterPivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

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
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {

    pivot.setControl(pivotRequest);

    leftShooter.setControl(flywheelRequest);

    inputs.pivotMotorPosition = angle;

    inputs.pivotMotorVoltage = pivot.getSupplyVoltage().getValueAsDouble();
    inputs.flywheelMotorVoltage = leftShooter.getSupplyVoltage().getValueAsDouble();

    inputs.pivotMotorTemp = pivot.getDeviceTemp().getValueAsDouble();
    inputs.flywheelMotorTemp = pivot.getDeviceTemp().getValueAsDouble();

    inputs.pivotMotorCurrent = pivot.getSupplyCurrent().getValueAsDouble();
    inputs.flywheelMotorCurrent = leftShooter.getSupplyCurrent().getValueAsDouble();

    pivot.getPosition();
  }

  @Override
  public void runFlywheel(double rpm) {
    flywheelRequest = new MotionMagicVelocityVoltage(rpm);
  }

  @Override
  public void runPivot(double angle) {
    pivotRequest = new MotionMagicVoltage(angle);
  }
}
