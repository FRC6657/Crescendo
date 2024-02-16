package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import org.littletonrobotics.junction.AutoLogOutput;

public class OuttakeIO_Real implements OuttakeIO {

  // Pivot Motor Controller
  TalonFX pivotMotor = new TalonFX(Constants.CANID.kShooterPivot);

  // Flywheel Motor Controllers
  TalonFX followerFlywheel = new TalonFX(Constants.CANID.kRightFlywheel);
  TalonFX leaderFlywheel = new TalonFX(Constants.CANID.kLeftFlywheel);

  // Chamber Beam Break Sensor
  DigitalInput beambreak = new DigitalInput(0);

  @AutoLogOutput(key = "Outtake/Raw Angle Setpoint")
  private double rawAngleSetpoint = OuttakeConstants.kMinAngle;

  @AutoLogOutput(key = "Outtake/Profiled Angle Setpoint")
  private double profiledAngleSetpoint = OuttakeConstants.kMinAngle;

  // Variables to store/log the setpoints
  @AutoLogOutput(key = "Outtake/RPM Setpoint")
  private double rpmSetpoint = 0;

  public OuttakeIO_Real() {

    // Set the right side to follow the left
    followerFlywheel.setControl(new StrictFollower(Constants.CANID.kLeftFlywheel));

    // Motor Controller Configurations

    // Configure the pivot motor
    var pivotConfigurator = pivotMotor.getConfigurator();
    var pivotConfigs = new TalonFXConfiguration();
    pivotConfigs.Feedback.SensorToMechanismRatio =
        1.0 / OuttakeConstants.kGearingPivot; // Sets default output to pivot rotations
    pivotConfigs.Slot0 = OuttakeConstants.kPivotSlot0; // PID Constants
    pivotConfigs.CurrentLimits = OuttakeConstants.kCurrentConfigs; // Current Limits
    pivotConfigs.MotionMagic = OuttakeConstants.kPivotMotionMagicConfig; // Motion Magic Constants
    pivotConfigurator.apply(pivotConfigs);

    // Pivot Status Signals
    var pivotPositionSignal = pivotMotor.getPosition();
    var pivotVelocitySignal = pivotMotor.getVelocity();
    var pivotTempSignal = pivotMotor.getDeviceTemp();
    var pivotVoltageSignal = pivotMotor.getMotorVoltage();
    var pivotCurrentSignal = pivotMotor.getSupplyCurrent();
    var pivotClosedLoopReferenceSignal = pivotMotor.getClosedLoopReference();

    pivotPositionSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
    pivotVelocitySignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
    pivotTempSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency / 4);
    pivotVoltageSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
    pivotCurrentSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
    pivotClosedLoopReferenceSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);

    pivotMotor.optimizeBusUtilization(); // Reduces CAN bus usage

    // Set the default pivot location
    pivotMotor.setPosition(Units.degreesToRotations(OuttakeConstants.kMinAngle));

    // Configure the leading flywheel motor
    var flywheelConfigurator = leaderFlywheel.getConfigurator();
    var flywheelConfigs = new TalonFXConfiguration();
    flywheelConfigs.Feedback.SensorToMechanismRatio =
        1.0 / OuttakeConstants.kGearingFlywheel; // Sets default output to flywheel rotations
    flywheelConfigs.Slot0 = OuttakeConstants.kFlyWheelSlot0; // PID Constants
    flywheelConfigs.CurrentLimits = OuttakeConstants.kCurrentConfigs; // Current Limits
    flywheelConfigurator.apply(flywheelConfigs);

    // Flywheel Status Signals
    var flywheelVelocitySignal = leaderFlywheel.getVelocity();
    var flywheelTempSignal = leaderFlywheel.getDeviceTemp();
    var flywheelVoltageSignal = leaderFlywheel.getMotorVoltage();
    var flywheelCurrentSignal = leaderFlywheel.getSupplyCurrent();
    var flywheelClosedLoopReferenceSignal = leaderFlywheel.getClosedLoopReference();

    flywheelVelocitySignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
    flywheelTempSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency / 4);
    flywheelVoltageSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
    flywheelCurrentSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
    flywheelClosedLoopReferenceSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);

    leaderFlywheel.optimizeBusUtilization(); // Reduces CAN bus usage

    // Feed the PID with default values
    changePivotSetpoint(OuttakeConstants.kMinAngle);
    changeFlywheelSetpoint(0);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {

    // Update the pivot inputs
    inputs.pivotMotorPosition =
        Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble()); // Degrees
    inputs.pivotMotorVelocity =
        Units.rotationsToDegrees(pivotMotor.getVelocity().getValueAsDouble()); // Degrees per second
    inputs.pivotMotorTemp = pivotMotor.getDeviceTemp().getValueAsDouble(); // Celcius
    inputs.pivotMotorVoltage = pivotMotor.getMotorVoltage().getValueAsDouble(); // Volts
    inputs.pivotMotorCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble(); // Amps

    // Update the flywheel inputs
    inputs.flywheelMotorVelocity = leaderFlywheel.getVelocity().getValueAsDouble() * 60; // RPM
    inputs.flywheelMotorTemp = leaderFlywheel.getDeviceTemp().getValueAsDouble(); // Celcius
    inputs.flywheelMotorVoltage = leaderFlywheel.getMotorVoltage().getValueAsDouble(); // Volts
    inputs.flywheelMotorCurrent = leaderFlywheel.getSupplyCurrent().getValueAsDouble(); // Amps

    // Update beambreak state
    inputs.beamBroken = !beambreak.get();

    // Updated profiled angle setpoint
    profiledAngleSetpoint =
        Units.rotationsToDegrees(pivotMotor.getClosedLoopReference().getValueAsDouble()); // Degrees
    rpmSetpoint = leaderFlywheel.getClosedLoopReference().getValueAsDouble() * 60; // RPM
  }

  /**
   * Change the setpoint of the shooter pivot
   *
   * @param angleDegrees The new setpoint in degrees
   *     <p>Acceptable Range: [-27.5, 152.25] Increase in angle moves the pivot towards the back of
   *     the robot
   */
  @Override
  public void changePivotSetpoint(double angleDegrees) {
    rawAngleSetpoint =
        MathUtil.clamp(angleDegrees, OuttakeConstants.kMinAngle, OuttakeConstants.kMaxAngle);
    pivotMotor.setControl(
        new MotionMagicVoltage(
            Units.degreesToRotations(rawAngleSetpoint))); // Degrees to Native Rotations
  }

  /**
   * Change the setpoint of the flywheel
   *
   * @param rpm The new setpoint in RPM (Rotations per minute)
   *     <p>Aceptable range: [-3190, 3190] Positive RPM the note towards the back of the robot
   */
  @Override
  public void changeFlywheelSetpoint(double rpm) {
    leaderFlywheel.setControl(
        new VelocityVoltage(
            MathUtil.clamp(rpmSetpoint, OuttakeConstants.kMinRpm, OuttakeConstants.kMaxRpm)
                / 60)); // RPM to Native Rotations per second
  }
}
