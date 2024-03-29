package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeIO_Real implements IntakeIO {

  // Pivot Motor Controller
  TalonFX pivotMotor = new TalonFX(Constants.CANID.kIntakePivot);

  // Roller Motor Controller
  TalonFX rollerMotor = new TalonFX(Constants.CANID.kIntakeRollers);

  // Intake TOF Sensor
  LaserCan sensor = new LaserCan(Constants.CANID.kIntakeTOF);

  // Variables to store/log the setpoints
  @AutoLogOutput(key = "Intake/Angle Setpoint")
  private double angleSetpoint = IntakeConstants.kMaxPivotAngle;

  @AutoLogOutput(key = "Intake/Speed Setpoint")
  private double speedSetpoint = 0;

  private DutyCycleOut rollerSetpoint = new DutyCycleOut(0);
  private MotionMagicVoltage pivotSetpoint = new MotionMagicVoltage(IntakeConstants.kMaxPivotAngle);

  public IntakeIO_Real() {

    // Motor Controller Configurations

    // Configure the pivot motor
    var pivotConfigurator = pivotMotor.getConfigurator();
    var pivotConfigs = new TalonFXConfiguration();
    pivotConfigs.Feedback.SensorToMechanismRatio =
        1.0 / IntakeConstants.kGearingPivot; // Sets default output to pivot rotations
    pivotConfigs.Slot0 = IntakeConstants.kPivotSlot0; // PID Constants
    pivotConfigs.CurrentLimits = IntakeConstants.kPivotCurrentConfigs; // Current Limits
    pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    pivotConfigs.MotionMagic = IntakeConstants.kPivotMotionMagicConfig;
    pivotConfigurator.apply(pivotConfigs);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);

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
    pivotMotor.setPosition(Units.degreesToRotations(IntakeConstants.kMaxPivotAngle));

    // Configure the leading roller motor
    var rollerConfigurator = rollerMotor.getConfigurator();
    var rollerConfigs = new TalonFXConfiguration();
    rollerConfigs.Feedback.SensorToMechanismRatio =
        1.0 / IntakeConstants.kGearingRollers; // Sets default output to roller rotations
    rollerConfigs.CurrentLimits = IntakeConstants.kRollersCurrentConfigs; // Current Limits
    rollerConfigurator.apply(rollerConfigs);

    // Roller Status Signals
    var rollerVelocitySignal = rollerMotor.getVelocity();
    var rollerTempSignal = rollerMotor.getDeviceTemp();
    var rollerVoltageSignal = rollerMotor.getMotorVoltage();
    var rollerCurrentSignal = rollerMotor.getSupplyCurrent();

    rollerVelocitySignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
    rollerTempSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency / 4);
    rollerVoltageSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
    rollerCurrentSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);

    rollerMotor.optimizeBusUtilization(); // Reduces CAN bus usage

    rollerMotor.setInverted(true);
    rollerMotor.setNeutralMode(NeutralModeValue.Brake);

    // Feed the PID with default values
    changePivotSetpoint(IntakeConstants.kMaxPivotAngle);
    changeRollerSpeed(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    // Update the pivot inputs
    inputs.pivotMotorPosition =
        Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble()); // Degrees
    inputs.pivotMotorVelocity =
        Units.rotationsToDegrees(pivotMotor.getVelocity().getValueAsDouble()); // Degrees per second
    inputs.pivotMotorTemp = pivotMotor.getDeviceTemp().getValueAsDouble(); // Celcius
    inputs.pivotMotorVoltage = pivotMotor.getMotorVoltage().getValueAsDouble(); // Volts
    inputs.pivotMotorCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble(); // Amps
    inputs.atSetpoint = MathUtil.isNear(angleSetpoint, inputs.pivotMotorPosition, 2);
    inputs.pivotMotorSetpoint = angleSetpoint;

    // Update the roller inputs
    inputs.rollerMotorVelocity = rollerMotor.getVelocity().getValueAsDouble() * 60; // RPM
    inputs.rollerMotorTemp = rollerMotor.getDeviceTemp().getValueAsDouble(); // Celcius
    inputs.rollerMotorVoltage = rollerMotor.getMotorVoltage().getValueAsDouble(); // Volts
    inputs.rollerMotorCurrent = rollerMotor.getSupplyCurrent().getValueAsDouble(); // Amps

    rollerMotor.setControl(rollerSetpoint.withOutput(speedSetpoint));
    pivotMotor.setControl(
        pivotSetpoint.withPosition(
            Units.degreesToRotations(angleSetpoint))); // Degrees to Native Rotations


    
    var measurement = sensor.getMeasurement();
    Logger.recordOutput("Intake/TOFStatus", measurement.status);

    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
      inputs.tofDistance = Units.metersToInches(sensor.getMeasurement().distance_mm * 0.001);
      inputs.tofUnplugged = false;
    }else{
      inputs.tofDistance = -1;
      inputs.tofUnplugged = true;
    }

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
    angleSetpoint = angleDegrees;
  }

  /**
   * Change the setpoint of the roller
   *
   * @param rpm The new setpoint in RPM (Rotations per minute)
   *     <p>Aceptable range: [-1, 1] Positive speed moves the note towards the pivot point
   */
  @Override
  public void changeRollerSpeed(double speed) {
    speedSetpoint = speed;
  }
}
