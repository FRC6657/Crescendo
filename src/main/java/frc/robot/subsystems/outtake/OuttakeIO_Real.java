package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class OuttakeIO_Real implements OuttakeIO {

  TalonFX rightShooter = new TalonFX(Constants.CANID.kRightShooter); 
  TalonFX leftShooter = new TalonFX(Constants.CANID.kLeftShooter);
  
  TalonFX pivot = new TalonFX(Constants.CANID.kShooterPivot);

  private double voltage = 0;
  private double angle = 0.0;

  public OuttakeIO_Real() {
    
    var shooterPivotConfigurator = pivot.getConfigurator();
    var shooterPivotConfigs = new TalonFXConfiguration();

    shooterPivotConfigs.Feedback.SensorToMechanismRatio = Constants.OuttakeConstants.kSensorToDegrees;
    
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 2; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    pivot.getConfigurator().apply(slot0Configs);

    final TrapezoidProfile mProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(360, 2880)
    );

    TrapezoidProfile.State mGoal = new TrapezoidProfile.State(180, 0);
    TrapezoidProfile.State mSetpoint = new TrapezoidProfile.State();

    final PositionVoltage mRequest = new PositionVoltage(0).withSlot(0);

    mSetpoint = mProfile.calculate(0.020, mSetpoint, mGoal);

    mRequest.Position = mSetpoint.position;
    mRequest.Velocity = mSetpoint.velocity;
    
    pivot.setControl(mRequest.withPosition(mSetpoint.position).withFeedForward(0.5));

    shooterPivotConfigurator.apply(shooterPivotConfigs);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {

    inputs.pivotMotorPosition = angle;
    inputs.flywheelMotorVoltage = voltage;
    
    pivot.getPosition();

  }

  
}
