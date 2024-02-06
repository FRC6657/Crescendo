package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ClimbConstants.ClimberInformation;
import frc.robot.Constants.OuttakeConstants;

public class ClimberIO_Real implements ClimberIO {
  private TalonFX climbMoter;
  private double setpoint = 0.0;

  private Slot0Configs climbSlot0;
  private Slot1Configs climbSlot1;

  private MotionMagicVoltage climbRequest = new MotionMagicVoltage(0);

  public ClimberIO_Real(ClimberInformation info) {
    climbMoter = new TalonFX(info.id);

    var motorConfigurator = climbMoter.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();

    climbSlot0 = new Slot0Configs();
    climbSlot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    climbSlot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    climbSlot0.kP = 2; // A position error of 2.5 rotations results in 12 V output
    climbSlot0.kI = 0; // no output for integrated error
    climbSlot0.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    climbSlot1 = new Slot1Configs();
    climbSlot1.kS = 0.25; // Add 0.25 V output to overcome static friction
    climbSlot1.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    climbSlot1.kP = 2; // A position error of 2.5 rotations results in 12 V output
    climbSlot1.kI = 0; // no output for integrated error
    climbSlot1.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    motorConfigs.Feedback.SensorToMechanismRatio = 10; // TOTALY NOT CORRECT PLEASE FIX
    motorConfigs.CurrentLimits.StatorCurrentLimit =
        OuttakeConstants.kCurrentLimit; // TOTALY NOT CORRECT PLEASE FIX
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    motorConfigurator.apply(motorConfigs);
    motorConfigurator.apply(climbSlot0);
    motorConfigurator.apply(climbSlot1);
  }

  @Override
  public void run(double height) {
    climbMoter.setControl(climbRequest);
  }
}
