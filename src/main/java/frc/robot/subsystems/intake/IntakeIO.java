package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {

    public double pivotMotorPosition = 0.0; // Degrees
    public double pivotMotorVelocity = 0.0; // Degrees per second
    public double pivotMotorTemp = 0.0; // Celcius
    public double pivotMotorVoltage = 0.0; // Volts
    public double pivotMotorCurrent = 0.0; // Amps
    public boolean atSetpoint = false;

    public double rollerMotorVelocity = 0.0; // RPM
    public double rollerMotorAcceleration = 0.0; // RPM per second
    public double rollerMotorTemp = 0.0; // Celcius
    public double rollerMotorVoltage = 0.0; // Volts
    public double rollerMotorCurrent = 0.0; // Amps

    public boolean beamBroken = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void changePivotSetpoint(double angleDegrees) {}

  public default void changeRollerSpeed(double speed) {}
}
