package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double currentSpeed = 0.0;
    public double rollerMotorTemp = 0.0;
    public double rollerMotorVoltage = 0.0;
    public double rollerMotorCurrent = 0.0;
    public double rollerMotorAcceleration = 0.0; // in motor rotations per second /s

    public double pivotMotorPosition = 0.0;
    public double pivotMotorTemp = 0.0;
    public double pivotMotorVoltage = 0.0;
    public double pivotMotorCurrent = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runRollers(double speed) {}

  public default void runPivot(double angle) {}
}
