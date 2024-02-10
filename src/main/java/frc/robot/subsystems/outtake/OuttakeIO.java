package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

  @AutoLog
  public static class OuttakeIOInputs {
    public double currentRPM = 0.0;
    public double flywheelMotorTemp = 0.0;
    public double flywheelMotorVoltage = 0.0;
    public double flywheelMotorCurrent = 0.0;

    public double pivotMotorPosition = 0.0;
    public double pivotMotorTemp = 0.0;
    public double pivotMotorVoltage = 0.0;
    public double pivotMotorCurrent = 0.0;
  }

  public default void updateInputs(OuttakeIOInputs inputs) {}

  public default void changeFlywheel(double rpm) {}

  public default void changePivot(double angle) {}

  public default boolean breakbeam() {
    return true;
  }
}
