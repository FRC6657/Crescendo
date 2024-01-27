package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
    public double appliedVoltage = 0.0;
    public double current = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void run(double height) {}
}