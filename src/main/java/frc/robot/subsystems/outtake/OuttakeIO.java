package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

  @AutoLog
  public static class OuttakeIOInputs {
    public double currentRPM = 0.0;
    public double motorTemperature = 0.0;
    public double motorVoltage = 0.0;
    public double motorCurrent = 0.0;
  }

  public default void updateInputs(OuttakeIOInputs inputs) {}

  public default void run(double rpm) {}
}
