package frc.robot.subsystems.outake;

import org.littletonrobotics.junction.AutoLog;

public interface OutakeIO {

  @AutoLog
  public static class OutakeIOInputs {
    public double currentRPM = 0.0;
    public double motorTemperature = 0.0;
    public double motorVoltage = 0.0;
    public double motorCurrent = 0.0;
  }

  public default void updateInputs(OutakeIOInputs inputs) {}

  public default void run(double rpm) {}
}
