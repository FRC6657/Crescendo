package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

  @AutoLog
  public static class OuttakeIOInputs {

    public double pivotMotorPosition = 0.0; //Degrees
    public double pivotMotorVelocity = 0.0; //Degrees per second
    public double pivotMotorTemp = 0.0; //Celcius
    public double pivotMotorVoltage = 0.0; //Volts
    public double pivotMotorCurrent = 0.0; //Amps

    public double flywheelMotorVelocity = 0.0; //RPM
    public double flywheelMotorTemp = 0.0; //Celcius
    public double flywheelMotorVoltage = 0.0; //Volts
    public double flywheelMotorCurrent = 0.0; //Amps
    
    public boolean beamBroken = false; //True if a note is blocking the beambreak

  }

  public default void updateInputs(OuttakeIOInputs inputs) {}

  public default void changePivotSetpoint(double angleDegrees) {}
  public default void changeFlywheelSetpoint(double rpm) {}


}
