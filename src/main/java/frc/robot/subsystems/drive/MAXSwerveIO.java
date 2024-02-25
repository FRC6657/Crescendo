package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Generic MAXSwerve IO interface */
public interface MAXSwerveIO {

  @AutoLog
  public static class MAXSwerveIOInputs {
    public double drivePositionMeters = 0.0;
    public double driveVelocityMPS = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public Rotation2d turnPositionRad = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnError = 0.0;
  }

  public default void updateInputs(MAXSwerveIOInputs inputs) {}

  public default void setDriveMPS(double mps) {}

  public default void setTurnAngle(Rotation2d angle) {}

  public default void stop() {}

  public default Rotation2d getTurnAngle() {
    return new Rotation2d();
  }
}
