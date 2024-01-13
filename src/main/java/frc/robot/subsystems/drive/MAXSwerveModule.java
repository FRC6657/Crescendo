package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class MAXSwerveModule {

  // MAXSwerve IO
  private final MAXSwerveIO io;
  private final MAXSwerveIOInputsAutoLogged inputs = new MAXSwerveIOInputsAutoLogged();

  // Name to identify the module
  public final String name;

  public MAXSwerveModule(MAXSwerveIO io, String name) {
    this.io = io;
    this.name = name;
  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/" + name + " Module", inputs);
  }

  /**
   * Sets the desired state of the module
   *
   * @param desiredState The desired state of the module
   * @return The optimized state of the module for reference
   */
  public SwerveModuleState run(SwerveModuleState desiredState) {
    var optimizedState = SwerveModuleState.optimize(desiredState, inputs.turnPositionRad);
    io.setDriveMPS(optimizedState.speedMetersPerSecond);
    io.setTurnAngle(optimizedState.angle);
    return optimizedState;
  }

  /** Logs the module IO */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/" + name + " Module", inputs);
  }

  public void stop() {
    io.setDriveMPS(0);
  }

  public Rotation2d getTurnPosition() {
    return inputs.turnPositionRad;
  }

  public double getDrivePositionMeters() {
    return inputs.drivePositionMeters;
  }

  public double getDriveVelocityMPS() {
    return inputs.driveVelocityMPS;
  }

  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(getDrivePositionMeters(), getTurnPosition());
  }

  public SwerveModuleState getSwerveModuleState() {
    return new SwerveModuleState(getDriveVelocityMPS(), getTurnPosition());
  }
}
