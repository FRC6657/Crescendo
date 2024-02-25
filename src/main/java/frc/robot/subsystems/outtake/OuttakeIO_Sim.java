package frc.robot.subsystems.outtake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.OuttakeConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class OuttakeIO_Sim implements OuttakeIO {

  // Variables to track voltage
  private double pivotVoltage = 0;
  private double shooterVoltage = 0;

  // Variables to track setpoints
  @AutoLogOutput(key = "Outtake/RPM Setpoint")
  private double rpmSetpoint = 0;

  @AutoLogOutput(key = "Outtake/Angle Setpoint")
  private double angleSetpoint = OuttakeConstants.kMinPivotAngle;

  // Simulated Motors
  private FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getFalcon500(2), 2, 0.00146376);
  private DCMotorSim pivotSim =
      new DCMotorSim(DCMotor.getFalcon500(1), 1 / Constants.OuttakeConstants.kGearingPivot, 0.04);

  // Pivot PID
  private PIDController pivotPID = new PIDController(24d / 360, 0, 0);

  // Flywheel PID + FF
  private SimpleMotorFeedforward flyWheelFeedForward = new SimpleMotorFeedforward(0, 12d / 3190);
  private PIDController flyWheelPID = new PIDController(0, 0, 0);

  public OuttakeIO_Sim() {
    pivotPID.setTolerance(2);
    flyWheelPID.setTolerance(50);
    pivotSim.setState(Units.degreesToRadians(OuttakeConstants.kMinPivotAngle), 0);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {

    updatePID(); // Update the PID controllers and their outputs

    // Update Simulations
    pivotSim.update(1 / CodeConstants.kMainLoopFrequency);
    flywheelSim.update(1 / CodeConstants.kMainLoopFrequency);

    // Pivot Inputs
    inputs.pivotMotorPosition = pivotSim.getAngularPositionRotations() * 360; // Degrees
    inputs.pivotMotorVelocity =
        pivotSim.getAngularVelocityRPM() * (360d / 60); // Degrees per second
    inputs.pivotMotorTemp = 0; // Celcius
    inputs.pivotMotorVoltage = pivotVoltage; // Volts
    inputs.pivotMotorCurrent = pivotSim.getCurrentDrawAmps(); // Amps
    inputs.pivotAtSetpoint = pivotPID.atSetpoint();

    // Flywheel Inputs
    inputs.flywheelMotorVelocity = flywheelSim.getAngularVelocityRPM(); // RPM
    inputs.flywheelMotorTemp = 0; // Celcius
    inputs.flywheelMotorVoltage = shooterVoltage; // Volts
    inputs.flywheelMotorCurrent = flywheelSim.getCurrentDrawAmps(); // Amps
    inputs.flywheelAtSetpoint = flyWheelPID.atSetpoint();

    // Beambreak state
    inputs.beamBroken = false; // Beambreak state
  }

  /**
   * Change the setpoint of the flywheel
   *
   * @param rpm The new setpoint in RPM (Rotations per minute)
   *     <p>Aceptable range: [-3190, 3190] Positive RPM the note towards the back of the robot
   */
  @Override
  public void changeFlywheelSetpoint(double rpm) {
    rpmSetpoint = rpm;
  }
  /**
   * Change the setpoint of the shooter pivot
   *
   * @param angleDegrees The new setpoint in degrees
   *     <p>Acceptable Range: [-27.5, 152.25] Increase in angle moves the pivot towards the back of
   *     the robot
   */
  @Override
  public void changePivotSetpoint(double angleDegrees) {
    angleSetpoint = angleDegrees;
  }

  /** Uses the current setpoints and the current states to calculate the output voltages. */
  private void updatePID() {
    // Pivot
    double pivotPIDEffort =
        pivotPID.calculate(pivotSim.getAngularPositionRotations() * 360, angleSetpoint);
    pivotVoltage = MathUtil.clamp(pivotPIDEffort, -12, 12);
    pivotSim.setInput(pivotVoltage);

    // Flywheel
    double ffEffort = flyWheelFeedForward.calculate(rpmSetpoint);
    double pidEffort = flyWheelPID.calculate(flywheelSim.getAngularVelocityRPM(), rpmSetpoint);
    shooterVoltage = MathUtil.clamp(ffEffort + pidEffort, -12, 12);
    flywheelSim.setInput(shooterVoltage);
  }
}
