package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class IntakeIO_Sim implements IntakeIO {

  // Variables to track voltage
  private double pivotVoltage = 0;
  private double rollerVoltage = 0;

  // Variables to track setpoints
  @AutoLogOutput(key = "Intake/Speed Setpoint")
  private double speedSetpoint = 0;

  @AutoLogOutput(key = "Intake/Angle Setpoint")
  private double angleSetpoint = IntakeConstants.kMaxPivotAngle;

  // Simulated Motors
  private DCMotorSim pivotSim =
      new DCMotorSim(DCMotor.getFalcon500(1), 1 / IntakeConstants.kGearingPivot, 0.04);
  private DCMotorSim rollerSim =
      new DCMotorSim(DCMotor.getFalcon500(1), IntakeConstants.kGearingRollers, 0.00146376);

  // Pivot PID
  private PIDController pivotPID = new PIDController(96d / 360, 0, 0);

  public IntakeIO_Sim() {
    pivotSim.setState(Units.degreesToRadians(IntakeConstants.kMaxPivotAngle), 0);
    pivotPID.setTolerance(2);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    updatePID(); // Update the PID controllers and their outputs

    // Update Simulations
    pivotSim.update(1 / CodeConstants.kMainLoopFrequency);
    rollerSim.update(1 / CodeConstants.kMainLoopFrequency);

    // Pivot Inputs
    inputs.pivotMotorPosition = pivotSim.getAngularPositionRotations() * 360; // Degrees
    inputs.pivotMotorVelocity =
        pivotSim.getAngularVelocityRPM() * (360d / 60); // Degrees per second
    inputs.pivotMotorTemp = 0; // Celcius
    inputs.pivotMotorVoltage = pivotVoltage; // Volts
    inputs.pivotMotorCurrent = pivotSim.getCurrentDrawAmps(); // Amps
    inputs.atSetpoint = pivotPID.atSetpoint();
    inputs.pivotMotorSetpoint = angleSetpoint;

    // Roller Inputs
    inputs.rollerMotorVelocity = rollerSim.getAngularVelocityRPM(); // RPM
    inputs.rollerMotorTemp = 0; // Celcius
    inputs.rollerMotorVoltage = rollerVoltage; // Volts
    inputs.rollerMotorCurrent = rollerSim.getCurrentDrawAmps(); // Amps]

    inputs.tofDistance = 27;
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

  /**
   * Change the setpoint of the rollers
   *
   * @param rpm The new Duty Cycle setpoint for the rollers
   *     <p>Aceptable range: [-1, 1] Positive RPM the note towards the back of the robot
   */
  @Override
  public void changeRollerSpeed(double speed) {
    rollerVoltage = speed;
  }

  /** Uses the current setpoints and the current states to calculate the output voltages. */
  private void updatePID() {
    // Pivot
    double pivotPIDEffort =
        pivotPID.calculate(pivotSim.getAngularPositionRotations() * 360, angleSetpoint);
    pivotVoltage = MathUtil.clamp(pivotPIDEffort, -12, 12);
    pivotSim.setInput(pivotVoltage);

    // Rollers
    rollerVoltage = speedSetpoint;
    rollerSim.setInput(speedSetpoint * RobotController.getBatteryVoltage());
  }
}
