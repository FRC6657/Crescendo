package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.CodeConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberIO_Sim implements ClimberIO {

  private DCMotorSim climberSim = new DCMotorSim(DCMotor.getFalcon500(1), 12, 0.01);
  private PIDController climberPID = new PIDController(90, 0, 0);

  double voltage = 0.0;
  double setpoint = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    climberSim.update(1 / CodeConstants.kMainLoopFrequency);

    inputs.position =
        climberSim.getAngularPositionRotations() * Constants.ClimbConstants.kSensorToVerticalMeters;

    inputs.appliedVoltage = voltage;

    inputs.atSetpoint = MathUtil.isNear(setpoint, inputs.position, 0.25);
  }

  @Override
  public void changeSetpoint(double height) {
    setpoint = height;
  }

  @Override
  public void run() {

    double pidEffort =
        climberPID.calculate(
            climberSim.getAngularPositionRotations()
                * Constants.ClimbConstants.kSensorToVerticalMeters,
            setpoint);

    voltage = MathUtil.clamp(pidEffort, -12, 12);
    climberSim.setInputVoltage(pidEffort);

    Logger.recordOutput("setpoint", setpoint);
  }
}
