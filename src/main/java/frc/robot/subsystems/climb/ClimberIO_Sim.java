package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.CodeConstants;

public class ClimberIO_Sim implements ClimberIO {

  private DCMotorSim climberSim = new DCMotorSim(DCMotor.getFalcon500(1), 12, 0.01);
  private PIDController climberPID = new PIDController(0.001, 0, 0);

  double voltage = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    climberSim.update(1 / CodeConstants.kMainLoopFrequency);

    inputs.position =
        climberSim.getAngularPositionRotations()
            * Constants.ElevatorConstants.kSensorToVerticalMeters;

    inputs.appliedVoltage = voltage;
  }

  @Override
  public void run(double height) {

    height =
        MathUtil.clamp(
            height, Constants.ElevatorConstants.kMinHeight, Constants.ElevatorConstants.kMaxHeight);

    double pidEffort =
        climberPID.calculate(
            climberSim.getAngularVelocityRPM()
                * Constants.ElevatorConstants.kSensorToVerticalMeters,
            height);

    voltage = MathUtil.clamp(pidEffort, -12, 12);
    climberSim.setInputVoltage(pidEffort);
  }
}
