package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.CodeConstants;

public class IntakeIO_Sim implements IntakeIO {

  private double voltage = 0;

  private DCMotorSim pivotSim = new DCMotorSim(DCMotor.getFalcon500(1), 12, 0.01);

  private PIDController pivotPID = new PIDController(0.001, 0, 0);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    pivotSim.update(1 / CodeConstants.kMainLoopFrequency);

    inputs.pivotMotorPosition = pivotSim.getAngularPositionRotations() * Constants.OuttakeConstants.kSensorToDegrees;
    inputs.pivotMotorVoltage = voltage;
    inputs.pivotMotorTemp = 0;
    inputs.pivotMotorCurrent = pivotSim.getCurrentDrawAmps();

  }

 @Override
  public void runPivot(double angle) {
    double pidEffort = pivotPID.calculate(pivotSim.getAngularPositionRotations() * Constants.OuttakeConstants.kSensorToDegrees, angle);
    voltage = MathUtil.clamp(pidEffort, -12, 12);
    pivotSim.setInput(voltage);
  }

}
