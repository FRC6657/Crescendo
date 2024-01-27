package frc.robot.subsystems.outtake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.CodeConstants;

public class OuttakeIO_Sim implements OuttakeIO {
  private double sVoltage = 0;
  private double pVoltage = 0;

  private FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getFalcon500(2), 2, 0.00146376);
  private DCMotorSim pivotSim = new DCMotorSim(DCMotor.getFalcon500(1), 1 / Constants.OuttakeConstants.kGearing, 0.01);

  private SimpleMotorFeedforward flyWheelFeedForward =
      new SimpleMotorFeedforward(0.1, 0.00374064837); // 0.003639801
  private PIDController flyWheelPID = new PIDController(0.032, 0, 0);

  private PIDController pivotPID = new PIDController(12d / 360, 0, 0);

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {

    flywheelSim.update(1 / CodeConstants.kMainLoopFrequency);
    pivotSim.update(1 / CodeConstants.kMainLoopFrequency);

    inputs.currentRPM = flywheelSim.getAngularVelocityRPM();
    inputs.flywheelMotorVoltage = sVoltage;
    inputs.flywheelMotorTemp = 0;
    inputs.flywheelMotorCurrent = flywheelSim.getCurrentDrawAmps();

    inputs.pivotMotorPosition =
        pivotSim.getAngularPositionRotations() * 360;
    inputs.pivotMotorVoltage = pVoltage;
    inputs.pivotMotorTemp = 0;
    inputs.pivotMotorCurrent = pivotSim.getCurrentDrawAmps();
  }

  @Override
  public void runFlywheel(double rpm) {
    rpm = MathUtil.clamp(rpm, -3190, 3190);
    double ffEffort = flyWheelFeedForward.calculate(rpm);
    double pidEffort = flyWheelPID.calculate(flywheelSim.getAngularVelocityRPM(), rpm);
    sVoltage = MathUtil.clamp(ffEffort + pidEffort, -12, 12);
    flywheelSim.setInput(sVoltage);
  }

  @Override
  public void runPivot(double angle) {
    double pidEffort =
        pivotPID.calculate(
            pivotSim.getAngularPositionRotations() * 360,
            angle);
    pVoltage = MathUtil.clamp(pidEffort, -12, 12);
    pivotSim.setInput(pVoltage);
  }
}
