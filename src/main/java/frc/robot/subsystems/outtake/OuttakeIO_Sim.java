package frc.robot.subsystems.outtake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.CodeConstants;

public class OuttakeIO_Sim implements OuttakeIO {
  private double voltage = 0;
  private double angle = 0;

  private FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getFalcon500(2), 2, 0.00146376);

  private SimpleMotorFeedforward flyWheelFeedForward =
      new SimpleMotorFeedforward(0.1, 0.00374064837); // 0.003639801
  private PIDController flyWheelPID = new PIDController(0.0032, 0, 0);

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {

    flywheelSim.update(1 / CodeConstants.kMainLoopFrequency);

    inputs.currentRPM = flywheelSim.getAngularVelocityRPM();
    inputs.flywheelMotorVoltage = voltage;
    inputs.flywheelMotorTemp = 0;
    inputs.flywheelMotorCurrent = flywheelSim.getCurrentDrawAmps();

    inputs.pivotMotorPosition = angle;
  }

  @Override
  public void runFlywheel(double rpm) {
    rpm = MathUtil.clamp(rpm, -3190, 3190);
    double ffEffort = flyWheelFeedForward.calculate(rpm);
    double pidEffort = flyWheelPID.calculate(flywheelSim.getAngularVelocityRPM(), rpm);
    voltage = MathUtil.clamp(ffEffort + pidEffort, -12, 12);
    flywheelSim.setInput(voltage);
  }

  @Override
  public void runPivot(double angle) {
    this.angle = angle;
  }
}
