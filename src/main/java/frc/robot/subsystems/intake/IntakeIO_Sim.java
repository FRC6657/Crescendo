package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CodeConstants;

public class IntakeIO_Sim implements IntakeIO {

  private double pivotVoltage = 0;
  private double rollerSpeed = 0;

  private DCMotorSim pivotSim = new DCMotorSim(DCMotor.getFalcon500(1), 12, 0.01);//double check gearing when the gearboxes are finalised
  private DCMotorSim rollerSim = new DCMotorSim(DCMotor.getFalcon500(1), 24.0/11.0, 0.01);

  private PIDController pivotPID = new PIDController(0.001, 0, 0);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
//pivot
    pivotSim.update(1 / CodeConstants.kMainLoopFrequency);

    inputs.pivotMotorPosition =
        pivotSim.getAngularPositionRotations() * 360;
    inputs.pivotMotorVoltage = pivotVoltage;
    inputs.pivotMotorTemp = 0;
    inputs.pivotMotorCurrent = pivotSim.getCurrentDrawAmps();
    
//roller
    inputs.currentSpeed = rollerSpeed;
    inputs.rollerMotorTemp = 0.0;
    inputs.rollerMotorVoltage = rollerSpeed * 12;
    inputs.rollerMotorCurrent = rollerSim.getCurrentDrawAmps();
  }

  @Override
  public void runPivot(double angle) {
    double pidEffort =
        pivotPID.calculate(
            pivotSim.getAngularPositionRotations() * 360,
            angle);
    pivotVoltage = MathUtil.clamp(pidEffort, -12, 12);
    pivotSim.setInput(pivotVoltage);
  }

  @Override
  public void runRollers(double speed){
    rollerSpeed = speed;
    rollerSim.setInput(MathUtil.clamp(rollerSpeed, -1, 1) * 12);
  }
}
