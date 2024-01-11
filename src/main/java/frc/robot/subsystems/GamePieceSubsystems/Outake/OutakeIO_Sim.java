package frc.robot.subsystems.GamePieceSubsystems.Outake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.CodeConstants;



public class OutakeIO_Sim implements OutakeIO {
    private double voltage = 0;

    private FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getFalcon500(2), 2, 0.01);

    private PIDController flyWheelController = new PIDController(12d/3190, 0.0, 0.0);
    private SimpleMotorFeedforward flyWheelFeedForward =
      new SimpleMotorFeedforward(0.0, 12d/3190);
    @Override
    public void updateInputs(OutakeIOInputs inputs) {

        flywheelSim.update(1 / CodeConstants.kMainLoopFrequency);

        inputs.currentRPM = flywheelSim.getAngularVelocityRPM();
        inputs.motorVoltage = voltage; 
        inputs.motorTemperature = 0;
        inputs.motorCurrent = flywheelSim.getCurrentDrawAmps();
    }

    @Override
  public void run(double rpm) {
    rpm = MathUtil.clamp(rpm, -3190, 3190);
    double ffEffort = flyWheelFeedForward.calculate(rpm);
    double pidEffort = flyWheelController.calculate(flywheelSim.getAngularVelocityRPM(), rpm);
    voltage = MathUtil.clamp(pidEffort,-12,12);
    flywheelSim.setInput(voltage);
  }
}
