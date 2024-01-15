package frc.robot.subsystems.climb;

public class ClimberIO_Sim implements ClimberIO {

  double height = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    inputs.position = height;

  }

  @Override
  public void run(double height) {
    
  }

}
