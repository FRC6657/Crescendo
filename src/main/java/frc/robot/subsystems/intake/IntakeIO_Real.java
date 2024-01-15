package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.CANID;

// falcon motors

public class IntakeIO_Real implements IntakeIO {
  private TalonFX OuttakeWeelsMotor = new TalonFX(CANID.kOutakeWeels);

  public IntakeIO_Real() {
    // motor configureation stuff here
  }
}
