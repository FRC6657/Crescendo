package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.CANID;

// falcon motors

public class OuttakeIO_Real implements OuttakeIO {
  private TalonFX OuttakeWeelsMotor = new TalonFX(CANID.kOutakeWeels);

  public OuttakeIO_Real() {
    // motor configureation stuff here
  }
}
