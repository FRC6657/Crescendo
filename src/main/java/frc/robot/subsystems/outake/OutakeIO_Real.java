package frc.robot.subsystems.outake;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.CANID;

// falcon motors

public class OutakeIO_Real implements OutakeIO {
  private TalonFX OutakeWeelsMotor = new TalonFX(CANID.kOutakeWeels);

  public OutakeIO_Real() {
    // motor configureation stuff here
  }
}
