package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANID;
import frc.robot.Constants.CodeConstants;

public class GyroIO_Real implements GyroIO {

  private final Pigeon2 pigeon = new Pigeon2(CANID.kPigeon);
  private final StatusSignal<Double> yaw = pigeon.getYaw();
  private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZDevice();

  /** Gyro IO for real robot */
  public GyroIO_Real() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration()); // Restore Factory Defaults
    pigeon.getConfigurator().setYaw(0);
    yaw.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
    yawVelocity.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }
}
