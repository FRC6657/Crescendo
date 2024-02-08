package frc.robot.subsystems.climb;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ClimbConstants.ClimberInformation;
import frc.robot.Constants.ClimbConstants;

public class ClimberIO_Real implements ClimberIO {
  private TalonFX mMotor;
  private final PIDController mPID = ClimbConstants.kClimbPID;
  private double setpoint = 0.0;
  private double voltage = 0.0;


  private MotionMagicVoltage climbRequest = new MotionMagicVoltage(0);

  public ClimberIO_Real(ClimberInformation info) {
    mMotor = new TalonFX(info.id);

    var motorConfigurator = mMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();

    motorConfigs.CurrentLimits = ClimbConstants.kCurrentConfigs;
    motorConfigs.Feedback.SensorToMechanismRatio = ClimbConstants.kGearing;
    

    motorConfigurator.apply(motorConfigs);
  
  }

  private double getHight(){
    return mMotor.getPosition().getValueAsDouble()*ClimbConstants.kSprocketPD;
  }

  @Override
  public void changeSetpoint(double height){
    setpoint = height;
  }

  @Override
  public void run() {
    voltage = MathUtil.clamp(mPID.calculate(getHight(), setpoint), -12, 12);

    if(setpoint >= getHight()){
      voltage *= 0.5;
    }
    else{
      mMotor.set(voltage*0.5);
    }
    
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs){
    inputs.appliedVoltage = mMotor.getMotorVoltage().getValueAsDouble();
    inputs.position = Units.inchesToMeters(getHight());
    inputs.current = mMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocity = mMotor.getVelocity().getValueAsDouble();
  }

}
