package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;

public class Climber {

  Solenoid level2ClimberUp;
  Solenoid level2ClimberDown;

  Solenoid traverseClimberUp;
  Solenoid traverseClimberDown;

  boolean isLevel2ClimberDown = true;
  boolean isTraverseClimberDown = true;

  CANSparkMax miraclePistonMotor = new CANSparkMax(8, MotorType.kBrushless);

    public void climberInit() {
      level2ClimberUp = Robot.ph.makeSolenoid(13);
      level2ClimberDown = Robot.ph.makeSolenoid(12);

      traverseClimberUp = Robot.ph.makeSolenoid(8);
      traverseClimberDown = Robot.ph.makeSolenoid(9);

      miraclePistonMotor.setInverted(true);
      miraclePistonMotor.setSmartCurrentLimit(35, 40);
    }

    public void climberTeleop() {

        if(Constants.xbox.getRawButtonPressed(7)){
          if(isLevel2ClimberDown){
            level2ClimberDown.set(false);
            level2ClimberUp.set(true);
            isLevel2ClimberDown = false;
          } else {
            level2ClimberUp.set(false);
            level2ClimberDown.set(true);
            isLevel2ClimberDown = true;
          }
        }

        if(Constants.xbox.getRawButtonPressed(8)){
          if(isTraverseClimberDown){
            traverseClimberDown.set(false);
            traverseClimberUp.set(true);
            isTraverseClimberDown = false;
          } else {
            traverseClimberUp.set(false);
            traverseClimberDown.set(true);
            isTraverseClimberDown = true;
          }
        }

        if(Constants.stick.getRawButton(7)){
           miraclePistonMotor.set(0.2);
        } else if(Constants.stick.getRawButton(8)){
          miraclePistonMotor.set(-0.2);
        } else {
          miraclePistonMotor.set(0.0);
          miraclePistonMotor.getPIDController().setReference(miraclePistonMotor.getEncoder().getPosition(), ControlType.kPosition);
        }
    }
}