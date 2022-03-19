package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake {

    CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorIndex, MotorType.kBrushed);
    CANSparkMax indexerMotor = new CANSparkMax(Constants.indexerMotorIndex, MotorType.kBrushless);

    Solenoid intakeUpSolenoid;
    Solenoid intakeDownSolenoid;

    static boolean isIntakeDown = false;

    I2C.Port i2cPort = I2C.Port.kOnboard;
    ColorSensorV3 indexerColorSensor = new ColorSensorV3(i2cPort);

    static boolean intaking = false;

    public void intakeInit() {
        intaking = false;

        intakeMotor.enableVoltageCompensation(12.5);
        indexerMotor.enableVoltageCompensation(12.5);

        intakeMotor.setSmartCurrentLimit(35, 40);

        intakeUpSolenoid = Robot.ph.makeSolenoid(11);
        intakeDownSolenoid = Robot.ph.makeSolenoid(10);
    }

    public void indexByColor() {
        Color detectedColor = indexerColorSensor.getColor();

        if (!Robot.shooter.shooting && !Robot.shooter.dumpShot) {
            // if ((Math.abs(detectedColor.red - 0.44) < 0.05) || (Math.abs(detectedColor.blue - 0.305) < 0.02)) {
            //     indexerMotor.set(0.0);
            // } else {
            //     indexerMotor.set(-0.2);
            // }
            if(((detectedColor.red > 0.44) && (detectedColor.red < 0.58)) || ((detectedColor.blue > 0.20) && (detectedColor.blue < 0.265))){
                indexerMotor.set(0.0);
            } else {
                indexerMotor.set(-0.2);
            }
        }
    }

    public void intakeUp(){
        intakeDownSolenoid.set(false);
        intakeUpSolenoid.set(true);
        isIntakeDown = false;
    }

    public void intakeDown(){
        intakeUpSolenoid.set(false);
        intakeDownSolenoid.set(true);
        isIntakeDown = true;
    }

    public void intakeTeleop() {

        if (isIntakeDown) {
            if (Constants.stick.getRawButton(10)) {
                intakeMotor.set(0.6);
                intaking = false;
            } else if (Constants.stick.getRawButtonPressed(1)) {
                intaking = !intaking;
                if (intaking) {
                    intakeMotor.set(-0.9);
                }
            } else {
                if (!intaking) {
                    intakeMotor.set(0.0);
                }
            }
        } else {
            intakeMotor.set(0.0);
        }

        if (!Robot.shooter.shooting && !Robot.shooter.dumpShot) {
            if (Constants.xbox.getPOV() == 0) {
                indexerMotor.set(-0.3);
            } else if (Constants.xbox.getPOV() == 180) {
                indexerMotor.set(0.3);
            } else { // either rewrite method in same way or just call method
                //indexByColor();
                indexerMotor.set(0.0);
            }
        }

        if(Constants.xbox.getRawButtonPressed(2)){
            if(isIntakeDown){
                intakeUp();
            } else {
                intakeDown();
            }
        }
    }
}