package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;

public class Robot extends TimedRobot {

  static Shooter shooter = new Shooter();
  static DriveTrain drive = new DriveTrain();
  static Intake intake = new Intake();
  static Climber climber = new Climber();
  static PowerDistribution PDP = new PowerDistribution(6, ModuleType.kRev);

  static PneumaticHub ph = new PneumaticHub(15);

  public void clearStickyFaults() {
    PDP.clearStickyFaults();
    ph.clearStickyFaults();

    shooter.masterShooterMotor.clearStickyFaults();
    shooter.slaveShooterMotor.clearStickyFaults();
    shooter.hoodMotor.clearFaults();

    drive.flMotor.clearFaults();
    drive.frMotor.clearFaults();
    drive.blMotor.clearFaults();
    drive.brMotor.clearFaults();

    intake.intakeMotor.clearFaults();
    intake.indexerMotor.clearFaults();
  }

  static boolean isRed;

  int autoStage = 0;
  Timer autonomousTimer = new Timer();

  ///////////////////////////////////////////////////////
  // Robot

  @Override
  public void robotInit() {
    clearStickyFaults();
    drive.driveTrainInit();
    shooter.shooterRobotInit();
    climber.climberInit();
    intake.intakeInit();

    if (DriverStation.getAlliance().equals(Alliance.Red)) {
      isRed = true;
    } else {
      isRed = false;
    }
  }

  @Override
  public void robotPeriodic() {
  }

  ////////////////////////////////////////////////////////
  // Autonomous

  @Override
  public void autonomousInit() {
    clearStickyFaults();
    autonomousTimer.reset();
    autonomousTimer.start();
    autoStage = 0;
    shooter.shooterInit();
    shooter.resetHoodEncoders();
    intake.isIntakeDown = false;
    climber.isLevel2ClimberDown = true;
    climber.isTraverseClimberDown = true;
    ph.disableCompressor();
  }

  @Override
  public void autonomousPeriodic(){
    shooter.shoot();
    shooter.dumpShot();
    shooter.shooterIdle(0.25);
    System.out.println(autonomousTimer.get());

    switch(autoStage){

      case 0 : {
        autoStage = 1;
        drive.stopMotors();
        drive.resetDriveTrainEncoders();
        shooter.shootInit();
        intake.intakeDown();
        break;
      }


      case 1 : {
        drive.driveTrainByInches(8.0, 5);
        shooter.moveHood(30.0);
        climber.miraclePistonMotor.set(0.6);
        if(autonomousTimer.get() > 0.5){
          autoStage = 2;
          climber.miraclePistonMotor.set(0.0);
          climber.miraclePistonMotor.getPIDController().setReference(climber.miraclePistonMotor.getEncoder().getPosition(), ControlType.kPosition);
          intake.intakeMotor.set(-0.65);
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
          shooter.shootInit();
        }
        break;
      }

      case 2 : {
        shooter.moveHood(30.0);
        drive.driveTrainByInches(90.0, 0);
        if(autonomousTimer.get() > 2.45){
          autoStage = 3;
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
          shooter.shootInit();
        }
        break;
      }

      case 3 : {
        if(shooter.centerRobotOnTarget()){
          shooter.shooting = true;
        } else {
          shooter.shooting = false;
        }
        if(autonomousTimer.get() > 6.2){
          autoStage = 4;
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
          shooter.shooting = false;
        }
        break;
        // if(autonomousTimer.get() > ){
        //   autoStage = 5;
        //   drive.stopMotors();
        //   drive.resetDriveTrainEncoders();
        //   shooter.dumpShot = false;
        // }
      }

      case 4 : {
        drive.driveTrainByInches(9.0, 5);
        if(autonomousTimer.get() > 7.0){
          autoStage = 5;
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
        }
        break;
      }

      case 5 : {
        drive.driveTrainByInches(120.0, 0);
        if(autonomousTimer.get() > 10.3){
          autoStage = 6;
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
        }
        break;
      }

      case 6 : {
        drive.driveTrainByInches(75.0, 1);
        if(autonomousTimer.get() > 12.5){
          autoStage = 7;
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
          shooter.shootInit();
        }
        break;
      }

      case 7 : {
        if(shooter.centerRobotOnTarget()){
          shooter.shooting = true;
        } else {
          shooter.shooting = false;
        }
        break;
      }

      // case 4 : {
      //   if(shooter.centerRobotOnTarget()){
      //     shooter.dumpShot = true;
      //   } else {
      //     shooter.dumpShot = false;
      //   }
      //   if(autonomousTimer.get() > 7.5){
      //     autoStage = 5;
      //     drive.stopMotors();
      //     drive.resetDriveTrainEncoders();
      //     shooter.dumpShot = false;
      //   }
      //   break;
      // }

      // case 5 : {
      //   drive.driveTrainByInches(7.5, 5);
      //   if(autonomousTimer.get() > 7.85){
      //     autoStage = 6;
      //     drive.stopMotors();
      //     drive.resetDriveTrainEncoders();
      //   }
      //   break;
      // }

      // case 6 : {
      //   drive.driveTrainByInches(205.0, 0);
      //   if(autonomousTimer.get() > 12.2){
      //     autoStage = 7;
      //     drive.stopMotors();
      //     drive.resetDriveTrainEncoders();
      //   }
      //   break;
      // }

      // case 7 : {
      //   drive.driveTrainByInches(150.0, 1);
      //   if(autonomousTimer.get() > 14.0){
      //     autoStage = 8;
      //     drive.stopMotors();
      //     drive.resetDriveTrainEncoders();
      //     shooter.shootInit();
      //   }
      //   break;
      // }

      // case 8 : {
      //   if(shooter.centerRobotOnTarget()){
      //     shooter.shooting = true;
      //   } else {
      //     shooter.shooting = false;
      //   }
      //   break;
      // }
    }

  }

  ///////////////////////////////////////////////////////////
  // Tele-operated

  @Override
  public void teleopInit() {
    clearStickyFaults();
    shooter.shooterInit();
    intake.isIntakeDown = true;
    climber.isLevel2ClimberDown = true;
    climber.isTraverseClimberDown = true;
  }

  @Override
  public void teleopPeriodic() {
    //System.out.println(shooter.getXDistanceFromCenterOfHub(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));

    if (Constants.stick.getRawButton(2)) {
      shooter.centerRobotOnTarget();
    } else {
      drive.driveTrainByControls(Constants.stick.getRawAxis(1), Constants.stick.getRawAxis(0),
          Constants.stick.getRawAxis(2), false);
    }
    intake.intakeTeleop();

    shooter.shoot();
    shooter.dumpShot();
    shooter.shooterIdle(0.0);
    shooter.hoodControl();

    
    climber.climberTeleop();
  }

  /////////////////////////////////////////////////////////////
  // Test
  

  @Override
  public void testInit() {

    shooter.shooterInit();
    autonomousTimer.reset();
    autonomousTimer.start();
    ph.enableCompressorAnalog(115, 120);
  }

  @Override
  public void testPeriodic() {
    // if(Constants.stick.getRawButton(1)){
    //   canSparkMax.set(0.2);
    // } else if(Constants.stick.getRawButton(2)){
    //   canSparkMax.set(-0.2);
    // } else {
    //   canSparkMax.set(0.0);
    //   canSparkMax.getPIDController().setReference(canSparkMax.getEncoder().getPosition(), ControlType.kPosition);
    // }

    // System.out.println("intakeUpSolenoid: " + Robot.intake.intakeUp.get());
    // System.out.println("intakeDownSolenoid: " + Robot.intake.intakeDown.get());




    // if(Constants.stick.getRawButtonPressed(1)){
    //   Robot.intake.intakeUp.set(false);
    //   Robot.intake.intakeDown.set(true);
    // }

    // if(Constants.stick.getRawButtonPressed(2)){
    //   Robot.intake.intakeDown.set(false);
    //   Robot.intake.intakeUp.set(true);
    // }

    // intake.intakeAngleMotor.set(0.2);
    // System.out.println(intake.intakeAngleMotor.get());
    //shooter.masterShooterMotor.set(ControlMode.Velocity, 0.2, DemandType.ArbitraryFeedForward, 0.5);
    //shooter.masterShooterMotor.pid
    // shooter.masterShooterMotor.set(ControlMode.Velocity, 1500.0);
    // System.out.println(shooter.masterShooterMotor.getSelectedSensorVelocity(1));

    // intake.indexByColor();

    // Color detectedColor = intake.indexerColorSensor.getColor();
    // System.out.println(
    //     "red: " + detectedColor.red + ",     blue: " + detectedColor.blue + ",     green: " + detectedColor.green);

    //     // if (!Robot.shooter.shooting && !Robot.shooter.dumpShot) {
    //     //     if ((Math.abs(detectedColor.red - 0.32) < 0.02) || (Math.abs(detectedColor.blue - 0.305) < 0.02)) {
    //     //         intake.indexerMotor.set(0.0);
    //     //     } else {
    //     //         intake.indexerMotor.set(-0.2);
    //     //     }
    //     // }

    //     intake.indexByColor();

    

    //System.out.println(doubleSolenoid.get());

    // if(Constants.stick.getRawButton(1)){
    //   intake.intakeAngleMotor.set(-0.5);
    // } else if (Constants.stick.getRawButton(2)) {
    //   intake.intakeAngleMotor.set(0.5);
    // } else {
    //   intake.intakeAngleMotor.set(0.0);
    // }

    // System.out.println(solenoid0.get());
    // System.out.println(solenoid1.get());
    // System.out.println("*******************************");

    // if(Constants.xbox.getRawButtonPressed(7)){
    //   solenoid1.set(true);
    //   // if(solenoid0.get()){
    //   //   solenoid0.set(false);
    //   //   solenoid1.set(true);
    //   // } else if(solenoid1.get()){
    //   //   solenoid1.set(false);
    //   //   solenoid0.set(true);
    //   // }
    //   //solenoid1.set(true);
    //   //doubleSolenoid.toggle();
    // }

    // if(Constants.xbox.getRawButtonPressed(8)){
    //   solenoid0.set(true);
    // }

    // if(Constants.xbox.getRawButtonPressed(2)){
    //   ph.fireOneShot(1);
    // }
    // ph.getCompressorConfigType()
    // ph.
    //System.out.println(ph.get);

    // if(autonomousTimer.get() > 10.0){
    //   doubleSolenoid.set(Value.kReverse);
    //   autonomousTimer.reset();
    //   autonomousTimer.start();
    // }

    //System.out.println("intake angle motor encoders: " + intake.intakeAngleMotor.getEncoder().getPosition());

    // still have the intake up and down stuff. Move at 0.5 speed until get within a certain number of encoder counts of the goal
    // then PID
    // have buttons that go up and down at 0.35 or something and if you press the buttons, you still switch between intake up and down
    // that finish the way

    // could double check distance calculation

    //////////////////////////////////////////////////////////////////////////
    //Color detectedColor = intake.indexerColorSensor.getColor();
    // System.out.println("blue: " + new Color(0.0, 0.290, 0.0).blue);
    // //double difference = Math.abs(detectedColor.blue - 0.2975); // or 0.295. < 0.015
    // double difference = Math.abs(detectedColor.red - 0.295); // 0.2932 // 0.2976
    // System.out.println("difference: " + difference);

    // if (difference < 0.025){
    //   intake.indexerMotor.set(0.0);
    // } else {
    //   intake.indexerMotor.set(-0.2);
    // }
    
    // ColorMatchResult match = intake.colorMatch.matchClosestColor(detectedColor);
    // System.out.println(match.color.red + " " + match.color.blue + " " + match.color.green);

    // if(Constants.stick.getRawButtonPressed(1)){
    //   shooter.shooting = !shooter.shooting;
    // }

    // if ((Math.abs(detectedColor.red - new Color(0.195, 0.374, 0.432).red) < 0.05) && detectingBall) {
    //   ballPresent = true;
    //   intakeTimer.reset();
    //   intakeTimer.start();
    // } else {
    //   if(!ballPresent){
    //     intake.indexerMotor.set(-0.2);
    //   }
    // }

    // if(ballPresent){
    //   detectingBall = false;
    //   intake.indexerMotor.set(-0.2);
    //   if(intakeTimer.get() > 0.6){
    //     intake.indexerMotor.set(0.0);
    //     if(shooter.shooting || shooter.dumpShot || (Constants.xbox.getPOV() == 180)){
    //       detectingBall = true;
    //       ballPresent = false;
    //     }
    //   }
    // }


    // // if(ballPresent){
      
    // // } else {

    // // }
    
    
    // // else {
    // //   ballPresent = false;
    // //   intake.indexerMotor.set(-0.2);
    // // }

    // if(ballPresent){

    // }
    /////////////////////////////////////////////////////////////////////

    //shooter.masterShooterMotor.set(ControlMode.PercentOutput, 0.87);
    // System.out.println("calculated " +
    // (shooter.shooterWheelLinearVelocityToMotorVelocity(shooter.calculatedVelocity)));
    // System.out.println("actual speed " +
    // (shooter.masterShooterMotor.getSelectedSensorVelocity()));
    // System.out.println("calculated angle: " + (90.0 - (shooter.calculatedAngle *
    // 180.0 / Math.PI)));
    // System.out.println("actual angle: " +
    // shooter.hoodMotor.getEncoder().getPosition());

    // if(Constants.stick.getRawButtonPressed(5)){
    // shooter.angleError -= 0.05;
    // } else if(Constants.stick.getRawButton(6)){
    // shooter.angleError += 0.05;
    // } else if(Constants.stick.getRawButton(3)){
    // shooter.angleError -= 0.01;
    // } else if(Constants.stick.getRawButtonPressed(4)){
    // shooter.angleError += 0.01;
    // }

    // System.out.println(shooter.getXDistanceFromCenterOfHub(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
    // System.out.println(shooter.angleError);

    // public static float getXDistanceFromCenterOfHub(double verticalAngle) { // x
    // // distance from front of robot to fender. better for testing
    // return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle +
    // cameraAngle - angleError))) - horizontalDistanceFromLimelightToFrontOfRobot -
    // distanceFromFenderToTape);
    // }

    // if(Constants.stick.getRawButtonPressed(7)){
    // shooterActivated = !shooterActivated;
    // }
    // if(Constants.stick.getRawButtonPressed(11)){ // adjust shooter speed
    // shooterActivated = true;
    // shooterSpeed -= 0.05;
    // }
    // if(Constants.stick.getRawButtonPressed(12)){
    // shooterActivated = true;
    // shooterSpeed += 0.05;
    // }
    // if(shooterActivated){
    // shooter.masterShooterMotor.set(ControlMode.PercentOutput, shooterSpeed);
    // } else {
    // shooter.masterShooterMotor.set(0.0);
    // }
  }

  //////////////////////////////////////////////////////////////
  // Disabled

  @Override
  public void disabledInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(1.0);
  }

  @Override
  public void disabledPeriodic() {
  }

}