/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;



public class Robot extends TimedRobot {

  private double numBalls;

  private ColorSensorV3 color;

//All Spark Maxes
  
  //4 Drive Motors
  private CANSparkMax frontLeft;
  private CANSparkMax backLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backRight;

  //4 Inventory / arm motors
  private CANSparkMax botLeftArm;
  private CANSparkMax botRightArm;
  private CANSparkMax topLeftArm;
  private CANSparkMax topRightArm;

  //4 Climbimg motors
  private CANSparkMax climbLeft;
  private CANSparkMax climbRight;

  //New motors
  private CANSparkMax colorWheel;
  private CANSparkMax lockClimb;

  //Shooter Motors
  private CANSparkMax shooterLeft;
  private CANSparkMax shooterRight;

  //Intake Motor
  private CANSparkMax intakeMotor;

//All Pneumatic Solenoids

  //Inventory Lift /  Arm lift
  private DoubleSolenoid inventoryLift;
  //Intake Lift
  private DoubleSolenoid intake;
  //Climb Arm Solenoid
  private DoubleSolenoid climbArm;

//Driving

  //Joystick/Controller
  private Joystick stick;
  //Drive object (Differential Drive)
  private DifferentialDrive driveTrain;

//Sensors and PID Controllers

  //Ultrasonic
  private DigitalInput sensor;

  //PID Controllers
  private PIDController autoDriveController;
  private PIDController autoAngleController;
  private PIDController aimControllerX;
  private PIDController aimControllerY;
  private PIDController climbController;

  // Limelight
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;

  private double steeringAdjust;
  private double powerAdjust;
  private double shooterPower;

//Encoder Instantiation

  //Drive Train Encoders
  private CANEncoder frontLeftEncoder;
  private CANEncoder frontRightEncoder;
  private CANEncoder backLeftEncoder;
  private CANEncoder backRightEncoder;

  //Bottom Left Inventory / Arm Encoders
  private CANEncoder botLeftArmEncoder;
  private CANEncoder botRightArmEncoder;
  
  //Top Left Inventory / Arm Encoders
  private CANEncoder topLeftArmEncoder;
  private CANEncoder topRightArmEncoder;

  //Climb Motor Encoders;
  private CANEncoder leftClimbEncoder;
  private CANEncoder rightClimbEncoder;

  //Color wheel encoders
  private CANEncoder colorWheelEncoder;

//NAVX

  private AHRS ahrs;


  @Override
  public void robotInit() {

    numBalls = 0;
    steeringAdjust = 0;
    powerAdjust = 0;
    shooterPower = 0;

  //Spark Max Instantiation

    //Drive Train Motors
    frontLeft = new CANSparkMax(14,MotorType.kBrushless);
    backLeft = new CANSparkMax(13,MotorType.kBrushless);
    frontRight = new CANSparkMax(5,MotorType.kBrushless);
    backRight = new CANSparkMax(4,MotorType.kBrushless);

    //Inventory / Arm Motors
    botRightArm = new CANSparkMax(15, MotorType.kBrushless);
    botLeftArm = new CANSparkMax(10, MotorType.kBrushless);
    topRightArm = new CANSparkMax(1, MotorType.kBrushless);
    topLeftArm = new CANSparkMax(11, MotorType.kBrushless);
    
    //Intake Motor
    intakeMotor = new CANSparkMax(7, MotorType.kBrushless);

    //Shooter Controller
    shooterLeft = new CANSparkMax(6, MotorType.kBrushless);
    shooterRight = new CANSparkMax(17, MotorType.kBrushless);

    //Climb Motors
    climbRight = new CANSparkMax(3, MotorType.kBrushless);
    climbLeft = new CANSparkMax(2, MotorType.kBrushless);
    
    //Climb Adjustment Motors
    lockClimb = new CANSparkMax(18, MotorType.kBrushless);
    colorWheel = new CANSparkMax(16, MotorType.kBrushless);

//Driving

    //Differential Drive
    driveTrain = new DifferentialDrive(frontLeft, frontRight);
    //Joystick
    stick = new Joystick(0);

//Pneumatic Solenoids

    //Intake
    intake = new DoubleSolenoid(0,1);
    //Arm/Inventory Lift
    inventoryLift = new DoubleSolenoid(2, 3);
    //Climbing Arm Solenoids
    climbArm = new DoubleSolenoid(4,5);


//Sensor Instantiation

  //Encoder Instantiation

    //Drive Train Encoders
    frontLeftEncoder = new CANEncoder(frontLeft);
    frontRightEncoder = new CANEncoder(frontRight);
    backLeftEncoder = new CANEncoder(backLeft);
    backRightEncoder = new CANEncoder(backRight);

    //Bottom Arm / Inventory Encoders
    botLeftArmEncoder = new CANEncoder(botLeftArm);
    botRightArmEncoder = new CANEncoder(botRightArm);

    //Top Arm / Inventory Encoders  
    topLeftArmEncoder = new CANEncoder(topLeftArm);
    topRightArmEncoder = new CANEncoder(topRightArm);

    //Climb Encoders
    leftClimbEncoder = new CANEncoder(climbLeft);
    rightClimbEncoder = new CANEncoder(climbRight);
    
    //Color Wheel
    colorWheelEncoder = new CANEncoder(colorWheel);

  //Other Sensors

    //Ultrasonic sensor
    sensor = new DigitalInput(0);
    color = new ColorSensorV3(I2C.Port.kOnboard);

    //Limelight
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    //PID Controllers
    autoDriveController = new PIDController(0.5, 0.05, 0.05);
    autoAngleController = new PIDController(0.5, 0.05, 0.05);
    aimControllerX = new PIDController(0.3,0,0);
    aimControllerY = new PIDController(0.3,0,0);
    climbController = new PIDController(0.8,0.08,0.08);

    //NAVx
    try{ 
     ahrs = new AHRS(SerialPort.Port.kMXP);
    }catch (RuntimeException ex ) {
     DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }


  }

  @Override
  public void autonomousInit() {

    //Set motors to coast mode
    frontRight.setIdleMode(IdleMode.kCoast);
    backRight.setIdleMode(IdleMode.kCoast);
    backLeft.setIdleMode(IdleMode.kCoast);
    frontLeft.setIdleMode(IdleMode.kCoast);
    topLeftArm.setIdleMode(IdleMode.kCoast);
    topRightArm.setIdleMode(IdleMode.kCoast);
    botLeftArm.setIdleMode(IdleMode.kCoast);
    botRightArm.setIdleMode(IdleMode.kCoast);
    shooterLeft.setIdleMode(IdleMode.kCoast);
    shooterRight.setIdleMode(IdleMode.kCoast);
    intakeMotor.setIdleMode(IdleMode.kCoast);

    //Set Motors to brake mode
    climbLeft.setIdleMode(IdleMode.kBrake);
    climbRight.setIdleMode(IdleMode.kBrake);
    colorWheel.setIdleMode(IdleMode.kBrake);
    lockClimb.setIdleMode(IdleMode.kBrake);

    //Set motors to follow leaders
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    //Reset Navx
    ahrs.reset();

    //Reset all Encoders
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    backLeftEncoder.setPosition(0);
    backRightEncoder.setPosition(0);
    botLeftArmEncoder.setPosition(0);
    botRightArmEncoder.setPosition(0);
    topLeftArmEncoder.setPosition(0);
    topRightArmEncoder.setPosition(0);
    leftClimbEncoder.setPosition(0);
    rightClimbEncoder.setPosition(0);
    
  }

  @Override
  public void autonomousPeriodic() {


  }

  @Override
  public void teleopInit() {

    //Set motors to coast mode
    frontRight.setIdleMode(IdleMode.kCoast);
    backRight.setIdleMode(IdleMode.kCoast);
    backLeft.setIdleMode(IdleMode.kCoast);
    frontLeft.setIdleMode(IdleMode.kCoast);
    topLeftArm.setIdleMode(IdleMode.kCoast);
    topRightArm.setIdleMode(IdleMode.kCoast);
    botLeftArm.setIdleMode(IdleMode.kCoast);
    botRightArm.setIdleMode(IdleMode.kCoast);
    shooterLeft.setIdleMode(IdleMode.kCoast);
    shooterRight.setIdleMode(IdleMode.kCoast);
    intakeMotor.setIdleMode(IdleMode.kCoast);

    //Set Motors to brake mode
    climbLeft.setIdleMode(IdleMode.kBrake);
    climbRight.setIdleMode(IdleMode.kBrake);
    colorWheel.setIdleMode(IdleMode.kBrake);
    lockClimb.setIdleMode(IdleMode.kBrake);

    //Set motors to follow leaders
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    //Reset Navx
    ahrs.reset();

    //Reset all Encoders
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    backLeftEncoder.setPosition(0);
    backRightEncoder.setPosition(0);
    botLeftArmEncoder.setPosition(0);
    botRightArmEncoder.setPosition(0);
    topLeftArmEncoder.setPosition(0);
    topRightArmEncoder.setPosition(0);
    leftClimbEncoder.setPosition(0);
    rightClimbEncoder.setPosition(0);

    //CameraServer.getInstance().startAutomaticCapture();
   // CameraServer.getInstance().startAutomaticCapture();
    table.getEntry("pipeline").setDouble(1);

  }

  @Override
  public void teleopPeriodic() {

    //Display Drivetrain motor outputs to smart dashboard
    SmartDashboard.putNumber("Shooter", shooterPower);

    //Display navx gyro
    SmartDashboard.putData(ahrs);
    SmartDashboard.putBoolean("sensor", sensor.get());
    SmartDashboard.putNumber("Distance", calculateDistance());
 
    

    
  if(stick.getRawButton(7)){
      climbLeft.set(0.3);
      climbRight.set(-0.3);
  }else if(stick.getRawButton(8)){
    climbLeft.set(-1);
    climbRight.set(1);
    lockClimb.set(-0.6);
  }else{
    climbLeft.set(0);
    climbRight.set(0);
    lockClimb.set(0);
  }

    


//Robot Control Commands

    //Lift Inventory Arms UP AND DOWN .getPOV(0);
    liftArm(stick.getPOV(0), stick.getPOV(0));
    //Lift Intake D PAD, LEFT AND RIGHT
    liftIntake(stick.getPOV(0), stick.getPOV(0));

    // //Normal Shoot
    // if(stick.getRawAxis(3) > 0.5){
    //   shoot(1);
    // }else{
    //   shoot(0);
    // }

    

    // //Aim and shoot
    if(stick.getRawAxis(3) > 0.5){
      limelight();
      //driveTrain.arcadeDrive(powerAdjust, steeringAdjust);
      shoot(shooterPower);
      if(shooterPower == 0){
        shoot(1.0);
      }else{
        shoot(shooterPower);
      }
     }else{
       table.getEntry("pipeline").setDouble(1);
       shoot(0);
     }

  

  
    //Drive with left and right Joystick
    driveTrain.arcadeDrive(-stick.getRawAxis(1) * 0.8, stick.getRawAxis(4) * 0.8);
  //TODO: insert correct numbers for encoders and for ultarsonic
    ballOrganize(0.275, 0.2, 0.2, stick.getRawAxis(2) > 0.25, 3.5, stick.getRawButton(6), stick.getRawButton(5));


    //Deploy climb Solenoids
    if(stick.getRawButton(2) == true){
      climbArm.set(Value.kForward);
    }else if(stick.getRawButton(1) == true){
      climbArm.set(Value.kReverse);
    }else{
      climbArm.set(Value.kOff);
    }


  

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

//Written Methods

  //Read and use Limelight data
  public void limelight(){

    //read values periodically
    double x = tx.getDouble(0.0); //The horizontal distance between the target and the crosshair
    double y = ty.getDouble(0.0); //The vertical distance between the target and the crosshair
    double v = tv.getDouble(0.0); //Checks if there is a target on screen
    double ky = 0.3;
    double kx = 0.3;
    table.getEntry("pipeline").setDouble(0);

    //Check if the limelight has a target
    if(v == 0){
      //No Target

      steeringAdjust = 0;
      powerAdjust = 0;
      shooterPower = 0;

    }
    else{
      //Target found
      steeringAdjust = -x * kx;
      shooterPower = calculateShooterSpeed();

      }
    }

  

//TODO: add the proper measurements. [Code based on Limelight 1.0 Docs]
  public double calculateDistance(){

    double targetHeight = 85; //from floor.
    double mountedHeight = 25.5; //Limelight mount height
    double mountedAngle = Math.toRadians(22.2);
    double cameraAngleFromTarget = Math.toRadians(ty.getDouble(0.0)); //take from limelight.
                //target height
    double distance = (targetHeight - mountedHeight) / (Math.tan(mountedAngle + cameraAngleFromTarget));

    return distance ;

  }

  //TODO: insert measurments for shooter
  public double calculateShooterSpeed(){

    double heightOfGoal = 98; // = y
    double distanceFromGoal = calculateDistance(); // = x
    double shooterMechHeight = 40; // = Yo
    double shooterAngle = Math.toRadians(50); // = Theta 32 and 40
    double radius = 2;
    double height = heightOfGoal - shooterMechHeight;

    double neoRPM = 5676;

    //double part1 = -192.9 * Math.pow(distanceFromGoal, 2);
   // double part2 = ((heightOfGoal - shooterMechHeight) - (distanceFromGoal * Math.tan(shooterAngle))) * (Math.pow(Math.cos(shooterAngle), 2));

     double part1 = -385.8;
    // //                2 * cos theta                           y  cos theta           -    x sin theta
     double part2 = (2 * Math.cos(shooterAngle)) * (height * Math.cos(shooterAngle) - distanceFromGoal* Math.sin(shooterAngle));

    double speed =  distanceFromGoal * Math.sqrt(part1 / part2);



    //Convert inch/s to ft/s to RPM || turn ft/s to ft/m then divide by circumference to get RPM|| Convert to percentage to get motor speed value by dividing by max RPMS
    double motorSpeed = (((speed * 60) / (radius * Math.PI )) / neoRPM);

    if(distanceFromGoal > 320){
      return 1;
    }

    return motorSpeed * 0.80;
  }



//Robot Control Methods

//TODO: Intake Control
  public void ballOrganize(double intakePower, double botArmPower, double topArmPower, boolean on,  double encoderConstant, boolean push, boolean pull){

//IF Doesnt work add numballs

    if(on){
      //Spin Intake and bottom arms
      organize(on, false, botArmPower, intakePower);
      //Check if a ball has been intaked
      if(sensor.get()){
        //Reset Encoder
        
        organize(on, false, botArmPower, intakePower);
        inventory(true, false, topArmPower);
       // Spins top of arms until ball is in position
        // if(topLeftArmEncoder.getPosition() < encoderConstant + topLeftArmEncoder.getPosition() && topRightArmEncoder.getPosition() > -encoderConstant + topRightArmEncoder.getPosition()){
        //   inventory(true, false, topArmPower);
        // }
      }else{

        inventory(false, false, topArmPower);
      }
    }else if(push){
      topLeftArm.set(0.2);
      topRightArm.set(-0.2);
      botLeftArm.set(0.2);
      botRightArm.set(-0.2);
    }else if(pull){
      topLeftArm.set(-0.2);
      topRightArm.set(0.2);

    }
    else{
      organize(false, false, 0, 0);
      inventory(false, false, 0);
    }
  }
 
//Lift Inventory arms
  public void liftArm(int up, int down){
    if(up == 0){
     
      inventoryLift.set(DoubleSolenoid.Value.kReverse);
    }
    else if(down == 180){
      inventoryLift.set(DoubleSolenoid.Value.kForward);
    }
    else{
      inventoryLift.set(DoubleSolenoid.Value.kOff);
    }
  }


//Lift Intake
  public void liftIntake(int up, int down){
    if(up == 270){
      intake.set(DoubleSolenoid.Value.kForward);
    }
    else if(down == 90){
      intake.set(DoubleSolenoid.Value.kReverse);
    }
    else{
      intake.set(DoubleSolenoid.Value.kOff);
      
    }
  }


 //inventory power control
 public void inventory(boolean intake, boolean output, double power){
   if(intake){
     topLeftArm.set(power);
     topRightArm.set(-power);
   }else if(output){
     topLeftArm.set(-power);
     topRightArm.set(power);
   }else{
     topLeftArm.set(0);
     topRightArm.set(0);
   }
 }
//Control intake and motor at same time
 public void organize(boolean intake, boolean output, double armPower, double intakePower){

   if(intake){
       botLeftArm.set(armPower);
       botRightArm.set(-armPower);
       intakeMotor.set(-intakePower);
   }else if(output){
       botLeftArm.set(-armPower);
       botRightArm.set(armPower);
   }else{
       botLeftArm.set(0);
       botRightArm.set(0);
       intakeMotor.set(0);
   }

 }
//Spin just intake
 public void spinIntake(boolean on, boolean reverse, double power){
   if(on){
     intakeMotor.set(power);
   }
   else if (reverse){
     intakeMotor.set(-power);
   }
   else{
     intakeMotor.set(0);
   }
 }

//Spin Shooter motors
  public void shoot( double power){
      shooterLeft.set(power);
      shooterRight.set(-power);
  }
//Force all balls through shooter
  public void pushBalls(double power, boolean on){
    
    if(on){
      topLeftArm.set(power);
      topRightArm.set(-power);
      botLeftArm.set(power);
      botRightArm.set(-power);
    }else{
      topLeftArm.set(0);
      topRightArm.set(0);
      botLeftArm.set(0);
      botRightArm.set(0);
    }
  }

 //Converts encoder ticks to feet
  public double tickToFeet(double numTicks, double gearRatio, double conversion, double diameter){
      //ticks x 1 rotation / 4096    x  1/1 gear ratio    x 6pi inches /   1 rotation x  1 feet/12 inch = ?feet
    double feet = numTicks * ((1 / 4096.0) * (gearRatio) * ((diameter * Math.PI)) * (1/12.0));
    //return ticks converted to feet
    return feet;

  }


  public void climb(boolean lift, boolean drop, boolean pullUp, double downSpeed, double encoderValue){
//Lift arms to exact height
    if(lift){

        climbLeft.set(0.3);
        climbRight.set(-0.3);

    }else if(drop){
      climbLeft.set(-downSpeed);
      climbRight.set(downSpeed);
    }else{
      climbLeft.set(0);
      climbRight.set(0);
    }

  }



     //TODO: test PID values
     public void driveTo(double distance){
      frontLeftEncoder.setPosition(0);
      double currentDistance = tickToFeet(frontLeftEncoder.getPosition(), 10.71, frontLeftEncoder.getCountsPerRevolution(), 6);
  
      if(currentDistance <= distance){
        driveTrain.arcadeDrive(autoDriveController.calculate(currentDistance, distance), 0);
      }else{
        driveTrain.arcadeDrive(0, 0);
      }
  
  }
  
  //TODO: test PID values
    public void rotateToAngle(double awngle){
  
      ahrs.reset();
      double currentAngle = ahrs.getAngle();
  
      //CHeck if you are closer to turn left or right
      if(currentAngle<=awngle){
        driveTrain.tankDrive(autoAngleController.calculate(currentAngle, awngle), -autoAngleController.calculate(currentAngle, awngle));
      }else{
        driveTrain.tankDrive(0, 0);
      }
  
  }




}