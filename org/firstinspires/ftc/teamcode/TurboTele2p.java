package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp
public class TurboTele2p extends LinearOpMode{
  // Control Hub Motors
  DcMotor frontLeft;
  DcMotor backLeft;
  
  DcMotor Turret;
  //DcMotor testMotor; // For testing only!
  
  // Control Hub Servos
  Servo clawServoRight;
  Servo clawServoLeft;
  Servo leftArm;
  Servo deposit;
  Servo frontArm;
  
  // Expansion Hub Motors
  DcMotor frontRight;
  DcMotor backRight;
  DcMotor linSlide;
  
  // Expansion Hub Servos
  Servo linearServo;
  Servo rightArm;

  // Odometry
  private OdometryWrapper odometryWrapper;
  private double inchesToTicks = 3000.0;
  private double trackWidth = 11.25;
  private double forwardOffset = 5.35;
  
  //Sensors
  TouchSensor leftTouch;
  TouchSensor rightTouch;
  private void initControlHub(){
    telemetry.addData("Status", "Initializing Control Hub");
    telemetry.update();
    
    // Get motors
    frontLeft = hardwareMap.dcMotor.get("FrontLeft");
    backLeft  = hardwareMap.dcMotor.get("BackLeft");
    //testMotor = hardwareMap.dcMotor.get("TestMotor");
    
    // Get servos
    clawServoRight = hardwareMap.servo.get("ClawServo1");
    clawServoLeft  = hardwareMap.servo.get("ClawServo2");
    leftArm        = hardwareMap.servo.get("LeftArm");
    deposit        = hardwareMap.servo.get("Deposit");
    
    frontLeft.setDirection(DcMotor.Direction.REVERSE);
    backLeft.setDirection(DcMotor.Direction.REVERSE);
    
    telemetry.addData("Status", "Initialized Control Hub");
    telemetry.update();
  }
  
  public void initExpansionHub() {
    telemetry.addData("Status", "Initializing Expansion Hub");
    telemetry.update();
    
    // Get motors
    frontRight    = hardwareMap.dcMotor.get("FrontRight");
    backRight     = hardwareMap.dcMotor.get("BackRight");
    linSlide      = hardwareMap.dcMotor.get("LinSlide");
    
    
    Turret = hardwareMap.dcMotor.get("Turret");
    
    //Get sensor
    leftTouch = hardwareMap.get(TouchSensor.class, "LeftTouchSensor");
    rightTouch = hardwareMap.get(TouchSensor.class, "RightTouchSensor");
    
    
    // Get servos
    linearServo = hardwareMap.servo.get("LinearServo");
    rightArm    = hardwareMap.servo.get("RightArm");
    frontArm    = hardwareMap.servo.get("FrontArm");
    
    telemetry.addData("Status", "Initialized Expansion Hub");
    telemetry.update();
  }

  private void initOdometry() {
    // this.odometryWrapper = new OdometryWrapper()
    //   .setLeftEncoder(hardwareMap.dcMotor.get("BackLeft"))
    //   .setRightEncoder(hardwareMap.dcMotor.get("FrontRight"))
    //   .setFrontEncoder(hardwareMap.dcMotor.get("BackRight"))
    //   .setTrackWidth(this.trackWidth * inchesToTicks)
    //   .setForwardOffset(this.forwardOffset * inchesToTicks);
  }
  
  public void initAll() {
    initControlHub();
    initOdometry();
    initExpansionHub();
    Retract();
    
    
    telemetry.addData("Status", "Initialized all");
    telemetry.update();
    
    waitForStart();
  }

  // todo: write your code here
  public void runOpMode() {
    
    initAll();
    //data
    int linPosition=0;
    int targeted_position = 0;
    
    int leftBound = 999;
    int rightBound = -999;
    linSlide.setTargetPosition(0);

    while (opModeIsActive()) {
  //   linearServo.setPosition(0.2);
      String active;
      
      double y = -0.7 * gamepad1.left_stick_y; // Remember, this is reversed!
      double x = 0.7 * gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
      double rx = 0.5 * gamepad1.right_stick_x;

      // Denominator is the largest motor power (absolute value) or 1
      // This ensures all the powers maintain the same ratio, but only when
      // at least one is out of the range [-1, 1]
      double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
      double frontLeftPower = (y + x + rx) / denominator;
      double backLeftPower = (y - x + rx) / denominator;
      double frontRightPower = (y - x - rx) / denominator;
      double backRightPower = (y + x - rx) / denominator;

      frontLeft.setPower(frontLeftPower);
      //motorFrontLeft.setPower(gamepad2.right_trigger);
      backLeft.setPower(backLeftPower);
      frontRight.setPower(frontRightPower);
      backRight.setPower(backRightPower);

      if(gamepad1.dpad_up){
        targeted_position = 1200;
        if(linPosition <= targeted_position){
          linPosition = targeted_position;
          linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);                linSlide.setTargetPosition(-linPosition);
          linSlide.setPower(1);
        }else{
          linPosition = targeted_position + 1;
          linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          linSlide.setTargetPosition(-linPosition);
          linSlide.setPower(0.2);
        }
      }
      if(gamepad1.dpad_left){
        targeted_position = 800;
        if(linPosition <= targeted_position){
          linPosition = targeted_position;
          linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);                linSlide.setTargetPosition(-linPosition);
          linSlide.setPower(1);
        }else{
          linPosition = targeted_position + 1;
          linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          linSlide.setTargetPosition(-linPosition);
          linSlide.setPower(0.2);
        }
      }
      if(gamepad1.dpad_down){
        targeted_position = 0;
        if(linPosition <= targeted_position){
          linPosition = targeted_position;
          linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);                linSlide.setTargetPosition(-linPosition);
          linSlide.setPower(1);
        }else{
          linPosition = targeted_position + 1;
          linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          linSlide.setTargetPosition(-linPosition);
          linSlide.setPower(0.4);
        }
      }
      if(linSlide.getCurrentPosition() == targeted_position){
        linPosition--;
      }
      
      if(leftTouch.isPressed()){
        leftBound = Turret.getCurrentPosition();
      }
      if(rightTouch.isPressed()){
        rightBound = Turret.getCurrentPosition();
      }
      
      if(gamepad1.a){
        Extend(1);
      }
      if(gamepad1.b){
        Retract();
      }
      if(gamepad1.x){
        linearServo.setPosition(0.1);
      }
      if(gamepad1.y){
        linearServo.setPosition(0.9);
      }
      
      Turret.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
      
      telemetry.addData("right turret bound", rightBound);
      telemetry.addData("left turret bound", leftBound);
      // this.odometryWrapper.update();
      // telemetry.addData("X", this.odometryWrapper.getX());
      // telemetry.addData("Y", this.odometryWrapper.getY());
      // telemetry.addData("R", this.odometryWrapper.getR());
      telemetry.update();
    }
  }
  public void Retract(){
    rightArm.setPosition(0.5);
    leftArm.setPosition(0.5);
  }

  public void Extend(double amount){
    rightArm.setPosition(0.5 + 0.35*amount);
    leftArm.setPosition(0.5 - 0.35*amount);
  }
  
}