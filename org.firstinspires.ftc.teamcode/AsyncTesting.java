package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class AsyncTesting extends LinearOpMode {
  // Control Hub Motors
  DcMotorWrapper frontLeft;
  DcMotorWrapper backLeft;
  DcMotorWrapper frontArm;
  
  // Control Hub Servos
  ServoWrapper clawServoRight;
  ServoWrapper clawServoLeft;
  ServoWrapper leftArm;
  
  // Expansion Hub Motors
  DcMotorWrapper frontRight;
  DcMotorWrapper backRight;
  DcMotorWrapper linSlideUpper;
  DcMotorWrapper linSlideLower;
  
  // Expansion Hub Servos
  ServoWrapper linearServo;
  ServoWrapper rightArm;

  GamepadWrapper gamepad1;
  
  // Claw control limits
  private double clawLowerBound = 0.1;
  private double clawUpperBound = 0.5;
  
  // Arm control limits
  private double armLowerBound = 0.15;
  private double armUpperBound = 0.5;
  
  // Front Arm control limits
  private int frontArmLowerBound = 0;
  private int frontArmUpperBound = 850;
  private double frontArmPower = 0.5;
  
  // Linear Slide control limits
  private int linSlideLowerBound = 0;
  private int linSlideUpperBound = 930;
  private double linSlideExtendingPower = 0.9;
  private double linSlideReturningPower = 0.3;
  
  // Linear Servo control limits
  private double linearServoLowerBound = 0.1;
  private double linearServoUpperBound = 0.9;
  
  private void initControlHub() {
    telemetry.addData("Status", "Initializing Control Hub");
    telemetry.update();
    
    // Get motors
    this.frontLeft = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("FrontLeft"));
    this.backLeft  = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("BackLeft"));
    this.frontArm  = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("FrontArm"))
      .setLowerBound(frontArmLowerBound)
      .setUpperBound(frontArmUpperBound)
      .setPower(frontArmPower);
    
    // Get servos
    this.clawServoRight = new ServoWrapper()
      .setServo(hardwareMap.servo.get("ClawServoRight"))
      .setLowerBound(clawLowerBound)
      .setUpperBound(clawUpperBound);
    this.clawServoLeft  = new ServoWrapper()
      .setServo(hardwareMap.servo.get("ClawServoLeft"))
      .setLowerBound(clawLowerBound)
      .setUpperBound(clawUpperBound);
    this.leftArm        = new ServoWrapper()
      .setServo(hardwareMap.servo.get("LeftArm"))
      .setLowerBound(armLowerBound)
      .setUpperBound(armUpperBound);
    
    telemetry.addData("Status", "Initialized Control Hub");
    telemetry.update();
  }
  
  public void initExpansionHub() {
    telemetry.addData("Status", "Initializing Expansion Hub");
    telemetry.update();
    
    // Get motors
    this.frontRight    = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("FrontRight"));
    this.backRight     = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("BackRight"));
    this.linSlideLower = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("LinSlideLower"))
      .setLowerBound(linSlideLowerBound)
      .setUpperBound(linSlideUpperBound)
      .setForwardPower(linSlideExtendingPower)
      .setReversePower(linSlideReturningPower);
    this.linSlideUpper = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("LinSlideUpper"))
      .setLowerBound(-linSlideLowerBound)
      .setUpperBound(-linSlideUpperBound)
      .setForwardPower(linSlideExtendingPower)
      .setReversePower(linSlideReturningPower);
    
    // Get servos
    this.linearServo = new ServoWrapper()
      .setServo(hardwareMap.servo.get("LinearServo"))
      .setLowerBound(linearServoLowerBound)
      .setUpperBound(linearServoUpperBound);
    this.rightArm    = new ServoWrapper()
      .setServo(hardwareMap.servo.get("RightArm"))
      .setLowerBound(armUpperBound)
      .setUpperBound(armLowerBound);
    
    telemetry.addData("Status", "Initialized Expansion Hub");
    telemetry.update();
  }

  public void initGamepads() {
    telemetry.addData("Status", "Initializing Gamepads");
    telemetry.update();
    
    // Get gamepad1
    this.gamepad1 = new GamepadWrapper()
      .setGamepad(gamepad1);

    telemetry.addData("Status", "Initialized Gamepads");
    telemetry.update();
  }
  
  public void initAll() {
    this.initControlHub();
    this.initExpansionHub();
    this.initGamepads();
    
    telemetry.addData("Status", "Initialized all");
    telemetry.update();
    
    waitForStart();
  }
  
  private void updateAll() {
    // Control Hub Motors
    this.frontLeft.update();
    this.backLeft.update();
    this.frontArm.update();
    
    // Control Hub Servos
    this.clawServoRight.update();
    this.clawServoLeft.update();
    this.leftArm.update();
    
    // Expansion Hub Motors
    this.frontRight.update();
    this.backRight.update();
    this.linSlideUpper.update();
    this.linSlideLower.update();
    
    // Expansion Hub Servos
    this.linearServo.update();
    this.rightArm.update();

    // Gamepads update
    this.gamepad1.update();
  }
  
  private void displayStats() {
  }
  
  @Override
  public void runOpMode() throws InterruptedException {
    this.initAll();

    if (isStopRequested()) return;
    
    while (opModeIsActive()) {
      this.updateAll();
      this.displayStats();
      telemetry.update();
    }
  }
}