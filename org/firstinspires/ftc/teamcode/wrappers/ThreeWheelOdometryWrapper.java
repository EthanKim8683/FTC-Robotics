package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ThreeWheelOdometryWrapper {
  private static double INCHES_TO_TICKS = 1901.86;

  private DcMotor _leftEncoder;
  private DcMotor _rightEncoder;
  private DcMotor _frontEncoder;
  private int _leftEncoderPosition;
  private int _rightEncoderPosition;
  private int _frontEncoderPosition;
  private int _leftEncoderDelta;
  private int _rightEncoderDelta;
  private int _frontEncoderDelta;
  private double _trackWidth;
  private double _forwardOffset;
  private double _x;
  private double _y;
  private double _r;
  private ArrayList<PoseEventHandler> poseEventHandlers;
  
  public ThreeWheelOdometryWrapper() {
    this._x = 0.0;
    this._y = 0.0;
    this._r = 0.0;
    this.poseEventHandlers = new ArrayList<PoseEventHandler>();
  }

  public static interface PoseEventHandler {
    public boolean execute(double x, double y, double r);
  }
  
  public ThreeWheelOdometryWrapper setLeftEncoder(DcMotor encoder) {
    this._leftEncoder = encoder;
    encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    return this;
  }
  
  public ThreeWheelOdometryWrapper setRightEncoder(DcMotor encoder) {
    this._rightEncoder = encoder;
    encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    return this;
  }
  
  public ThreeWheelOdometryWrapper setFrontEncoder(DcMotor encoder) {
    this._frontEncoder = encoder;
    encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    return this;
  }
  
  public ThreeWheelOdometryWrapper setTrackWidth(double _trackWidth) {
    this._trackWidth = _trackWidth * ThreeWheelOdometryWrapper.INCHES_TO_TICKS;
    return this;
  }
  
  public ThreeWheelOdometryWrapper setForwardOffset(double _forwardOffset) {
    this._forwardOffset = _forwardOffset * ThreeWheelOdometryWrapper.INCHES_TO_TICKS;
    return this;
  }

  public ThreeWheelOdometryWrapper subscribePoseEvent(PoseEventHandler eventHandler) {
    this.poseEventHandlers.add(eventHandler);
    return this;
  }
  
  public double getX() {
    return this._x;
  }
  
  public double getY() {
    return this._y;
  }
  
  public double getR() {
    return this._r;
  }

  public DcMotor getLeftEncoder() {
    return this._leftEncoder;
  }

  public DcMotor getRightEncoder() {
    return this._rightEncoder;
  }

  public DcMotor getFrontEncoder() {
    return this._frontEncoder;
  }

  public int getLeftEncoderPosition() {
    return this._leftEncoderPosition;
  }

  public int getRightEncoderPosition() {
    return this._rightEncoderPosition;
  }

  public int getFrontEncoderPosition() {
    return this._frontEncoderPosition;
  }

  public int getLeftEncoderDelta() {
    return this._leftEncoderDelta;
  }

  public int getRightEncoderDelta() {
    return this._rightEncoderDelta;
  }

  public int getFrontEncoderDelta() {
    return this._rightEncoderDelta;
  }

  public double getTrackWidth() {
    return this._trackWidth;
  }

  public double getForwardOffset() {
    return this._forwardOffset;
  }
  
  private void updateLeftEncoder() {
    int nowLeftEncoderPosition = this._leftEncoder.getCurrentPosition();
    this._leftEncoderDelta = nowLeftEncoderPosition - this._leftEncoderPosition;
    this._leftEncoderPosition = nowLeftEncoderPosition;
  }
  
  private void updateRightEncoder() {
    int nowRightEncoderPosition = -this._rightEncoder.getCurrentPosition();
    this._rightEncoderDelta = nowRightEncoderPosition - this._rightEncoderPosition;
    this._rightEncoderPosition = nowRightEncoderPosition;
  }
  
  private void updateFrontEncoder() {
    int nowFrontEncoderPosition = this._frontEncoder.getCurrentPosition();
    this._frontEncoderDelta = nowFrontEncoderPosition - this._frontEncoderPosition;
    this._frontEncoderPosition = nowFrontEncoderPosition;
  }
  
  private void updatePose() {
    double phi = (double) (this._leftEncoderDelta - this._rightEncoderDelta) / this._trackWidth;
    double paraDelta = (double) (this._leftEncoderDelta + this._rightEncoderDelta) / 2;
    double perpDelta = this._frontEncoderDelta - this._forwardOffset * phi;
    this._x -= (paraDelta * Math.sin(this._r) + perpDelta * Math.cos(this._r)) / ThreeWheelOdometryWrapper.INCHES_TO_TICKS;
    this._y += (paraDelta * Math.cos(this._r) - perpDelta * Math.sin(this._r)) / ThreeWheelOdometryWrapper.INCHES_TO_TICKS;
    this._r += phi;
    for (int i = poseEventHandlers.size() - 1; i >= 0; i--) {
      PoseEventHandler poseEventHandler = this.poseEventHandlers.get(i);
      if (poseEventHandler.execute(this._x, this._y, this._r)) this.poseEventHandlers.remove(i);
    }
  }
  
  public void update() {
    this.updateLeftEncoder();
    this.updateRightEncoder();
    this.updateFrontEncoder();
    this.updatePose();
  }
}
