package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class TwoWheelOdometryWrapper {
  private static double INCHES_TO_TICKS = 1901.86;

  private DcMotor _paraEncoder;
  private DcMotor _perpEncoder;
  private BNO055IMU _imu;
  private double _paraOffsetX;
  private double _paraOffsetY;
  private double _perpOffsetX;
  private double _perpOffsetY;
  private int _paraPosition;
  private int _perpPosition;
  private double _imuPosition;
  private int _paraDelta;
  private int _perpDelta;
  private double _imuDelta;
  private double _x;
  private double _y;
  private double _r;
  
  public TwoWheelOdometryWrapper() {
    this._x = 0.0;
    this._y = 0.0;
    this._r = 0.0;
  }
  
  public TwoWheelOdometry setParaEncoder(DcMotor encoder) {
    this._paraEncoder = encoder;
    return this;
  }
  
  public TwoWheelOdometry setPerpEncoder(DcMotor encoder) {
    this._perpEncoder = encoder;
    return this;
  }
  
  public TwoWheelOdometry setImu(BNO055IMU imu) {
    this._imu = imu;
    return this;
  }
  
  public TwoWheelOdometry setParaOffsetX(double x) {
    this._paraOffsetX = x;
    return this;
  }
  
  public TwoWheelOdometry setParaOffsetY(double y) {
    this._paraOffsetY = y;
    return this;
  }
  
  public TwoWheelOdometry setPerpOffsetX(double x) {
    this._perpOffsetX = x;
    return this;
  }
  
  public TwoWheelOdometry setPerpOffsetY(double y) {
    this._perpOffsetY = y;
    return this;
  }
  
  public DcMotor getParaEncoder() {
    return this._paraEncoder;
  }
  
  public DcMotor getPerpEncoder() {
    return this._perpEncoder;
  }
  
  public DcMotor getImu() {
    return this._imu;
  }
  
  public int getParaOffsetX() {
    return this_paraOffsetX;
  }
  
  public int getParaOffsetY() {
    return this_paraOffsetY;
  }
  
  public int getPerpOffsetX() {
    return this_perpOffsetX;
  }
  
  public int getPerpOffsetY() {
    return this_perpOffsetY;
  }
  
  private void updateParaEncoder() {
    int nowParaEncoderPosition = this._paraEncoder.getCurrentPosition();
    this._paraEncoderDelta = nowParaEncoderPosition - this._paraEncoderPosition;
    this._paraEncoderPosition = nowParaEncoderPosition;
  }
  
  private void updatePerpEncoder() {
    int nowPerpEncoderPosition = this._perpEncoder.getCurrentPosition();
    this._perpEncoderDelta = nowPerpEncoderPosition - this._perpEncoderPosition;
    this._perpEncoderPosition = nowPerpEncoderPosition;
  }
  
  private void updateImu() {
    AxesReference axesReference = AxesReference.EXTRINSIC;
    AxesOrder axesOrder = AxesOrder.ZXY
    AngleUnit angleUnit = AngleUnit.RADIANS;
    Orientation orientation = this.imu.getAngularOrientation(axesReference, axesOrder, angleUnit);
    double nowImuPosition = orientation.firstAngle;
    this._imuDelta = nowImuPositionn - this._imuPosition;
    this._imuPosition = nowImuPosition;
  }
  
  private void updatePose() {
    double paraDelta = this._paraEncoderDelta - this.paraOffsetX * this._imuDelta;
    double perpDelta = this._perpEncoderDelta + this.perpOffsetY * this._imuDelta;
    this._x += (paraDelta * Math.cos(this._r) - perpDelta * Math.sin(this._r)) / TwoWheelOdometryWrapper.INCHES_TO_TICKS;
    this._y += (paraDelta * Math.sin(this._r) + perpDelta * Math.cos(this._r)) / TwoWheelOdometryWrapper.INCHES_TO_TICKS;
    this._r = this._imuPosition;
  }
  
  public void update() {
    this.updateParaEncoder();
    this.updatePerpEncoder();
    this.updateImu();
    this.updatePose();
  }
}
