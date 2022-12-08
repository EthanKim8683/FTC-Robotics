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
  private double _paraOffset;
  private double _perpOffset;
  private int _paraPosition;
  private int _perpPosition;
  private double _imuPosition;
  private int _paraDelta;
  private int _perpDelta;
  private double _imuDelta;
  private double _x;
  private double _y;
  private double _r;
  private ArrayList<PoseEventHandler> poseEventHandlers;
  
  public TwoWheelOdometryWrapper() {
    this._x = 0.0;
    this._y = 0.0;
    this._r = 0.0;
    this.poseEventHandlers = new ArrayList<PoseEventHandler>();
  }

  public static interface PoseEventHandler {
    public boolean execute(double x, double y, double r);
  }
  
  public TwoWheelOdometryWrapper setParaEncoder(DcMotor encoder) {
    this._paraEncoder = encoder;
    encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    return this;
  }
  
  public TwoWheelOdometryWrapper setPerpEncoder(DcMotor encoder) {
    this._perpEncoder = encoder;
    encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    return this;
  }
  
  public TwoWheelOdometryWrapper setImu(BNO055IMU imu) {
    this._imu = imu;
    return this;
  }
  
  public TwoWheelOdometry setParaOffset(double offset) {
    this._paraOffset = offset * TwoWheelOdometryWrapper.INCHES_TO_TICKS;
    return this;
  }
  
  public TwoWheelOdometryWrapper setPerpOffset(double offset) {
    this._perpOffset = offset * TwoWheelOdometryWrapper.INCHES_TO_TICKS;
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
  
  public int getParaOffset() {
    return this_paraOffset;
  }
  
  public int getPerpOffset() {
    return this_perpOffset;
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
    this._imuDelta = nowImuPosition - this._imuPosition;
    this._imuPosition = nowImuPosition;
  }
  
  private void updatePose() {
    double paraDelta = this._paraDelta - this._paraOffset * this._imuDelta;
    double perpDelta = this._perpDelta + this._perpOffset * this._imuDelta;
    this._x += (paraDelta * Math.cos(this._r) - perpDelta * Math.sin(this._r)) / TwoWheelOdometryWrapper.INCHES_TO_TICKS;
    this._y += (paraDelta * Math.sin(this._r) + perpDelta * Math.cos(this._r)) / TwoWheelOdometryWrapper.INCHES_TO_TICKS;
    this._r = this._imuPosition;
    for (int i = poseEventHandlers.size() - 1; i >= 0; i--) {
      PoseEventHandler poseEventHandler = this.poseEventHandlers.get(i);
      if (poseEventHandler.execute(this._x, this._y, this._r)) this.poseEventHandlers.remove(i);
    }
  }
  
  public void update() {
    this.updateParaEncoder();
    this.updatePerpEncoder();
    this.updateImu();
    this.updatePose();
  }
}
