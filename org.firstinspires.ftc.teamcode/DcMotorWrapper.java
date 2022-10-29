package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DcMotorWrapper {
  private DcMotor _dcMotor;
  private int _position = 0;
  private int _targetPosition = 0;
  private int _lowerBound = 0;
  private int _upperBound = 1;
  private double _forwardPower = 0.5;
  private double _reversePower = 0.5;
  private boolean _isBusy = false;
  private boolean _isAsync = false;
  private EventHandlerManager becomeBusyEventHandlerManager;
  private EventHandlerManager becomeIdleEventHandlerManager;
  
  public DcMotorWrapper() {
    this.becomeBusyEventHandlerManager = new EventHandlerManager();
    this.becomeIdleEventHandlerManager = new EventHandlerManager();
  }
  
  public DcMotorWrapper setDcMotor(DcMotor dcMotor) {
    this._dcMotor = dcMotor;
    dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    return this;
  }
  
  public DcMotorWrapper setLowerBound(int lowerBound) {
    this._lowerBound = lowerBound;
    return this;
  }
  
  public DcMotorWrapper setUpperBound(int upperBound) {
    this._upperBound = upperBound;
    return this;
  }

  public DcMotorWrapper setForwardPower(double forwardPower) {
    this._forwardPower = forwardPower;
    return this;
  }

  public DcMotorWrapper setReversePower(double reversePower) {
    this._reversePower = reversePower;
    return this;
  }

  public DcMotorWrapper setPower(double power) {
    this._forwardPower = power;
    this._reversePower = power;
    return this;
  }

  public DcMotorWrapper setPosition(double weight) {
    int targetPosition = Helper.applyWeight(this._lowerBound, this._upperBound, weight);
    double power = 0.0;
    if (targetPosition > this._position) power = this._forwardPower;
    if (targetPosition < this._position) power = this._reversePower;
    this._targetPosition = targetPosition;
    this._dcMotor.setTargetPosition(targetPosition);
    this._dcMotor.setPower(power);
    this._dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    return this;
  }

  public DcMotorWrapper setPosition(int position) {
    this._dcMotor.setTargetPosition(position);
    return this;
  }

  public DcMotorWrapper subscribeBecomeBusyEvent(EventHandler eventHandler) {
    this.becomeBusyEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public DcMotorWrapper subscribeBecomeIdleEvent(EventHandler eventHandler) {
    this.becomeIdleEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public AsyncBody gotoPosition(double weight) {
    return next -> {
      this.setPosition(weight);
      this._isAsync = true;
      EventHandler becomeBusy = () -> {
        EventHandler becomeIdle = () -> {
          this._isAsync = false;
          next.execute();
          return true;
        };
        this.subscribeBecomeIdleEvent(becomeIdle);
        return true;
      };
      this.subscribeBecomeBusyEvent(becomeBusy);
    };
  }

  public DcMotor getDcMotor() {
    return this._dcMotor;
  }

  public int getLowerBound() {
    return this._lowerBound;
  }

  public int getUpperBound() {
    return this._upperBound;
  }

  public double getForwardPower() {
    return this._forwardPower;
  }

  public double getReversePower() {
    return this._reversePower;
  }

  public boolean isBusy() {
    return this._isBusy;
  }

  public boolean isAsync() {
    return this._isAsync;
  }

  public int getPosition() {
    return this._position;
  }

  public int getTargetPosition() {
    return this._targetPosition;
  }

  private void updatePosition() {
    this._position = this._dcMotor.getCurrentPosition();
  }

  private void updateIsBusy() {
    boolean nowBusy = this._dcMotor.isBusy();
    if (!this._isBusy && nowBusy) this.becomeBusyEventHandlerManager.execute();
    if (this._isBusy && !nowBusy) this.becomeIdleEventHandlerManager.execute();
    this._isBusy = nowBusy;
  }
  
  public void update() {
    this.updatePosition();
    this.updateIsBusy();
  }
}