package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoWrapper {
  private Servo _servo;
  private double _position = 0.0;
  private double _targetPosition = 0.0;
  private double _lowerBound = 0.0;
  private double _upperBound = 1.0;
  private boolean _isBusy = false;
  private boolean _isAsync = false;
  private EventHandlerManager becomeBusyEventHandlerManager;
  private EventHandlerManager becomeIdleEventHandlerManager;
  
  public ServoWrapper() {
    this.becomeBusyEventHandlerManager = new EventHandlerManager();
    this.becomeIdleEventHandlerManager = new EventHandlerManager();
  }
  
  public ServoWrapper setServo(Servo servo) {
    this._servo = servo;
    return this;
  }
  
  public ServoWrapper setLowerBound(double lowerBound) {
    this._lowerBound = lowerBound;
    return this;
  }
  
  public ServoWrapper setUpperBound(double upperBound) {
    this._upperBound = upperBound;
    return this;
  }

  public ServoWrapper setPosition(double weight) {
    double targetPosition = Helper.applyWeight(this._lowerBound, this._upperBound, weight);
    this._targetPosition = targetPosition;
    this._servo.setPosition(targetPosition);
    return this;
  }

  public ServoWrapper subscribeBecomeBusyEvent(EventHandler eventHandler) {
    this.becomeBusyEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public ServoWrapper subscribeBecomeIdleEvent(EventHandler eventHandler) {
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

  public Servo getServo() {
    return this._servo;
  }

  public double getLowerBound() {
    return this._lowerBound;
  }

  public double getUpperBound() {
    return this._upperBound;
  }

  public boolean isBusy() {
    return this._isBusy;
  }

  public boolean isAsync() {
    return this._isAsync;
  }

  public double getPosition() {
    return this._position;
  }

  private void updatePosition() {
    this._position = this._servo.getPosition();
  }

  private void updateIsBusy() {
    boolean nowBusy = Math.abs(this._position - this._targetPosition) > Helper.EPSILON;
    if (!this._isBusy && nowBusy) this.becomeBusyEventHandlerManager.execute();
    if (this._isBusy && !nowBusy) this.becomeIdleEventHandlerManager.execute();
    this._isBusy = nowBusy;
  }
  
  public void update() {
    this.updatePosition();
    this.updateIsBusy();
  }
}