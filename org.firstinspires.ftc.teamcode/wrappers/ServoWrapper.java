package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoWrapper {
  private Servo _servo;
  private double _position = 0.0;
  private double _lowerBound = 0.0;
  private double _upperBound = 1.0;
  private boolean _isAsync = false;
  
  public ServoWrapper() {}
  
  public ServoWrapper setServo(Servo servo) {
    this._servo = servo;
    return this;
  }
  
  public ServoWrapper setLowerBound(double lowerBound) {
    this._lowerBound = lowerBound;
    this._servo.scaleRange(this._lowerBound, this._upperBound);
    return this;
  }
  
  public ServoWrapper setUpperBound(double upperBound) {
    this._upperBound = upperBound;
    this._servo.scaleRange(this._lowerBound, this._upperBound);
    return this;
  }

  public ServoWrapper setRange(double lowerBound, double upperBound) {
    this._lowerBound = lowerBound;
    this._upperBound = upperBound;
    this._servo.scaleRange(this._lowerBound, this._upperBound);
    return this;
  }

  public ServoWrapper setPosition(double weight) {
    this._servo.setPosition(weight);
    return this;
  }

  public Async.AsyncBody gotoPosition(double weight) {
    return async -> {
      this.setPosition(weight);
      this._isAsync = true;
      // Servo wait code here!
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

  public boolean isAsync() {
    return this._isAsync;
  }

  public double getPosition() {
    return this._position;
  }
}