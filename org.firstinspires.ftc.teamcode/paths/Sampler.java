package org.firstinspires.ftc.teamcode;

import java.lang.Exception;

public class Sampler {
  private Component valueComponent;
  private Component timingComponent;
  private int dim;
  private int count;

  public Sampler(Component valueComponent, Component timingComponent) throws Vec.DimMismatchException, Sampler.CountMisMatchException {
    if (valueComponent.getCount() != timingComponent.getCount()) throw new Sampler.CountMisMatchException();
    if (timingComponent.getDim() != 1) throw new Vec.DimMismatchException("Timing component dimension mismatch");
    this.valueComponent = valueComponent;
    this.timingComponent = timingComponent;
    this.dim = valueComponent.getDim();
    this.count = valueComponent.getCount();
  }

  public static class CountMisMatchException extends Exception {
    public CountMisMatchException(String error) {
      super(error);
    }

    public CountMisMatchException() {
      super("Component count mismatch.");
    }
  }

  public int getDim() {
    return this.dim;
  }

  public int getCount() {
    return this.count;
  }

  public Vec getTime(double t) throws Vec.DimMismatchException {
    return this.timingComponent.getPosition(t).dims[0];
  }

  public Vec getValue(double t) throws Vec.DimMismatchException {
    return this.valueComponent.getPosition(this.getTime());
  }
}