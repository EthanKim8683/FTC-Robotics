package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TimeManager {
  private ArrayList<Double> times;
  private ArrayList<TimeHandler> timeHandlers;
  private double time;
  private OpMode opMode;

  public TimeManager() {
    this.times = new ArrayList<Double>();
    this.timeHandlers = new ArrayList<TimeHandler>();
  }

  public interface TimeHandler {
    public void execute();
  }

  public void subscribeTimeEvent(double time, TimeHandler timeHandler) {
    this.times.add(this.time + time);
    this.timeHandlers.add(timeHandler);
  }

  public void update() {
    double nowTime = opMode.time;
    for (int i = timeHandlers.size() - 1; i >= 0; i--) {
      TimeHandler timeHandler = timeHandlers.get(i);
      timeHandler.execute();
      if (times.get(i) < nowTime){
        this.times.remove(i);
        this.timeHandlers.remove(i);
      }
    }
  }
}