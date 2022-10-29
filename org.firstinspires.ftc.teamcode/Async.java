package org.firstinspires.ftc.teamcode;

import java.util.Queue;
import java.util.LinkedList;

public class Async {
  private Queue<AsyncBody> chain;

  public Async(AsyncBody body) {
    this.chain = new LinkedList<AsyncBody>();
    body.execute(() -> step());
  }

  private void step() {
    AsyncBody body = this.chain.poll();
    if (body == null) return;
    body.execute(() -> step());
  }

  public Async then(AsyncBody body) {
    this.chain.add(body);
    return this;
  }

  public void cancel() {
    this.chain.clear();
  }
}