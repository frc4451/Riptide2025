package frc.robot.util;

import java.util.function.Supplier;

public class TimeSinceConditionTracker {
  private double onTrueTimestamp = 0.0;
  private double deltaTime = 0.0;

  private final Supplier<Boolean> conditionSupplier;
  private final double timeThreshold;

  public TimeSinceConditionTracker(Supplier<Boolean> conditionSupplier, double timeThreshold) {
    this.conditionSupplier = conditionSupplier;
    this.timeThreshold = timeThreshold;
  }

  public void update(double timestamp) {
    boolean condition = this.conditionSupplier.get();

    if (!condition) {
      onTrueTimestamp = timestamp;
      deltaTime = 0.0;
    } else {
      deltaTime = timestamp - onTrueTimestamp;
    }
  }

  public double get() {
    return deltaTime;
  }

  public boolean hasExceededThreshold() {
    return get() >= timeThreshold;
  }
}
