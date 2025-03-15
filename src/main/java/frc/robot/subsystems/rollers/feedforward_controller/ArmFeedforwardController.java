package frc.robot.subsystems.rollers.feedforward_controller;

import edu.wpi.first.math.controller.ArmFeedforward;

public final class ArmFeedforwardController implements FeedforwardController {
  private final ArmFeedforward ff;

  public ArmFeedforwardController(ArmFeedforward ff) {
    this.ff = ff;
  }

  public double calculate(double nextPosition, double nextVelocity) {
    return ff.calculate(nextPosition, nextVelocity);
  }
}
