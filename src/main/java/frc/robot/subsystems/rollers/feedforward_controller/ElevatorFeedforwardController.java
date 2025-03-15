package frc.robot.subsystems.rollers.feedforward_controller;

import edu.wpi.first.math.controller.ElevatorFeedforward;

public final class ElevatorFeedforwardController implements FeedforwardController {
  private final ElevatorFeedforward ff;

  public ElevatorFeedforwardController(ElevatorFeedforward ff) {
    this.ff = ff;
  }

  public double calculate(double nextVelocity) {
    return ff.calculate(nextVelocity, 0);
  }
}
