package frc.robot.subsystems.rollers.feedforward_controller;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

// Weird Java stuff so we can have this be simulated
public sealed interface FeedforwardController
    permits ArmFeedforwardController, ElevatorFeedforwardController, EmptyFeedforwardController {
  public static double calculate(
      FeedforwardController controller, TrapezoidProfile.State setpoint) {
    if (controller instanceof ArmFeedforwardController) {
      ArmFeedforwardController ff = (ArmFeedforwardController) controller;
      return ff.calculate(setpoint.position, setpoint.velocity);
    } else if (controller instanceof ElevatorFeedforwardController) {
      ElevatorFeedforwardController ff = (ElevatorFeedforwardController) controller;
      return ff.calculate(setpoint.velocity);
    } else if (controller instanceof EmptyFeedforwardController) {
      return 0;
    } else {
      return 0;
    }
  }
}
