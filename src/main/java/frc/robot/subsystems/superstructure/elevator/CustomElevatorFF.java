package frc.robot.subsystems.superstructure.elevator;

/**
 * @see edu.wpi.first.math.controller.ElevatorFeedForward
 */
public record CustomElevatorFF(
    double kG, double kSUp, double kSDown, double kVUp, double kVDown, double kAUp, double kADown) {

  /**
   * @see edu.wpi.first.math.controller.ElevatorFeedForward#calculate(double, double)
   */
  public double calculate(double velocity, double acceleration) {
    double direction = Math.signum(velocity);
    return kG
        + (direction == 1 ? kSUp : kSDown) * direction
        + (direction == 1 ? kVUp : kVDown) * velocity
        + (direction == 1 ? kAUp : kADown) * acceleration;
  }
}
