package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * @see edu.wpi.first.math.controller.ElevatorFeedForward
 */
public class CustomElevatorController {
  private final double kG;
  private final double kSUp;
  private final double kSDown;
  private final double kVUp;
  private final double kVDown;
  private final double kAccel;
  private final double kDecel;

  private final PIDController feedbackController;

  public CustomElevatorController(
      double tolerance,
      double kP,
      double kI,
      double kD,
      double kG,
      double kSUp,
      double kSDown,
      double kVUp,
      double kVDown,
      double kAccel,
      double kDecel) {
    this.kG = kG;
    this.kSUp = kSUp;
    this.kSDown = kSDown;
    this.kVUp = kVUp;
    this.kVDown = kVDown;
    this.kAccel = kAccel;
    this.kDecel = kDecel;

    this.feedbackController = new PIDController(kP, kI, kD);
    feedbackController.setTolerance(tolerance);
  }

  /**
   * @param nextVelocity The next velocity setpoint.
   * @param accleration The current acceleration of the profile.
   * @return The computed feedforward in volts.
   */
  public double calculate(double positionError, double velocity, double acceleration) {
    double direction = Math.signum(velocity);
    double feedforward =
        kG
            + (direction == 1 ? kSUp : kSDown) * direction
            + (direction == 1 ? kVUp : kVDown) * velocity
            + (Math.signum(acceleration) == 1 ? kAccel : kDecel) * acceleration;
    double feedback = feedbackController.calculate(0, positionError);

    // Add kS to feedback when feedforward is done
    double output = feedforward + (feedbackController.atSetpoint() ? 0 : feedback);
    // Comment this while tuning only feedforward as you don't have P
    boolean feedforwardDone = velocity == 0 && acceleration == 0;
    if (feedforwardDone && !feedbackController.atSetpoint()) {
      output += (Math.signum(positionError) == 1 ? kSUp : -kSDown);
    }

    return output;
  }

  /**
   * Calculates the feedforward from the gains and setpoints assuming discrete control.
   *
   * @param currentVelocity The current velocity setpoint.
   * @param nextVelocity The next velocity setpoint.
   * @return The computed feedforward in volts.
   */
  public double calculateWithVelocities(
      double currentPosition, double nextPosition, double currentVelocity, double nextVelocity) {
    double positionError = nextPosition - currentPosition;
    double acceleration = (nextVelocity - currentVelocity) / Constants.loopPeriodSecs;
    Logger.recordOutput("Superstructure/Elevator/AccelerationInchesPerSecPerSec", acceleration);
    return calculate(positionError, nextVelocity, acceleration);
  }
}
