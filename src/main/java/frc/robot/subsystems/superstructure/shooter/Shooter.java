package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.follow.FollowRollersIO;
import frc.robot.subsystems.rollers.follow.FollowRollersSubsystem;

public class Shooter extends FollowRollersSubsystem {
  public Shooter(FollowRollersIO io) {
    super("Shooter", io);
  }

  public void runVelocity(double velocity) {
    io.runVelocity(velocity);
  }

  public Command runVelocityCommand(double velocity) {
    return run(() -> runVelocity(velocity));
  }
}
