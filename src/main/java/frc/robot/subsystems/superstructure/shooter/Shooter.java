package frc.robot.subsystems.superstructure.shooter;

import frc.robot.subsystems.rollers.follow.FollowRollersIO;
import frc.robot.subsystems.rollers.follow.FollowRollersSubsystem;

public class Shooter extends FollowRollersSubsystem {
  public Shooter(FollowRollersIO io) {
    super("Shooter", io);
  }
}
