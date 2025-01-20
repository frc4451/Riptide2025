package frc.robot.subsystems.superstructure.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerSubsystem;

public class Chute extends SingleRollerSubsystem {
  public Chute(SingleRollerIO io) {
    super("Chute", io);
  }

  public void runVelocity(double velocity) {
    io.runVelocity(velocity);
  }

  public Command runVelocityCommand(double velocity) {
    return run(() -> runVelocity(velocity));
  }
}
