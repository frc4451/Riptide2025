package frc.robot.subsystems.superstructures.corel.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerSubsystem;

public class CorelConveyor extends SingleRollerSubsystem {
  public CorelConveyor(SingleRollerIO io) {
    super("Corel/Conveyor", io);
  }

  // Probably incorrect voltage, but for sim, try this
  public Command scoreAtL1() {
    return this.run(() -> this.runRoller(12));
  }
}
