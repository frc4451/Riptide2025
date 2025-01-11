package frc.robot.subsystems.rollers.elevators;

import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class ElevatorSubsystem extends SingleRollerSubsystem {
  private final double inchesPerRad;

  public ElevatorSubsystem(String name, SingleRollerIO io, double radiusInches) {
    super(name, io);
    this.inchesPerRad = radiusInches;
  }

  @AutoLogOutput
  private double getHeight() {
    return inputs.positionRad * inchesPerRad;
  }
}
