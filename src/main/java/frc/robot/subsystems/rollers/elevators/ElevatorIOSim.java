package frc.robot.subsystems.rollers.elevators;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;

public class ElevatorIOSim extends SingleRollerIOSim implements ElevatorIO {
  public ElevatorIOSim(DCMotor gearbox, double reduction, double moi) {
    super(gearbox, reduction, moi);
  }
}
