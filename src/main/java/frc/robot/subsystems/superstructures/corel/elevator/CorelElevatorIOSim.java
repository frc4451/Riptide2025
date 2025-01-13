package frc.robot.subsystems.superstructures.corel.elevator;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.elevators.ElevatorIOSim;

public class CorelElevatorIOSim extends ElevatorIOSim {
  public CorelElevatorIOSim(
      String name, DCMotor gearbox, double reduction, double moi, PIDConstants pidConstants) {
    super(name, gearbox, reduction, moi, pidConstants);
  }

  public CorelElevatorIOSim() {
    this(
        "CorelElevator",
        CorelElevatorConstants.gearbox,
        CorelElevatorConstants.reduction,
        CorelElevatorConstants.moi,
        CorelElevatorConstants.pidConstants);
  }
}
