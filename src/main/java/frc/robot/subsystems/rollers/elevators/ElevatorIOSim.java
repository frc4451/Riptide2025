package frc.robot.subsystems.rollers.elevators;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;

public class ElevatorIOSim extends SingleRollerIOSim implements ElevatorIO {
  public ElevatorIOSim(
      String name, DCMotor gearbox, double reduction, double moi, PIDConstants pidConstants) {
    super(gearbox, reduction, moi, pidConstants);
  }

  public ElevatorIOSim() {
    this(
        "Elevator",
        ElevatorConstants.gearbox,
        ElevatorConstants.reduction,
        ElevatorConstants.moi,
        ElevatorConstants.pidConstants);
  }
}
