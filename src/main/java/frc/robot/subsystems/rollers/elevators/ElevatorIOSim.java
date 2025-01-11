package frc.robot.subsystems.rollers.elevators;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.setpoint.SetpointRollerIOSim;

public class ElevatorIOSim extends SetpointRollerIOSim implements ElevatorIO {
  public ElevatorIOSim(PIDConstants pidConstants, DCMotor gearbox, double reduction, double moi) {
    super(pidConstants, gearbox, reduction, moi);
  }

  public ElevatorIOSim() {
    this(
        ElevatorConstants.pidConstants,
        ElevatorConstants.gearbox,
        ElevatorConstants.reduction,
        ElevatorConstants.moi);
  }
}
