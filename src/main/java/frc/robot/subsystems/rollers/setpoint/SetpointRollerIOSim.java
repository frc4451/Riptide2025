package frc.robot.subsystems.rollers.setpoint;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;

public class SetpointRollerIOSim extends SingleRollerIOSim implements SetpointRollerIO {
  private final PIDController controller;

  public SetpointRollerIOSim(
      PIDConstants pidConstants, DCMotor gearbox, double reduction, double moi) {
    super(gearbox, reduction, moi);
    controller = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
  }

  @Override
  public void setPosition(double setpointInches) {
    // TODO
  }
}
