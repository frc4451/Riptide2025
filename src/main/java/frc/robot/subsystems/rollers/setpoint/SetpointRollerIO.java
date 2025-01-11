package frc.robot.subsystems.rollers.setpoint;

import frc.robot.subsystems.rollers.single.SingleRollerIO;

public interface SetpointRollerIO extends SingleRollerIO {
  public default void setPosition(double setpoint) {}
}
