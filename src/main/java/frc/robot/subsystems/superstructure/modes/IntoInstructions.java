package frc.robot.subsystems.superstructure.modes;

public enum IntoInstructions {
  NONE,
  /**
   * Move pivot into position before moving the elevator up. This is to prevent hitting the REEF.
   * For L4
   */
  PIVOTS_BEFORE_ELEVATOR,
}
