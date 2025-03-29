package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ClimberModes {
  TUCKED(Rotation2d.kZero),
  EXTENDED(Rotation2d.kZero),
  GRABBED(Rotation2d.kZero);

  public Rotation2d rotation;

  private ClimberModes(Rotation2d rotation) {
    this.rotation = rotation;
  }
}
