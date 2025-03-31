package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ClimberModes {
  TUCK(Rotation2d.fromDegrees(0)),
  EXTEND(Rotation2d.fromDegrees(200.0)),
  GRAB(Rotation2d.fromDegrees(70.0));
  // 90
  //

  public final Rotation2d rotation;

  private ClimberModes(Rotation2d rotation) {
    this.rotation = rotation;
  }
}
