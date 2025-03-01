package frc.robot.subsystems.superstructure.modes;

import edu.wpi.first.math.geometry.Rotation2d;

public enum SuperStructureModes {
  TUCKED(0, Rotation2d.kZero),
  INTAKE(1.0 / 2.0, Rotation2d.kPi),
  L1Coral(17.0 / 2.0, Rotation2d.kPi),
  L2Coral(17.0 / 2.0, Rotation2d.kPi),
  L3Coral(33.0 / 2.0, Rotation2d.kPi),
  L4Coral(47.0 / 2.0, Rotation2d.fromDegrees(165.0)),
  L2Algae(20 / 2.0, Rotation2d.fromDegrees(60.0)),
  L3Algae(31 / 2.0, Rotation2d.fromDegrees(60.0)),
  Barge(49 / 2.0, Rotation2d.fromDegrees(180));

  public final double elevatorHeightInches;
  public final Rotation2d coralPos;

  private SuperStructureModes(double elevatorHeightIn, Rotation2d coralPos) {
    this.elevatorHeightInches = elevatorHeightIn;
    this.coralPos = coralPos;
  }
}
