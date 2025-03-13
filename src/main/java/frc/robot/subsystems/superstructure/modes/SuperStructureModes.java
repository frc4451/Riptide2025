package frc.robot.subsystems.superstructure.modes;

import edu.wpi.first.math.geometry.Rotation2d;

public enum SuperStructureModes {
  TUCKED(1.1 / 2.0, Rotation2d.kZero),
  TUCKED_L4(1.1 / 2.0, Rotation2d.fromDegrees(180.0)),
  TEST_45(1.0 / 2.0, Rotation2d.fromDegrees(45)),
  TEST_90(1.0 / 2.0, Rotation2d.fromDegrees(90)),
  TEST_180(1.0 / 2.0, Rotation2d.fromDegrees(182)),
  L2Coral(19.0 / 2.0, Rotation2d.kZero),
  L3Coral(36.0 / 2.0, Rotation2d.kZero),
  L4Coral(50.0 / 2.0, Rotation2d.fromDegrees(180.0)),
  FLOOR_ALGAE(1.0 / 2.0, Rotation2d.fromDegrees(60)),
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
