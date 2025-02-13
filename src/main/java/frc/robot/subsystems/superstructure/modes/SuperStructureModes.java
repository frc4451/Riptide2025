package frc.robot.subsystems.superstructure.modes;

import edu.wpi.first.math.geometry.Rotation2d;

public enum SuperStructureModes {
  TUCKED(0, Rotation2d.kZero, Rotation2d.kZero),
  INTAKE(40.0, Rotation2d.kPi, Rotation2d.kCCW_Pi_2),
  L1(1.0, Rotation2d.kPi, Rotation2d.kPi),
  L2(2.0, Rotation2d.kPi, Rotation2d.kPi),
  L3(3.0, Rotation2d.kPi, Rotation2d.kPi),
  L4(40.0, Rotation2d.kPi, Rotation2d.kPi, true, true);

  public final double elevatorHeightInches;
  public final Rotation2d coralPos;
  public final Rotation2d algaePos;
  public final boolean rotateBeforeElevator;
  public final boolean elevatorBeforeRotate;

  private SuperStructureModes(double elevatorHeightIn, Rotation2d coralPos, Rotation2d algaePos) {
    this(elevatorHeightIn, coralPos, algaePos, false, false);
  }

  private SuperStructureModes(
      double elevatorHeightIn,
      Rotation2d coralPos,
      Rotation2d algaePos,
      boolean rotateBeforeElevator,
      boolean elevatorBeforeRotate) {
    this.elevatorHeightInches = elevatorHeightIn;
    this.coralPos = coralPos;
    this.algaePos = algaePos;
    this.rotateBeforeElevator = rotateBeforeElevator;
    this.elevatorBeforeRotate = elevatorBeforeRotate;
  }
}
