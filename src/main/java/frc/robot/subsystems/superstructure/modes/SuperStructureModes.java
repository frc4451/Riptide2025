package frc.robot.subsystems.superstructure.modes;

import edu.wpi.first.math.geometry.Rotation2d;

public enum SuperStructureModes {
  TUCKED(0, Rotation2d.kZero, Rotation2d.kZero),
  INTAKE(10, Rotation2d.kPi, Rotation2d.kPi),
  L1(1.0, Rotation2d.kPi, Rotation2d.kPi),
  L2(2.0, Rotation2d.kPi, Rotation2d.kPi),
  L3(3.0, Rotation2d.kPi, Rotation2d.kPi),
  L4(4.0, Rotation2d.kPi, Rotation2d.kPi);

  public final double elevatorHeightInches;
  public final Rotation2d coralPos;
  public final Rotation2d algaePos;

  private SuperStructureModes(double elevatorHeightIn, Rotation2d coralPos, Rotation2d algaePos) {
    this.elevatorHeightInches = elevatorHeightIn;
    this.coralPos = coralPos;
    this.algaePos = algaePos;
  }
}
