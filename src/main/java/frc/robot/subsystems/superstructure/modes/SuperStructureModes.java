package frc.robot.subsystems.superstructure.modes;

import edu.wpi.first.math.geometry.Rotation2d;

public enum SuperStructureModes {
  TUCKED(0, Rotation2d.kZero),
  INTAKE(40.0, Rotation2d.kPi),
  L1(1.0, Rotation2d.kPi),
  L2(2.0, Rotation2d.kPi),
  L3(3.0, Rotation2d.kPi),
  L4(
      40.0,
      Rotation2d.kPi,
      IntoInstructions.PIVOTS_BEFORE_ELEVATOR,
      ExitInstructions.ELEVATOR_BEFORE_PIVOTS);

  public final double elevatorHeightInches;
  public final Rotation2d coralPos;
  public final IntoInstructions intoInstructions;
  public final ExitInstructions exitInstructions;

  private SuperStructureModes(double elevatorHeightIn, Rotation2d coralPos) {
    this(elevatorHeightIn, coralPos, IntoInstructions.NONE, ExitInstructions.NONE);
  }

  private SuperStructureModes(
      double elevatorHeightIn,
      Rotation2d coralPos,
      IntoInstructions intoInstructions,
      ExitInstructions exitInstructions) {
    this.elevatorHeightInches = elevatorHeightIn;
    this.coralPos = coralPos;
    this.intoInstructions = intoInstructions;
    this.exitInstructions = exitInstructions;
  }
}
