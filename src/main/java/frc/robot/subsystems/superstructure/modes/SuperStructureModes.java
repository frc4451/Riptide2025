package frc.robot.subsystems.superstructure.modes;

import edu.wpi.first.math.geometry.Rotation2d;

public enum SuperStructureModes {
  TUCKED(0, Rotation2d.kZero),
  INTAKE(1.0, Rotation2d.kPi),
  L1Coral(17.0, Rotation2d.kPi),
  L2Coral(17.0, Rotation2d.kPi),
  L3Coral(
      33.0,
      Rotation2d.kPi,
      IntoInstructions.PIVOTS_BEFORE_ELEVATOR,
      ExitInstructions.ELEVATOR_BEFORE_PIVOTS),
  L4Coral(
      47.0,
      Rotation2d.fromDegrees(165.0),
      IntoInstructions.PIVOTS_BEFORE_ELEVATOR,
      ExitInstructions.ELEVATOR_BEFORE_PIVOTS),
  L2Algae(20, Rotation2d.fromDegrees(60.0)),
  L3Algae(
      31,
      Rotation2d.fromDegrees(60.0),
      IntoInstructions.PIVOTS_BEFORE_ELEVATOR,
      ExitInstructions.ELEVATOR_BEFORE_PIVOTS),
  Barge(
      49,
      Rotation2d.fromDegrees(180),
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
