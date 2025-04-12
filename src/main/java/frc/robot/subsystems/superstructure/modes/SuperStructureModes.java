package frc.robot.subsystems.superstructure.modes;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.superstructure.constants.PivotConstants;

public enum SuperStructureModes {
  TUCKED(0.9 / 2.0, PivotConstants.intialPosition),
  TUCKED_L4(SuperStructureModes.TUCKED.elevatorHeightInches, Rotation2d.fromDegrees(92)),
  L1Coral(9.5 / 2.0, Rotation2d.fromDegrees(-60)),
  L2Coral(19.0 / 2.0, SuperStructureModes.TUCKED.coralPos),
  L3Coral(36.0 / 2.0, SuperStructureModes.TUCKED.coralPos),
  L4Coral(50.0 / 2.0, SuperStructureModes.TUCKED_L4.coralPos),
  FLOOR_ALGAE(1.0 / 2.0, Rotation2d.fromDegrees(-40.0)),
  L2Algae(23.0 / 2.0, Rotation2d.fromDegrees(-10.0)),
  L3Algae(40.0 / 2.0, Rotation2d.fromDegrees(-10.0)),
  Barge(50.0 / 2.0, Rotation2d.fromDegrees(65));

  public final double elevatorHeightInches;
  public final Rotation2d coralPos;

  private SuperStructureModes(double elevatorHeightIn, Rotation2d coralPos) {
    this.elevatorHeightInches = elevatorHeightIn;
    this.coralPos = coralPos;
  }
}
