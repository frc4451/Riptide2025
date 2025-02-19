package frc.robot.field;

public enum ReefFaces {
  AB(FieldConstants.blueReefAB, FieldConstants.redReefAB),
  CD(FieldConstants.blueReefCD, FieldConstants.redReefCD),
  EF(FieldConstants.blueReefEF, FieldConstants.redReefEF),
  GH(FieldConstants.blueReefGH, FieldConstants.redReefGH),
  IJ(FieldConstants.blueReefIJ, FieldConstants.redReefIJ),
  KL(FieldConstants.blueReefKL, FieldConstants.redReefKL);

  public final ReefFace blue;
  public final ReefFace red;

  private ReefFaces(ReefFace blue, ReefFace red) {
    this.blue = blue;
    this.red = red;
  }

  public ReefFace get() {
    return FieldUtils.isBlueAlliance() ? blue : red;
  }
}
