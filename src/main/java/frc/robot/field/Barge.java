package frc.robot.field;

public class Barge {
  public static final Cages blue = new Cages(FieldConstants.blueBarge);
  public static final Cages red = new Cages(FieldConstants.redBarge);

  public static Cages get() {
    return FieldUtils.isBlueAlliance() ? blue : red;
  }
}
