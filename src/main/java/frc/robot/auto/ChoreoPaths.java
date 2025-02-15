package frc.robot.auto;

public enum ChoreoPaths {
  TWO_METERS("2 Meters"),
  THREE_METERS("3 Meters"),
  FIVE_METERS("5 Meters"),
  CURVY("Curvy");

  public final String name;

  private ChoreoPaths(String name) {
    this.name = name;
  }
}
