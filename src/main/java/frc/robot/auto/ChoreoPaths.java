package frc.robot.auto;

public enum ChoreoPaths {
  TWO_METERS("2 Meters"),
  THREE_METERS("3 Meters"),
  FIVE_METERS("5 Meters"),
  CURVY("Curvy"),
  START_MID_TO_G("StartMid--G"),
  G_TO_HPS_RIGHT("G-HPSRight"),
  HPS_RIGHT_TO_C("HPSRight-C");

  public final String name;

  private ChoreoPaths(String name) {
    this.name = name;
  }
}
