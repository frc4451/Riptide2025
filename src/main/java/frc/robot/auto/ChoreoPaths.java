package frc.robot.auto;

import java.util.Arrays;
import java.util.Iterator;

public enum ChoreoPaths {
  TWO_METERS("2 Meters"),
  THREE_METERS("3 Meters"),
  FIVE_METERS("5 Meters"),
  CURVY("Curvy"),
  
  // Start to ___
  START_MID_TO_G("StartMid-G"),

  // ___ to HPS
  A_TO_HPS_RIGHT("A-HPSRight"),
  B_TO_HPS_RIGHT("B-HPSRight"),
  C_TO_HPS_RIGHT("C-HPSRight"),
  D_TO_HPS_RIGHT("D-HPSRight"),
  E_TO_HPS_RIGHT("E-HPSRight"),
  F_TO_HPS_RIGHT("F-HPSRight"),
  G_TO_HPS_RIGHT("G-HPSRight"),
  H_TO_HPS_RIGHT("H-HPSRight"),
  I_TO_HPS_RIGHT("I-HPSRight"),
  J_TO_HPS_RIGHT("J-HPSRight"),
  K_TO_HPS_RIGHT("K-HPSRight"),
  L_TO_HPS_RIGHT("L-HPSRight"),

  // HPS to ___
  HPS_RIGHT_TO_A("HPSRight-A"),
  HPS_RIGHT_TO_B("HPSRight-B"),
  HPS_RIGHT_TO_C("HPSRight-C"),
  HPS_RIGHT_TO_D("HPSRight-D"),
  HPS_RIGHT_TO_E("HPSRight-E"),
  HPS_RIGHT_TO_F("HPSRight-F"),
  HPS_RIGHT_TO_G("HPSRight-G"),
  HPS_RIGHT_TO_H("HPSRight-H"),
  HPS_RIGHT_TO_I("HPSRight-I"),
  HPS_RIGHT_TO_J("HPSRight-J"),
  HPS_RIGHT_TO_K("HPSRight-K"),
  HPS_RIGHT_TO_L("HPSRight-L");

  public final String name;

  private ChoreoPaths(String name) {
    this.name = name;
  }

  public static Iterator<String> pathSequence(ChoreoPaths... paths) {
    return Arrays.stream(paths)
      .map(path -> path.name)
      .iterator();
  }
}
