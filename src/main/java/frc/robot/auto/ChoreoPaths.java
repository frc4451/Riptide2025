package frc.robot.auto;

import java.util.Arrays;
import java.util.Iterator;

public enum ChoreoPaths {
  TWO_METERS("2 Meters"),
  THREE_METERS("3 Meters"),
  FIVE_METERS("5 Meters"),
  CURVY("Curvy"),

  // Start Mid to ___
  START_MID_TO_A("StartMid-A"),
  START_MID_TO_B("StartMid-B"),
  START_MID_TO_C("StartMid-C"),
  START_MID_TO_D("StartMid-D"),
  START_MID_TO_E("StartMid-E"),
  START_MID_TO_F("StartMid-F"),
  START_MID_TO_G("StartMid-G"),
  START_MID_TO_H("StartMid-H"),
  START_MID_TO_I("StartMid-I"),
  START_MID_TO_J("StartMid-J"),
  START_MID_TO_K("StartMid-K"),
  START_MID_TO_L("StartMid-L"),

  // Start Bottom to ___
  START_BOTTOM_TO_E("StartBottom-E"),
  START_BOTTOM_TO_F("StartBottom-F"),

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

  // __ to HPS but slower and lewis or cole should rename this
  E_TO_HPS_RIGHTS("E-HPSRightS"),

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
  HPS_RIGHT_TO_L("HPSRight-L"),

  // HPS to ___ but slower I DONT KNOW
  HPS_RIGHT_TO_CS("HPSRight-CS");

  public final String name;

  private ChoreoPaths(String name) {
    this.name = name;
  }

  public static Iterator<String> pathSequence(ChoreoPaths... paths) {
    return Arrays.stream(paths).map(path -> path.name).iterator();
  }
}
