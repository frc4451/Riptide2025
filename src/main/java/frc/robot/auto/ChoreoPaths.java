package frc.robot.auto;

import java.util.Arrays;
import java.util.Iterator;

public enum ChoreoPaths {
  TWO_METERS("2 Meters"),
  THREE_METERS("3 Meters"),
  FIVE_METERS("5 Meters"),
  CURVY("Curvy"),

  // Start Mid to ___
  START_MID_TO_AL4("StartMid-AL4"),
  START_MID_TO_BL4("StartMid-BL4"),
  START_MID_TO_CL4("StartMid-CL4"),
  START_MID_TO_DL4("StartMid-DL4"),
  START_MID_TO_EL4("StartMid-EL4"),
  START_MID_TO_FL4("StartMid-FL4"),
  START_MID_TO_GL4("StartMid-GL4"),
  START_MID_TO_HL4("StartMid-HL4"),
  START_MID_TO_IL4("StartMid-IL4"),
  START_MID_TO_JL4("StartMid-JL4"),
  START_MID_TO_KL4("StartMid-KL4"),
  START_MID_TO_LL4("StartMid-LL4"),

  // Start Bottom to ___
  START_BOTTOM_TO_EL4("StartBottom-EL4"),
  START_BOTTOM_TO_FL4("StartBottom-FL4"),

  // ___ to HPS
  AL4_TO_HPS_RIGHT("AL4-HPSRight"),
  BL4_TO_HPS_RIGHT("BL4-HPSRight"),
  CL4_TO_HPS_RIGHT("CL4-HPSRight"),
  DL4_TO_HPS_RIGHT("DL4-HPSRight"),
  EL4_TO_HPS_RIGHT("EL4-HPSRight"),
  FL4_TO_HPS_RIGHT("FL4-HPSRight"),
  GL4_TO_HPS_RIGHT("GL4-HPSRight"),
  HL4_TO_HPS_RIGHT("HL4-HPSRight"),
  IL4_TO_HPS_RIGHT("IL4-HPSRight"),
  JL4_TO_HPS_RIGHT("JL4-HPSRight"),
  KL4_TO_HPS_RIGHT("KL4-HPSRight"),
  LL4_TO_HPS_RIGHT("LL4-HPSRight"),

  // HPS to ___
  HPS_RIGHT_TO_AL4("HPSRight-AL4"),
  HPS_RIGHT_TO_BL4("HPSRight-BL4"),
  HPS_RIGHT_TO_CL4("HPSRight-CL4"),
  HPS_RIGHT_TO_DL4("HPSRight-DL4"),
  HPS_RIGHT_TO_EL4("HPSRight-EL4"),
  HPS_RIGHT_TO_FL4("HPSRight-FL4"),
  HPS_RIGHT_TO_GL4("HPSRight-GL4"),
  HPS_RIGHT_TO_HL4("HPSRight-HL4"),
  HPS_RIGHT_TO_IL4("HPSRight-IL4"),
  HPS_RIGHT_TO_JL4("HPSRight-JL4"),
  HPS_RIGHT_TO_KL4("HPSRight-KL4"),
  HPS_RIGHT_TO_LL4("HPSRight-LL4"),
//
;

  public final String name;

  private ChoreoPaths(String name) {
    this.name = name;
  }

  public static Iterator<String> pathSequence(ChoreoPaths... paths) {
    return Arrays.stream(paths).map(path -> path.name).iterator();
  }
}
