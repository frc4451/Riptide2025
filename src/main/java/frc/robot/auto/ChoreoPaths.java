package frc.robot.auto;

import java.util.Arrays;
import java.util.Iterator;

public enum ChoreoPaths {
  TWO_METERS("2 Meters"),
  THREE_METERS("3 Meters"),
  FIVE_METERS("5 Meters"),
  CURVY("Curvy"),

  // Start Top to ___
  START_TOP_TO_IL4("StartTop-IL4"),

  // Start Mid to ___
  START_MID_TO_GL4("StartMid-GL4"),
  START_MID_TO_HL4("StartMid-HL4"),

  // Start Bottom to ___
  START_BOTTOM_TO_EL2("StartBottom-EL2"),
  START_BOTTOM_TO_EL4("StartBottom-EL4"),
  START_BOTTOM_TO_FL2("StartBottom-FL2"),
  START_BOTTOM_TO_FL4("StartBottom-FL4"),
  START_RIGHT_TO_FL4_NO_STOP("StartRight-FL4-NoStop"),
  START_LEFT_TO_IL4_NO_STOP("StartLeft-IL4-NoStop"),

  // ___ to HPS Right
  BL4_TO_HPS_RIGHT("BL4-HPSRight"),
  CL4_TO_HPS_RIGHT("CL4-HPSRight"),
  CL4_TO_HPS_RIGHT_NO_STOP("CL4-HPSRight-NoStop"),
  DL4_TO_HPS_RIGHT("DL4-HPSRight"),
  DL4_TO_HPS_RIGHT_NO_STOP("DL4-HPSRight-NoStop"),
  EL2_TO_HPS_RIGHT("EL2-HPSRight"),
  EL4_TO_HPS_RIGHT("EL4-HPSRight"),
  EL4_TO_HPS_RIGHT_NO_STOP("EL4-HPSRight-NoStop"),
  FL2_TO_HPS_RIGHT("FL2-HPSRight"),
  FL4_TO_HPS_RIGHT_NO_STOP("FL4-HPSRight-NoStop"),
  FL4_TO_HPS_RIGHT("FL4-HPSRight"),
  GL4_TO_HPS_RIGHT("GL4-HPSRight"),

  // HPS Right to ___
  HPS_RIGHT_TO_BL4("HPSRight-BL4"),
  HPS_RIGHT_TO_CL4("HPSRight-CL4"),
  HPS_RIGHT_TO_CL4_NO_STOP("HPSRight-CL4-NoStop"),
  HPS_RIGHT_TO_DL4("HPSRight-DL4"),
  HPS_RIGHT_TO_DL4_NO_STOP("HPSRight-DL4-NoStop"),
  HPS_RIGHT_TO_EL2("HPSRight-EL2"),
  HPS_RIGHT_TO_EL4("HPSRight-EL4"),
  HPS_RIGHT_TO_FL4("HPSRight-FL4"),
  HPS_RIGHT_TO_GL4("HPSRight-GL4"),

  // ___ to HPS Left
  AL4_TO_HPS_LEFT("AL4-HPSLeft"),
  HL4_TO_HPS_LEFT("HL4-HPSLeft"),
  IL4_TO_HPS_LEFT("IL4-HPSLeft"),
  IL4_TO_HPS_LEFT_NO_STOP("IL4-HPSLeft-NoStop"),
  JL4_TO_HPS_LEFT_NO_STOP("JL4-HPSLeft-NoStop"),
  JL4_TO_HPS_LEFT("JL4-HPSLeft"),
  KL4_TO_HPS_LEFT("KL4-HPSLeft"),
  KL4_TO_HPS_LEFT_NO_STOP("KL4-HPSLeft-NoStop"),
  LL4_TO_HPS_LEFT("LL4-HPSLeft"),
  LL4_TO_HPS_LEFT_NO_STOP("LL4-HPSLeft-NoStop"),

  // HPS Left to ___
  HPS_LEFT_TO_KL4("HPSLeft-KL4"),
  HPS_LEFT_TO_KL4_NO_STOP("HPSLeft-KL4-NoStop"),
  HPS_LEFT_TO_LL4("HPSLeft-LL4"),
  HPS_LEFT_TO_LL4_NO_STOP("HPSLeft-LL4-NoStop"),
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
