package frc.robot.field;

import frc.robot.bobot_state.BobotState;

public class Barge {
  public static final Cages blue = new Cages(FieldConstants.blueBarge);
  public static final Cages blueOpposing = new Cages(FieldConstants.blueBargeOpposing);
  public static final Cages red = new Cages(FieldConstants.redBarge);
  public static final Cages redOpposing = new Cages(FieldConstants.redBargeOpposing);

  public static Cages get() {
    boolean onAllianceSide = FieldUtils.onAllianceSide(BobotState.getGlobalPose(), 0);
    return FieldUtils.isBlueAlliance()
        ? (onAllianceSide ? blue : blueOpposing)
        : (onAllianceSide ? red : redOpposing);
  }
}
