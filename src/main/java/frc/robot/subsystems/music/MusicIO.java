package frc.robot.subsystems.music;

import org.littletonrobotics.junction.AutoLog;

public interface MusicIO {
  @AutoLog
  public static class MusicIOInputs {
    public boolean isPlaying = false;
    public double currentTime = 0.0;
  }

  public default void updateInputs(MusicIOInputs inputs) {}

  public default void load(MusicTracks track) {}

  public default void play() {}

  public default void pause() {}

  public default void stop() {}
}
