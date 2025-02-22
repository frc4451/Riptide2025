package frc.robot.subsystems.music;

import com.ctre.phoenix6.Orchestra;

public class MusicIOOrchestra implements MusicIO {
  private final Orchestra orchestra = new Orchestra();

  public MusicIOOrchestra() {
    for (Instruments instrument : Instruments.values()) {
      orchestra.addInstrument(instrument.deviceSource.get(), instrument.trackNumber);
    }
  }

  @Override
  public void updateInputs(MusicIOInputs inputs) {
    inputs.isPlaying = orchestra.isPlaying();
    inputs.currentTime = orchestra.getCurrentTime();
  }

  @Override
  public void load(MusicTracks track) {
    orchestra.loadMusic(track.filePath);
  }

  @Override
  public void play() {
    orchestra.play();
  }

  @Override
  public void pause() {
    orchestra.pause();
  }

  @Override
  public void stop() {
    orchestra.stop();
  }
}
