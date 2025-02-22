package frc.robot.subsystems.music;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {
  private final MusicIO io;
  private final MusicIOInputsAutoLogged inputs = new MusicIOInputsAutoLogged();

  private MusicTracks track;

  public Music(MusicIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  public void setTrack(MusicTracks track) {
    this.track = track;
  }

  public Command setTrackCommand(MusicTracks track) {
    return runOnce(() -> setTrack(track));
  }
}
