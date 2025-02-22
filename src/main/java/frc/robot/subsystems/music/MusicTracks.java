package frc.robot.subsystems.music;

public enum MusicTracks {
  MEGALOVANIA("songs/megalovania.chrp");

  public final String filePath;

  private MusicTracks(String filePath) {
    this.filePath = filePath;
  }
}
