package frc.robot.subsystems.vision;

import edu.wpi.first.util.protobuf.ProtobufSerializable;
import frc.robot.subsystems.vision.proto.PhotonPipelineResultsProto;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonPipelineResults implements ProtobufSerializable {
  public final List<PhotonPipelineResult> results;

  /** Default constructor for when no values are set */
  public PhotonPipelineResults() {
    results = List.of();
  }

  public PhotonPipelineResults(List<PhotonPipelineResult> results) {
    this.results = results;
  }

  public static final PhotonPipelineResultsProto proto = new PhotonPipelineResultsProto();
}
