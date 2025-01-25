package frc.robot.subsystems.vision.proto;

import edu.wpi.first.util.protobuf.Protobuf;
import frc.robot.proto.Riptide.ProtobufPhotonPipelineResults;
import frc.robot.subsystems.vision.PhotonPipelineResults;
import java.util.ArrayList;
import org.photonvision.proto.Photon.ProtobufPhotonPipelineResult;
import org.photonvision.targeting.PhotonPipelineResult;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class PhotonPipelineResultsProto
    implements Protobuf<PhotonPipelineResults, ProtobufPhotonPipelineResults> {
  @Override
  public Class<PhotonPipelineResults> getTypeClass() {
    return PhotonPipelineResults.class;
  }

  @Override
  public Descriptor getDescriptor() {
    return ProtobufPhotonPipelineResults.getDescriptor();
  }

  @Override
  public ProtobufPhotonPipelineResults createMessage() {
    return ProtobufPhotonPipelineResults.newInstance();
  }

  @Override
  public PhotonPipelineResults unpack(ProtobufPhotonPipelineResults msg) {
    ArrayList<PhotonPipelineResult> results = new ArrayList<>(msg.getResults().length());
    for (ProtobufPhotonPipelineResult resultProto : msg.getResults()) {
      results.add(PhotonPipelineResult.proto.unpack(resultProto));
    }
    return new PhotonPipelineResults(results);
  }

  @Override
  public void pack(ProtobufPhotonPipelineResults msg, PhotonPipelineResults value) {
    var results = msg.getMutableResults().reserve(value.results.size());
    for (PhotonPipelineResult result : value.results) {
      var resultProto = results.next();
      PhotonPipelineResult.proto.pack(resultProto, result);
    }
  }
}
