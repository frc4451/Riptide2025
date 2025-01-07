package frc.robot.subsystems.quest;

import java.util.concurrent.ConcurrentLinkedQueue;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.AdvantageKitConstants;
import frc.utils.VirtualSubsystem;

class QuestSubsystem extends VirtualSubsystem {
    private static record Quest(QuestIO io, QuestIOInputsAutoLogged inputs) {
    }

    private final Quest quest;

    private final ConcurrentLinkedQueue<TimestampedPose> questMeasurements = new ConcurrentLinkedQueue<>();

    public QuestSubsystem() {
        QuestIO io;
        if (AdvantageKitConstants.getMode() == AdvantageKitConstants.Mode.REAL) {
            io = new QuestReal();
        } else {
            io = new QuestIO() {
            };
        }

        quest = new Quest(io, new QuestIOInputsAutoLogged());
    }

    private double lastTimestamp = Double.NaN;

    @Override
    public void periodic() {
        final String logRoot = "Oculus/";

        quest.io.updateInputs(quest.inputs);
        Logger.processInputs(logRoot, quest.inputs);

        // If we have a duplicate frame, don't bother updating anything
        if (quest.inputs.timestamp == lastTimestamp) {
            return;
        }
        lastTimestamp = quest.inputs.timestamp;

        Logger.recordOutput(logRoot + "Pose", quest.inputs.pose);

        TimestampedPose estimatedPose = new TimestampedPose(quest.inputs.pose, quest.inputs.timestamp);

        questMeasurements.add(estimatedPose);
    }

    @Override
    public void simulationPeriodic() {
    }
}
