package frc.robot.Subsystems.Vision;

import static frc.robot.Subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {

    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;
    private Set<Integer> targetTagIds = new HashSet<>();

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param robotToCamera The 3D position of the camera relative to the robot.
     */
    public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void setTargetTagIds(Set<Integer> ids) {
        this.targetTagIds = ids;
    }

    @Override
    public void logOutputs(VisionIOOutputs outputs) {
        outputs.connected = camera.isConnected();

        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();

        for (var result : camera.getAllUnreadResults()) {

            // When targetTagIds is set use lowest ambg tag from the set
            // When empty use best target

            PhotonTrackedTarget bestTarget = null;
            if (result.hasTargets()) {
                if (targetTagIds.isEmpty()) {
                    bestTarget = result.getBestTarget();
                } else {
                    bestTarget = result.getTargets().stream()
                        .filter(t -> targetTagIds.contains(t.getFiducialId()))
                        .min(Comparator.comparingDouble(PhotonTrackedTarget::getPoseAmbiguity))
                        .orElse(null);
                }
            }

            if (bestTarget != null) {
                outputs.latestTargetObservation = new TargetObservation(
                    Rotation2d.fromDegrees(bestTarget.getYaw()),
                    Rotation2d.fromDegrees(bestTarget.getPitch())
                );
            } else {
                outputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
            }

            boolean forceSingleTag = !targetTagIds.isEmpty();

        
            if (!forceSingleTag && result.multitagResult.isPresent()) {
                // Multitag
                var multitagResult = result.multitagResult.get();

                // Skip if if tower
                if (multitagResult.fiducialIDsUsed.size() == 1
                        && APRIL_TAG_IGNORE.contains(multitagResult.fiducialIDsUsed.get(0))) continue;

                // Calculate Pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
            	Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                double totalTagDistance = 0.0;
                double totalTagArea = 0.0;
                for (var target : result.getTargets()) {
                    totalTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                    totalTagArea += target.getArea();
                }

                // Set tag id to set
                Set<Short> observationTagIds = new HashSet<>();
                for (int id : multitagResult.fiducialIDsUsed) {
                    observationTagIds.add((short) id);
                }
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                poseObservations.add(
                    new PoseObservation(
                        result.getTimestampSeconds(),
                        robotPose,
                        multitagResult.estimatedPose.ambiguity,
                        multitagResult.fiducialIDsUsed.size(),
                        totalTagDistance / result.getTargets().size(),
                        PoseObservationType.PHOTONVISION,
                        totalTagArea / result.getTargets().size(),
                        observationTagIds
                    )
                );

            } else if (!result.getTargets().isEmpty()) {
                //  1 tag 
                PhotonTrackedTarget target = (forceSingleTag && bestTarget != null)
                    ? bestTarget
                    : result.getBestTarget();

                // If forcedbut no allowed tag is visiable skip 
                if (forceSingleTag && target == null) continue;

                var tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(
                        tagPose.get().getTranslation(),
                        tagPose.get().getRotation()
                    );
                    Transform3d cameraToTarget = target.getBestCameraToTarget();
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    tagIds.add(target.getFiducialId());
                    poseObservations.add(
                        new PoseObservation(
                            result.getTimestampSeconds(),
                            robotPose,
                            target.getPoseAmbiguity(),
                            1,
                            cameraToTarget.getTranslation().getNorm(),
                            PoseObservationType.PHOTONVISION,
                            target.getArea(),
                            Set.of((short) target.getFiducialId())
                        )
                    );
                }
            }
        }

        // Save pose you see to outputs
        outputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            outputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save  IDs to outputs
        outputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            outputs.tagIds[i++] = id;
        }
    }
}
