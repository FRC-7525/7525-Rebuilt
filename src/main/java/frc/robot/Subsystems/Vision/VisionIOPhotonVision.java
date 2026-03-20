package frc.robot.Subsystems.Vision;

import static frc.robot.Subsystems.Vision.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
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
    public void logOutputs(VisionIOOutputs outputs) {
        outputs.connected = camera.isConnected();

        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
		List<Integer> tagIdsList = new LinkedList<>();

        for (var result : camera.getAllUnreadResults()) {

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

            // Force single tag
            boolean forceSingleTag = !targetTagIds.isEmpty();

            if (!forceSingleTag && result.multitagResult.isPresent()) {
				//multitag
				var multitagResult = result.multitagResult.get();

				// Calculate robot pose (Field -> Camera -> Robot)
				Pose3d robotPose = new Pose3d(multitagResult.estimatedPose.best.getTranslation(), multitagResult.estimatedPose.best.getRotation())
						.transformBy(robotToCamera.inverse());

				// Calculate average tag distance and avg tag area
				double totalTagDistance = 0.0;
				double totalTagArea = 0.0;
				for (var target : result.getTargets()) {

					totalTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
					totalTagArea += target.getArea();
				}

				for (int id : multitagResult.fiducialIDsUsed) {
					tagIdsList.add(id);
				}
				// Add tag IDs and observation
				tagIds.addAll(tagIdsList);
				poseObservations.add(
					new PoseObservation(
						result.getTimestampSeconds(),
						robotPose,
						multitagResult.estimatedPose.ambiguity,
						multitagResult.fiducialIDsUsed.size(),
						totalTagDistance / result.getTargets().size(),
						PoseObservationType.PHOTONVISION,
						totalTagArea / result.getTargets().size(),
						new HashSet<Short>(multitagResult.fiducialIDsUsed)
					)
				);

            } else if (!result.getTargets().isEmpty()) {
                // in force single tag use the best target in list
                PhotonTrackedTarget target = (forceSingleTag && bestTarget != null)
                    ? bestTarget
                    : result.getBestTarget();

                if (forceSingleTag && target == null) continue;

                var tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    // Calculate robot pose 
                    Transform3d cameraToTarget = target.getBestCameraToTarget();
                    Pose3d robotPose = tagPose.get()
                            .transformBy(target.getBestCameraToTarget().inverse())
                            .transformBy(robotToCamera.inverse());

                    // Add tag ID and observation
                    tagIds.add(target.getFiducialId());
                    poseObservations.add(
                        new PoseObservation(
                            result.getTimestampSeconds(),
                            robotPose,
                            target.getPoseAmbiguity(),
                            1,
                            target.getBestCameraToTarget().getTranslation().getNorm(),
                            PoseObservationType.PHOTONVISION,
                            target.getArea(),
                            Set.of(target.getFiducialId())
                        )
                    );
                }
            }
        }

        // Save pose observations to outputs
        outputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            outputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to outputs
        outputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            outputs.tagIds[i++] = id;
        }
    }

    @Override
    public void setTargetTagIds(Set<Integer> ids) {
        this.targetTagIds = ids;
    }
}
