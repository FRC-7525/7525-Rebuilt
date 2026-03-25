package frc.robot.Subsystems.Vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Drive.DriveConstants.SUBSYSTEM_NAME;
import static frc.robot.Subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.VisionIO.PoseObservation;
import frc.robot.Subsystems.Vision.VisionIO.VisionIOOutputs;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

	private final VisionIO[] io;
	private final VisionIOOutputs[] outputs;
	private final Alert[] disconnectedAlerts;

	List<Pose3d> allTagPoses = new LinkedList<>();
	List<Pose3d> allRobotPoses = new LinkedList<>();
	List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
	List<Pose3d> allRobotPosesRejected = new LinkedList<>();

	Set<Short> allianceHubTags = Robot.isRedAlliance ? RED_HUB_TAGS : BLUE_HUB_TAGS;
	Set<Short> allianceTrenchTags = Robot.isRedAlliance ? RED_TRENCH_SCORE_TAGS : BLUE_TRENCH_SCORE_TAGS;

	private static Vision instance;

	public static Vision getInstance() {
		if (instance == null) {
			instance = new Vision(
				switch (ROBOT_MODE) {
					case REAL -> new VisionIO[] { 
						new VisionIOPhotonVision(FRONT_CAM_1_NAME, ROBOT_TO_FRONT_CAM_1),
						new VisionIOPhotonVision(FRONT_CAM_2_NAME, ROBOT_TO_FRONT_CAM_2) 
					};
					case SIM -> new VisionIO[] {
						new VisionIOPhotonVisionSim(FRONT_CAM_1_NAME, ROBOT_TO_FRONT_CAM_1, Drive.getInstance()::getPose),
						new VisionIOPhotonVisionSim(FRONT_CAM_2_NAME, ROBOT_TO_FRONT_CAM_2, Drive.getInstance()::getPose),
					};
					case TESTING -> new VisionIO[] { new VisionIOPhotonVision(FRONT_CAM_1_NAME, ROBOT_TO_FRONT_CAM_1), new VisionIOPhotonVision(FRONT_CAM_2_NAME, ROBOT_TO_FRONT_CAM_2) };
				}
			);
		}
		return instance;
	}

	private Vision(VisionIO... io) {
		this.io = io;

		// Initialize outputs
		this.outputs = new VisionIOOutputs[io.length];
		for (int i = 0; i < outputs.length; i++) {
			outputs[i] = new VisionIOOutputs();
		}

		// Initialize disconnected alerts
		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < outputs.length; i++) {
			disconnectedAlerts[i] = new Alert("Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
		}
	}

	/**
	 * Returns the X angle to the best target, which can be used for simple servoing
	 * with vision.
	 *
	 * @param cameraIndex The index of the camera to use.
	 */
	public Rotation2d getTargetX(int cameraIndex) {
		return outputs[cameraIndex].latestTargetObservation.tx();
	}

	@Override
	public void periodic() {
		allianceHubTags = Robot.isRedAlliance ? RED_HUB_TAGS : BLUE_HUB_TAGS;
		allianceTrenchTags = Robot.isRedAlliance ? RED_TRENCH_SCORE_TAGS : BLUE_TRENCH_SCORE_TAGS;


		for (int i = 0; i < io.length; i++) {
			io[i].logOutputs(outputs[i]);
			Logger.recordOutput(SUBSYSTEM_NAME + "/Camera " + Integer.toString(i) + "/Connected", outputs[i].connected);
			Logger.recordOutput(SUBSYSTEM_NAME + "/Camera " + Integer.toString(i) + "/Latest Target X Deg", outputs[i].latestTargetObservation.tx().getDegrees());
			Logger.recordOutput(SUBSYSTEM_NAME + "/Camera " + Integer.toString(i) + "/Latest Target Y Deg", outputs[i].latestTargetObservation.ty().getDegrees());
			Logger.recordOutput(SUBSYSTEM_NAME + "/Camera " + Integer.toString(i) + "/Pose Observations Count", outputs[i].poseObservations.length);
			Logger.recordOutput(SUBSYSTEM_NAME + "/Camera " + Integer.toString(i) + "/Tag IDs", outputs[i].tagIds);
		}

		// Initialize logging values
		allTagPoses = new LinkedList<>();
		allRobotPoses = new LinkedList<>();
		allRobotPosesAccepted = new LinkedList<>();
		allRobotPosesRejected = new LinkedList<>();

		processVision();
		// Log summary data
		Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
		Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
		Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
		Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
	}

	private void processVision() {
		// Loop over cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alert
			disconnectedAlerts[cameraIndex].set(!outputs[cameraIndex].connected);

			// Initialize logging values
			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();

			// Add tag poses
			for (int tagId : outputs[cameraIndex].tagIds) {
				var tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(tagId);
				if (tagPose.isPresent()) {
					tagPoses.add(tagPose.get());
				}
			}

			// Loop over pose observations
			for (var observation : outputs[cameraIndex].poseObservations) {
				// Check whether to reject pose
				boolean rejectPose = shouldBeRejected(observation);

				Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/Tag Count", observation.tagCount() == 0);
				Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/Ambiguous", (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity));
				Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/Outside of Field X", observation.pose().getX() < 0.0 || observation.pose().getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength());
				Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/Outside of Field Y", observation.pose().getY() < 0.0 || observation.pose().getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth());
				Logger.recordOutput("Vision/Camera" + cameraIndex + "/SeenTrenchTags", seenTrenchTags(observation));


				// Add pose to log
				robotPoses.add(observation.pose());
				if (rejectPose) {
					robotPosesRejected.add(observation.pose());
				} else {
					robotPosesAccepted.add(observation.pose());
				}

				// Skip if rejected
				if (rejectPose) continue;

				//254 standard dev
				Matrix<N3, N1> visionStandardDev = calculateStandardDev(observation);

				// Send vision observation
				Drive.getInstance().addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp(), visionStandardDev);

			}
			
			// Log camera datadata
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses", robotPoses.toArray(new Pose3d[robotPoses.size()]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}
	}

	private boolean shouldBeRejected(PoseObservation observation) {
		boolean observedLadder = false;
		boolean observedOutpost = false;

		for (Short tagObserved : observation.tagsObserved()) {
			if (APRIL_TAG_IGNORE.contains(tagObserved)) {
				observedLadder = true;
				break;
			}
			if (observedLadder) break;
		}

		for (Short tagObserved : observation.tagsObserved()) {
			if (HUMAN_TAGS.contains(tagObserved)) {
				observedOutpost = true;
				break;
			}
			if (observedOutpost) break;
		}

		return (
			observation.tagCount() == 0 || // Must have at least one tag
			(observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) || // Cannot be high ambiguity
			Math.abs(observation.pose().getZ()) > maxZError || // Must have realistic Z coordinate
			// Must be within the field boundaries
			observation.pose().getX() <
			0.0 ||
			observation.pose().getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength() ||
			observation.pose().getY() < 0.0 ||
			observation.pose().getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth() ||
			Math.abs(Units.radiansToDegrees(Drive.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond)) > MAX_ANGULAR_VELOCITY.in(DegreesPerSecond)
		); // Robot must not be rotating rapidly
	}

	public Matrix<N3, N1> calculateStandardDev(PoseObservation observation) {
		double xyStds;
		double degStds;
		if (observation.tagCount() == 1) {
			double poseDifference = observation.pose().getTranslation().toTranslation2d().getDistance(Drive.getInstance().getPose().getTranslation());
			
			// 1 target with large area and close to estimated pose
			if (observation.avgTagArea() > 0.8 && poseDifference < 0.5) {
				xyStds = 0.5;
			}
			// 1 target farther away and estimated pose is close
			else if (observation.avgTagArea() > 0.1 && poseDifference < 0.3) {
				xyStds = 1.0;
			} else {
				xyStds = 2.0;
			}
			return VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(50)); // I dont even know, ts so random
		} else {
			xyStds = 0.5;
			degStds = 6;
			return VecBuilder.fill(xyStds, xyStds, degStds);
		}
	}

	private boolean seenTrenchTags(PoseObservation observation) {
    if (observation.tagsObserved().isEmpty()) return false;

    for (Short tagObserved : observation.tagsObserved()) {
        if (allianceTrenchTags.contains(tagObserved)) {
            return true;
        }
    }
    return false;
	}
}
