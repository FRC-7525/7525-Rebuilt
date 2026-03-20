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
import frc.robot.Subsystems.Manager.Manager;
import frc.robot.Subsystems.Manager.ManagerStates;
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
	List<Pose3d> allTagPosesAccepted = new LinkedList<>();

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
                        // new VisionIOPhotonVisionSim(FRONT_CAM_2_NAME, ROBOT_TO_FRONT_CAM_2, Drive.getInstance()::getPose),
                    };
                    case TESTING -> new VisionIO[] {
                        new VisionIOPhotonVision(FRONT_CAM_1_NAME, ROBOT_TO_FRONT_CAM_1),
                        new VisionIOPhotonVision(FRONT_CAM_2_NAME, ROBOT_TO_FRONT_CAM_2)
                    };
                }
            );
        }
        return instance;
    }

    private Vision(VisionIO... io) {
        this.io = io;

        this.outputs = new VisionIOOutputs[io.length];
        for (int i = 0; i < outputs.length; i++) {
            outputs[i] = new VisionIOOutputs();
        }

        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < outputs.length; i++) {
            disconnectedAlerts[i] = new Alert(
                "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return outputs[cameraIndex].latestTargetObservation.tx();
    }

    @Override
    public void periodic() {
        allianceHubTags = Robot.isRedAlliance ? RED_HUB_TAGS : BLUE_HUB_TAGS;

        //  which tag ids to restrict to based on state + alliance zone position.
        Set<Integer> targetTagIds = resolveTargetTagIds();
        for (VisionIO visionIO : io) {
            visionIO.setTargetTagIds(targetTagIds);
        }

        for (int i = 0; i < io.length; i++) {
            io[i].logOutputs(outputs[i]);
            Logger.recordOutput(SUBSYSTEM_NAME + "/Camera " + Integer.toString(i) + "/Connected", outputs[i].connected);
            Logger.recordOutput(SUBSYSTEM_NAME + "/Camera " + Integer.toString(i) + "/Latest Target X Deg", outputs[i].latestTargetObservation.tx().getDegrees());
            Logger.recordOutput(SUBSYSTEM_NAME + "/Camera " + Integer.toString(i) + "/Latest Target Y Deg", outputs[i].latestTargetObservation.ty().getDegrees());
            Logger.recordOutput(SUBSYSTEM_NAME + "/Camera " + Integer.toString(i) + "/Pose Observations Count", outputs[i].poseObservations.length);
            Logger.recordOutput(SUBSYSTEM_NAME + "/Camera " + Integer.toString(i) + "/Tag IDs", outputs[i].tagIds);
        }

        allTagPoses = new LinkedList<>();
		allTagPosesAccepted = new LinkedList<>();
        allRobotPoses = new LinkedList<>();
        allRobotPosesAccepted = new LinkedList<>();
        allRobotPosesRejected = new LinkedList<>();

        processVision();

        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
		Logger.recordOutput("Vision/Summary/TagPosesAccepted", allTagPosesAccepted.toArray(new Pose3d[0]));
    }

  
    private Set<Integer> resolveTargetTagIds() {
        
        ManagerStates state = Manager.getInstance().getState();
        boolean isShootingState = (state == ManagerStates.WINDING_UP || state == ManagerStates.SCORING_AUTO);
        

        if (!isShootingState) return Set.of();

        double robotX = Drive.getInstance().getPose().getX();
        double fieldLength = APRIL_TAG_FIELD_LAYOUT.getFieldLength(); // 16.54m

        if (Robot.isRedAlliance) {
    	return (robotX > fieldLength - ALLIANCE_ZONE_DEPTH) ? RED_TRENCH_SCORE_TAGS : Set.of();
		} else {
    	return (robotX < ALLIANCE_ZONE_DEPTH) ? BLUE_TRENCH_SCORE_TAGS : Set.of();
		}
    }

    private void processVision() {
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            disconnectedAlerts[cameraIndex].set(!outputs[cameraIndex].connected);

            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            for (int tagId : outputs[cameraIndex].tagIds) {
                var tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            for (var observation : outputs[cameraIndex].poseObservations) {
                boolean rejectPose = shouldBeRejected(observation);

                Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/Tag Count", observation.tagCount() == 0);
                Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/Ambiguous", (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity));
                Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/Outside of Field X", observation.pose().getX() < 0.0 || observation.pose().getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength());
                Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/Outside of Field Y", observation.pose().getY() < 0.0 || observation.pose().getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth());

                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());

					for (Short id : observation.tagsObserved()) {
						var tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(id);
						if (tagPose.isPresent() && allianceTrenchTags.contains(id)) {
							allTagPosesAccepted.add(tagPose.get());
						}
					}
				}

                if (rejectPose) continue;

                Matrix<N3, N1> visionStandardDev = calculateStandardDev(observation);
                Drive.getInstance().addVisionMeasurement(
                    observation.pose().toPose2d(), observation.timestamp(), visionStandardDev);
            }

            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
			allTagPosesAccepted.addAll(tagPoses);
        }
	}
    

    private boolean shouldBeRejected(PoseObservation observation) {
    boolean observedTower = false;
    boolean observedHumanStation = false;

    for (Short tagObserved : observation.tagsObserved()) {
        if (APRIL_TAG_IGNORE.contains(tagObserved)) {
            observedTower = true;
            break;
        }
    }

    if (!observedTower) {
        for (Short tagObserved : observation.tagsObserved()) {
            if (HUMAN_TAGS.contains(tagObserved)) {
                observedHumanStation = true;
                break;
            }
        }
    }

    return (
        observation.tagCount() == 0 ||
        (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) ||
        Math.abs(observation.pose().getZ()) > maxZError ||
        observation.pose().getX() < 0.0 ||
        observation.pose().getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength() ||
        observation.pose().getY() < 0.0 ||
        observation.pose().getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth() ||
        Math.abs(Units.radiansToDegrees(
            Drive.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond))
                > MAX_ANGULAR_VELOCITY.in(DegreesPerSecond) ||
        observedTower ||
        observedHumanStation
    );
}

    public Matrix<N3, N1> calculateStandardDev(PoseObservation observation) {
        double xyStds;
        double degStds;
        if (observation.tagCount() == 1) {
            double poseDifference = observation.pose().getTranslation()
                .toTranslation2d()
                .getDistance(Drive.getInstance().getPose().getTranslation());

            if (seenReefTags(observation) && observation.avgTagArea() > 0.2) {
                xyStds = 0.5;
            } else if (observation.avgTagArea() > 0.8 && poseDifference < 0.5) {
                xyStds = 0.5;
            } else if (observation.avgTagArea() > 0.1 && poseDifference < 0.3) {
                xyStds = 1.0;
            } else {
                xyStds = 2.0;
            }
            return VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(50));
        } else {
            xyStds = 0.5;
            degStds = 6;
            return VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));
        }
    }

    private boolean seenReefTags(PoseObservation observation) {
        return allianceHubTags.contains(observation.tagsObserved().toArray()[0]);
    }
}
