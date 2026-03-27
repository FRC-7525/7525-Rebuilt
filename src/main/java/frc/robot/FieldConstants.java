package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import kotlin.Pair;

public class FieldConstants {

	public static final Pair<Translation2d, Translation2d> BLUE_ALLIANCE_BOUNDS = new Pair<>(new Translation2d(0, 0), new Translation2d(4, 8.05));
	public static final Pair<Translation2d, Translation2d> RED_ALLIANCE_BOUNDS = new Pair<>(new Translation2d(12.54, 0), new Translation2d(16.57, 8.05));

	public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.625, 4.08, Rotation2d.kZero);
	public static final Pose2d RED_HUB_POSE = new Pose2d(11.92, 4.08, Rotation2d.kZero);
	public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
	public static final Pose2d TRENCH_POSE_LEFT_BLUE = FIELD_LAYOUT.getTagPose(23).get().toPose2d();
	public static final Pose2d TRENCH_POSE_RIGHT_BLUE = FIELD_LAYOUT.getTagPose(28).get().toPose2d();
	public static final Pose2d TRENCH_POSE_LEFT_RED = FIELD_LAYOUT.getTagPose(7).get().toPose2d();
	public static final Pose2d TRENCH_POSE_RIGHT_RED = FIELD_LAYOUT.getTagPose(12).get().toPose2d();
}
