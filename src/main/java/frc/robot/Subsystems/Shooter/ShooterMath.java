package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.FieldConstants.*;
import static frc.robot.GlobalConstants.ROBOT_TO_SHOOTER_2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;

public class ShooterMath {

	public static Angle solveHubShot(Pose2d currentPose) {
		return Degrees.of(
			Robot.isRedAlliance ? ShooterTables.ALLIANCE_TABLE.get(RED_HUB_POSE.relativeTo(currentPose.transformBy(ROBOT_TO_SHOOTER_2D)).getTranslation().getNorm()) : ShooterTables.ALLIANCE_TABLE.get(BLUE_HUB_POSE.relativeTo(currentPose.transformBy(ROBOT_TO_SHOOTER_2D)).getTranslation().getNorm())
		);
	}

	public static Angle solveAllianceShot(Pose2d currentPose) {
		return Degrees.of(
			Robot.isRedAlliance ? ShooterTables.SHUTTLING_TABLE.get(RED_HUB_POSE.relativeTo(currentPose.transformBy(ROBOT_TO_SHOOTER_2D)).getTranslation().getNorm()) : ShooterTables.ALLIANCE_TABLE.get(BLUE_HUB_POSE.relativeTo(currentPose.transformBy(ROBOT_TO_SHOOTER_2D)).getTranslation().getNorm())
		);
	}
}
