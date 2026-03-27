package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.FieldConstants.*;
import static frc.robot.GlobalConstants.ROBOT_TO_SHOOTER_2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ShooterMath {

	public static Angle solveHubShot(Pose2d currentPose) {
		Logger.recordOutput("Shooter/Math/Target", Robot.isRedAlliance ? RED_HUB_POSE : BLUE_HUB_POSE);
		Logger.recordOutput("Shooter/Math/Shooter", currentPose.transformBy(ROBOT_TO_SHOOTER_2D));
		Logger.recordOutput("Shooter/Math/Translation", Robot.isRedAlliance ? RED_HUB_POSE.relativeTo(currentPose.transformBy(ROBOT_TO_SHOOTER_2D)) : BLUE_HUB_POSE.relativeTo(currentPose.transformBy(ROBOT_TO_SHOOTER_2D)));
		Logger.recordOutput("Shooter/Math/Distance", Robot.isRedAlliance ? RED_HUB_POSE.relativeTo(currentPose.transformBy(ROBOT_TO_SHOOTER_2D)).getTranslation().getNorm() : BLUE_HUB_POSE.relativeTo(currentPose.transformBy(ROBOT_TO_SHOOTER_2D)).getTranslation().getNorm());

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
