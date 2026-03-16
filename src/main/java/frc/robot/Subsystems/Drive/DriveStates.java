package frc.robot.Subsystems.Drive;

import static frc.robot.Subsystems.Drive.AutoAlign.AutoAlignConstants.*;
import static frc.robot.Subsystems.Shooter.ShooterConstants.BLUE_HUB_POSE;
import static frc.robot.Subsystems.Shooter.ShooterConstants.RED_HUB_POSE;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems.Drive.AutoAlign.PosePair;
import org.team7525.subsystem.SubsystemStates;

/**
 * An enumeration representing different drive states for a robot's drive subsystem.
 */
public enum DriveStates implements SubsystemStates {
	NORMAL("Driving Normally", new PosePair(Pose2d.kZero, Pose2d.kZero)),
	AIMLOCK_HUB("Locking Aim To Hub", new PosePair(RED_HUB_POSE, BLUE_HUB_POSE)),
	AIMLOCK_ALLIANCE_LEFT_DEEP("Locking Aim To deep left side of alliance station", AIMLOCK_LEFT_DEEP_POSES),
	AIMLOCK_ALLIANCE_LEFT_SHALLOW("Locking Aim To shallow left side of alliance station", AIMLOCK_LEFT_SHALLOW_POSES),
	AIMLOCK_ALLIANCE_RIGHT_DEEP("Locking Aim To deep right side of alliance station", AIMLOCK_RIGHT_DEEP_POSES),
	AIMLOCK_ALLIANCE_RIGHT_SHALLOW("Locking Aim To shallow right side of alliance station", AIMLOCK_RIGHT_SHALLOW_POSES),
	AA_TOWER_LEFT("Going to left side of tower", TOWER_LEFT),
	AA_TOWER_RIGHT("Going to right side of tower", TOWER_RIGHT),
	AA_NEUTRAL("Going to neutral zone", NEUTRAL_POSES),
	SNAKE_DRIVE("Snake Drive Mode", new PosePair(Pose2d.kZero, Pose2d.kZero));
	AUTO("test", NEUTRAL_POSES);

	private String stateString;
	private PosePair targetPosePair;

	/**
	 * Constructs a DriveStates enum value with the specified state string and field-relative flag.
	 *
	 * @param stateString    the string representation of the drive state
	 * @param Runnable  runnable that gets called to drive the robot
	 */
	DriveStates(String stateString, PosePair targetPosePair) {
		this.stateString = stateString;
		this.targetPosePair = targetPosePair;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public PosePair getTargetPosePair() {
		return targetPosePair;
	}
}
