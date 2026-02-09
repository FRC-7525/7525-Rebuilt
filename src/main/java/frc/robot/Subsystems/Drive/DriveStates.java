package frc.robot.Subsystems.Drive;

import static frc.robot.Subsystems.Drive.AutoAlign.AutoAlignConstants.*;

import frc.robot.Subsystems.Drive.AutoAlign.PosePair;
import org.team7525.subsystem.SubsystemStates;

/**
 * An enumeration representing different drive states for a robot's drive subsystem.
 */
public enum DriveStates implements SubsystemStates {
	NORMAL("Driving Normally", PosePair.kZero),
	AIMLOCK_HUB("Locking Aim", PosePair.kZero),
	AA_TOWER_LEFT("Going to left side of tower", TOWER_LEFT),
	AA_TOWER_RIGHT("Going to right side of tower", TOWER_RIGHT),
	AA_NEUTRAL("Going to neutral zone", NEUTRAL_POSES);

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
