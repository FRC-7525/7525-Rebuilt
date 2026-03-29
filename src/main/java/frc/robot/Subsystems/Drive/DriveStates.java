package frc.robot.Subsystems.Drive;

import static frc.robot.FieldConstants.*;
import static frc.robot.Subsystems.Drive.AutoAlign.AutoAlignConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems.Drive.AutoAlign.PosePair;
import org.team7525.subsystem.SubsystemStates;

public enum DriveStates implements SubsystemStates {
	NORMAL("Driving Normally", new PosePair(Pose2d.kZero, Pose2d.kZero)),
	AIMLOCK_HUB("Locking Aim To Hub", new PosePair(RED_HUB_POSE, BLUE_HUB_POSE)),
	AIMLOCK_ALLIANCE_LEFT_DEEP("Locking Aim To deep left side of alliance station", AIMLOCK_LEFT_DEEP_POSES),
	AIMLOCK_ALLIANCE_LEFT_SHALLOW("Locking Aim To shallow left side of alliance station", AIMLOCK_LEFT_SHALLOW_POSES),
	AIMLOCK_ALLIANCE_RIGHT_DEEP("Locking Aim To deep right side of alliance station", AIMLOCK_RIGHT_DEEP_POSES),
	AIMLOCK_ALLIANCE_RIGHT_SHALLOW("Locking Aim To shallow right side of alliance station", AIMLOCK_RIGHT_SHALLOW_POSES),
	AA_TRENCH_LEFT("Going to left side of tower", TRENCH_LEFT),
	AA_TRENCH_RIGHT("Going to right side of tower", TRENCH_RIGHT),
	AA_NEUTRAL("Going to neutral zone", NEUTRAL_POSES),
	SNAKE_DRIVE("Snake Drive Mode", new PosePair(Pose2d.kZero, Pose2d.kZero)),
	AUTO("test", NEUTRAL_POSES),
	LOCKED_X_POSE("Locking X Pose", new PosePair(Pose2d.kZero, Pose2d.kZero));

	private String stateString;
	private PosePair targetPosePair;

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
