package frc.robot.Subsystems.Manager;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public enum CurrentLimiterStates {
	// Only has shooter, drive, and turn bc these are the only ones that change
	IDLE(CurrentLimitConstants.NORMAL_DRIVE_LIMITS, CurrentLimitConstants.NORMAL_TURN_LIMITS, CurrentLimitConstants.IDLE_SHOOTER_LIMITS),
	EXTENDED_IDLE(CurrentLimitConstants.NORMAL_DRIVE_LIMITS, CurrentLimitConstants.NORMAL_TURN_LIMITS, CurrentLimitConstants.IDLE_SHOOTER_LIMITS),
	INTAKING(CurrentLimitConstants.LOW_DRIVE_LIMITS, CurrentLimitConstants.LOW_TURN_LIMITS, CurrentLimitConstants.INTAKE_WHEEL_LIMITS),
	WINDING_UP(CurrentLimitConstants.MEDIUM_DRIVE_LIMITS, CurrentLimitConstants.MEDIUM_TURN_LIMITS, CurrentLimitConstants.WINDUP_SHOOTER_LIMITS),
	WINDING_UP_FIXED_SHOT(CurrentLimitConstants.MEDIUM_DRIVE_LIMITS, CurrentLimitConstants.MEDIUM_TURN_LIMITS, CurrentLimitConstants.WINDUP_SHOOTER_LIMITS),
	SHUTTLING(CurrentLimitConstants.LOW_DRIVE_LIMITS, CurrentLimitConstants.LOW_TURN_LIMITS, CurrentLimitConstants.FIRING_SHOOTER_LIMITS),
	SHOOTING_HUB(CurrentLimitConstants.SHOOTING_DRIVE_LIMITS, CurrentLimitConstants.LOW_TURN_LIMITS, CurrentLimitConstants.FIRING_SHOOTER_LIMITS),
	SHOOTING_FIXED(CurrentLimitConstants.SHOOTING_DRIVE_LIMITS, CurrentLimitConstants.LOW_TURN_LIMITS, CurrentLimitConstants.FIRING_SHOOTER_LIMITS),
	EXTENDING_CLIMBER(CurrentLimitConstants.NORMAL_DRIVE_LIMITS, CurrentLimitConstants.NORMAL_TURN_LIMITS, CurrentLimitConstants.IDLE_SHOOTER_LIMITS),
	RETRACTING_CLIMBER(CurrentLimitConstants.NORMAL_DRIVE_LIMITS, CurrentLimitConstants.NORMAL_TURN_LIMITS, CurrentLimitConstants.IDLE_SHOOTER_LIMITS),
	INTAKING_AND_SHOOTING_AUTO(CurrentLimitConstants.LOW_DRIVE_LIMITS, CurrentLimitConstants.LOW_TURN_LIMITS, CurrentLimitConstants.FIRING_SHOOTER_LIMITS);
	
	private CurrentLimitsConfigs driveLimit;
	private CurrentLimitsConfigs turnLimit;
	private CurrentLimitsConfigs shooterLimit;

	private CurrentLimiterStates(CurrentLimitsConfigs driveLimit, CurrentLimitsConfigs turnLimit, CurrentLimitsConfigs shooterLimit) {
		this.driveLimit = driveLimit;
		this.turnLimit = turnLimit;
		this.shooterLimit = shooterLimit;
	}

	public CurrentLimitsConfigs getDriveLimit() {
		return driveLimit;
	}

	public CurrentLimitsConfigs getTurnLimit() {
		return turnLimit;
	}

	public CurrentLimitsConfigs getShooterLimit() {
		return shooterLimit;
	}
}
