package frc.robot.Subsystems.Manager;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class CurrentLimitConstants {

	public static final CurrentLimitsConfigs NORMAL_DRIVE_LIMITS = new CurrentLimitsConfigs();
	public static final CurrentLimitsConfigs MEDIUM_DRIVE_LIMITS = new CurrentLimitsConfigs();
	public static final CurrentLimitsConfigs LOW_DRIVE_LIMITS = new CurrentLimitsConfigs();
	public static final CurrentLimitsConfigs SHOOTING_DRIVE_LIMITS = new CurrentLimitsConfigs();

	public static final CurrentLimitsConfigs NORMAL_TURN_LIMITS = new CurrentLimitsConfigs();
	public static final CurrentLimitsConfigs MEDIUM_TURN_LIMITS = new CurrentLimitsConfigs();
	public static final CurrentLimitsConfigs LOW_TURN_LIMITS = new CurrentLimitsConfigs();

	public static final CurrentLimitsConfigs IDLE_SHOOTER_LIMITS = new CurrentLimitsConfigs();
	public static final CurrentLimitsConfigs WINDUP_SHOOTER_LIMITS = new CurrentLimitsConfigs();
	public static final CurrentLimitsConfigs FIRING_SHOOTER_LIMITS = new CurrentLimitsConfigs();

	public static final CurrentLimitsConfigs INTAKE_PIVOT_LIMITS = new CurrentLimitsConfigs();
	public static final CurrentLimitsConfigs INTAKE_WHEEL_LIMITS = new CurrentLimitsConfigs();

	public static final CurrentLimitsConfigs HOOD_LIMITS = new CurrentLimitsConfigs();
	public static final CurrentLimitsConfigs SPINDEXER_LIMITS = new CurrentLimitsConfigs();
	public static final CurrentLimitsConfigs KICKER_LIMITS = new CurrentLimitsConfigs();
	public static final CurrentLimitsConfigs KICKER_LIMITS_2 = new CurrentLimitsConfigs();

	static {
		NORMAL_DRIVE_LIMITS.SupplyCurrentLimit = 85;
		MEDIUM_DRIVE_LIMITS.SupplyCurrentLimit = 60;
		LOW_DRIVE_LIMITS.SupplyCurrentLimit = 40;
		SHOOTING_DRIVE_LIMITS.SupplyCurrentLimit = 30;

		NORMAL_TURN_LIMITS.SupplyCurrentLimit = 40;
		MEDIUM_TURN_LIMITS.SupplyCurrentLimit = 30;
		LOW_TURN_LIMITS.SupplyCurrentLimit = 20;

		IDLE_SHOOTER_LIMITS.SupplyCurrentLimit = 10;
		WINDUP_SHOOTER_LIMITS.SupplyCurrentLimit = 30;
		FIRING_SHOOTER_LIMITS.SupplyCurrentLimit = 90;

		INTAKE_PIVOT_LIMITS.SupplyCurrentLimit = 20;
		INTAKE_WHEEL_LIMITS.SupplyCurrentLimit = 60; //TODO: Might need to decrease

		HOOD_LIMITS.SupplyCurrentLimit = 20;
		SPINDEXER_LIMITS.SupplyCurrentLimit = 20;
		KICKER_LIMITS.SupplyCurrentLimit = 35;
		KICKER_LIMITS_2.SupplyCurrentLimit = 10;
	}
}
