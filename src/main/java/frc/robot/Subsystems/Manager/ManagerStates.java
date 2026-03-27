package frc.robot.Subsystems.Manager;

import frc.robot.Subsystems.Climber.ClimberStates;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Hopper.HopperStates;
import frc.robot.Subsystems.Intake.IntakeStates;
import frc.robot.Subsystems.LEDs.LEDStates;
import frc.robot.Subsystems.Shooter.ShooterStates;
import org.team7525.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
	IDLE("IDLE", IntakeStates.IN, HopperStates.IDLE, ShooterStates.IDLE, ClimberStates.IDLE, CurrentLimiterStates.IDLE, LEDStates.IDLE),
	EXTENDED_IDLE("EXTENDED_IDLE", IntakeStates.OUT, HopperStates.IDLE, ShooterStates.IDLE, ClimberStates.IDLE, CurrentLimiterStates.EXTENDED_IDLE, LEDStates.IDLE),
	INTAKING("INTAKING", IntakeStates.INTAKE, HopperStates.IDLE, ShooterStates.STANDBY, ClimberStates.IDLE, CurrentLimiterStates.INTAKING, LEDStates.INTAKING),
	WINDING_UP("WINDING_UP", IntakeStates.OUT, HopperStates.IDLE, ShooterStates.SHOOT_HUB, ClimberStates.IDLE, CurrentLimiterStates.WINDING_UP, LEDStates.WINDING_UP),
	WINDING_UP_FIXED_SHOT("WINDING_UP_FIXED_SHOT", IntakeStates.OUT, HopperStates.IDLE, ShooterStates.SHOOT_FIXED, ClimberStates.IDLE, CurrentLimiterStates.WINDING_UP_FIXED_SHOT, LEDStates.WINDING_UP), //TODO: Switch back to SHOOTING_FIXED shooter state after testing is done
	SHUTTLING("SHUTTLING", IntakeStates.INTAKE, HopperStates.SPINDEXING, ShooterStates.SHOOT_ALLIANCE, ClimberStates.IDLE, CurrentLimiterStates.SHUTTLING, LEDStates.SHUTTLING),
	SHOOTING_HUB("SHOOTING_HUB", IntakeStates.OUT, HopperStates.SPINDEXING, ShooterStates.SHOOT_HUB, ClimberStates.IDLE, CurrentLimiterStates.SHOOTING_HUB, LEDStates.SHOOTING),
	SHOOTING_FIXED("SHOOTING_FIXED", IntakeStates.OUT, HopperStates.SPINDEXING, ShooterStates.SHOOT_FIXED, ClimberStates.IDLE, CurrentLimiterStates.SHOOTING_FIXED, LEDStates.SHOOTING),
	EXTENDING_CLIMBER("EXTENDING_CLIMBER", IntakeStates.IN, HopperStates.IDLE, ShooterStates.IDLE, ClimberStates.EXTEND, CurrentLimiterStates.EXTENDING_CLIMBER, LEDStates.IDLE),
	RETRACTING_CLIMBER("RETRACTING_CLIMBER", IntakeStates.IN, HopperStates.IDLE, ShooterStates.IDLE, ClimberStates.RETRACT, CurrentLimiterStates.RETRACTING_CLIMBER, LEDStates.IDLE),
	ZEROING("ZEROING MECHANISMS", IntakeStates.OUT, HopperStates.IDLE, ShooterStates.ZEROING, ClimberStates.IDLE, CurrentLimiterStates.EXTENDED_IDLE, LEDStates.IDLE),
	// AUTO EXCLUSIVE STATES
	INTAKING_AUTO("AUTO INTAKING", IntakeStates.INTAKE, HopperStates.IDLE, ShooterStates.STANDBY, ClimberStates.IDLE, CurrentLimiterStates.INTAKING, LEDStates.IDLE),
	INTAKE_AND_PASS_AUTO("AUTO INTAKING AND PASSING", IntakeStates.INTAKE, HopperStates.SPINDEXING, ShooterStates.SHOOT_FIXED, ClimberStates.IDLE, CurrentLimiterStates.INTAKING_AND_SHOOTING_AUTO, LEDStates.SHUTTLING),
	WINDING_TO_SCORE_AUTO("AUTO WINDING SCORE", IntakeStates.INTAKE, HopperStates.IDLE, ShooterStates.SHOOT_ALLIANCE, ClimberStates.IDLE, CurrentLimiterStates.WINDING_UP, LEDStates.WINDING_UP),
	WINDING_TO_SHUTTLE_AUTO("AUTO WINDING PASS", IntakeStates.INTAKE, HopperStates.IDLE, ShooterStates.SHOOT_ALLIANCE, ClimberStates.IDLE, CurrentLimiterStates.WINDING_UP, LEDStates.WINDING_UP),
	SCORING_AUTO("AUTO SCORING", IntakeStates.INTAKE, HopperStates.SPINDEXING, ShooterStates.SHOOT_HUB, ClimberStates.IDLE, CurrentLimiterStates.SHOOTING_HUB, LEDStates.SHOOTING);

	private final String stateString;
	private final IntakeStates intakeState;
	private final HopperStates hopperState;
	private final ShooterStates shooterState;
	private final ClimberStates climberState;
	private final CurrentLimiterStates currentLimiterState;
	private final LEDStates ledState;

	ManagerStates(String stateString, IntakeStates intakeState, HopperStates hopperState, ShooterStates shooterState, ClimberStates climberState, CurrentLimiterStates currentLimiterState, LEDStates ledState) {
		this.stateString = stateString;
		this.intakeState = intakeState;
		this.hopperState = hopperState;
		this.shooterState = shooterState;
		this.climberState = climberState;
		this.currentLimiterState = currentLimiterState;
		this.ledState = ledState;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public IntakeStates getIntakeState() {
		return intakeState;
	}

	public HopperStates getHopperState() {
		return hopperState;
	}

	public ShooterStates getShooterState() {
		return shooterState;
	}

	public ClimberStates getClimberState() {
		return climberState;
	}

	public CurrentLimiterStates getCurrentLimiterState() {
		return currentLimiterState;
	}

	public LEDStates getLEDState() {
		return ledState;
	}
}
