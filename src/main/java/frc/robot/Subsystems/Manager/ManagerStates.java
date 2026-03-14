package frc.robot.Subsystems.Manager;

import frc.robot.Subsystems.Climber.ClimberStates;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Hopper.HopperStates;
import frc.robot.Subsystems.Intake.IntakeStates;
import frc.robot.Subsystems.Shooter.ShooterStates;
import org.team7525.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
	IDLE("IDLE", IntakeStates.IN, HopperStates.IDLE, ShooterStates.IDLE, ClimberStates.IDLE),
	EXTENDED_IDLE("EXTENDED_IDLE", IntakeStates.OUT, HopperStates.IDLE, ShooterStates.IDLE, ClimberStates.IDLE),
	INTAKING("INTAKING", IntakeStates.INTAKE, HopperStates.SPINDEXING, ShooterStates.STANDBY, ClimberStates.IDLE),
	WINDING_UP("WINDING_UP", IntakeStates.IN, HopperStates.IDLE, null, ClimberStates.IDLE),
	WINDING_UP_FIXED_SHOT("WINDING_UP_FIXED_SHOT", IntakeStates.IN, HopperStates.IDLE, ShooterStates.SHOOT_FIXED, ClimberStates.IDLE),
	SHUTTLING("SHUTTLING", IntakeStates.IN, HopperStates.SPINDEXING, ShooterStates.SHOOT_ALLIANCE, ClimberStates.IDLE),
	SHOOTING_HUB("SHOOTING_HUB", IntakeStates.IN, HopperStates.SPINDEXING, ShooterStates.SHOOT_HUB, ClimberStates.IDLE),
	SHOOTING_FIXED("SHOOTING_FIXED", IntakeStates.IN, HopperStates.SPINDEXING, ShooterStates.SHOOT_FIXED, ClimberStates.IDLE),
	EXTENDING_CLIMBER("EXTENDING_CLIMBER", IntakeStates.IN, HopperStates.IDLE, ShooterStates.IDLE, ClimberStates.EXTEND),
	RETRACTING_CLIMBER("RETRACTING_CLIMBER", IntakeStates.IN, HopperStates.IDLE, ShooterStates.IDLE, ClimberStates.RETRACT),
	// AUTO EXCLUSIVE STATES
	INTAKING_AUTO("AUTO INTAKING", IntakeStates.INTAKE, HopperStates.IDLE, ShooterStates.STANDBY, ClimberStates.IDLE),
	INTAKE_AND_PASS_AUTO("AUTO INTAKING AND PASSING", IntakeStates.INTAKE, HopperStates.SPINDEXING, ShooterStates.SHOOT_FIXED, ClimberStates.IDLE),
	WINDING_TO_SCORE_AUTO("AUTO WINDING SCORE", IntakeStates.INTAKE, HopperStates.IDLE, ShooterStates.SHOOT_ALLIANCE, ClimberStates.IDLE),
	WINDING_TO_SHUTTLE_AUTO("AUTO WINDING PASS", IntakeStates.INTAKE, HopperStates.IDLE, ShooterStates.SHOOT_ALLIANCE, ClimberStates.IDLE),
	SCORING_AUTO("AUTO SCORING", IntakeStates.INTAKE, HopperStates.SPINDEXING, ShooterStates.SHOOT_ALLIANCE, ClimberStates.IDLE);
	

	private final String stateString;
	private final IntakeStates intakeState;
	private final HopperStates hopperState;
	private final ShooterStates shooterState;
	private final ClimberStates climberState;

	ManagerStates(String stateString, IntakeStates intakeState, HopperStates hopperState, ShooterStates shooterState, ClimberStates climberState) {
		this.stateString = stateString;
		this.intakeState = intakeState;
		this.hopperState = hopperState;
		this.shooterState = shooterState;
		this.climberState = climberState;
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
		// Could be in shooter but would lead to wierd states i think
		// shhoter doesn't change between winding up and shooting only hopper does
		if (this == WINDING_UP && shooterState == null) {
			return Drive.getInstance().isAtAllianceShootingPosition() ? ShooterStates.SHOOT_ALLIANCE : ShooterStates.SHOOT_HUB;
		}
		return shooterState;
	}

	public ClimberStates getClimberState() {
		return climberState;
	}
}
