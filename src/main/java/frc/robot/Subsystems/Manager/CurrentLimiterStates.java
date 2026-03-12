package frc.robot.Subsystems.Manager;

import org.team7525.subsystem.SubsystemStates;

public enum CurrentLimiterStates implements SubsystemStates {
    IDLE(
        80, 
        40, 
        40, 
        40, 
        40, 
        40, 
        40, 
        40
    ),
	EXTENDED_IDLE(
        80, 
        40, 
        40, 
        40, 
        40, 
        40, 
        40, 
        40
    ),
	INTAKING(
        40,
        20,
        40,
        20,
        20,
        40,
        40,
        40
    ),
	WINDING_UP(
        60,
        30,
        30,
        40,
        40,
        40,
        40,
        40
    ),
	WINDING_UP_FIXED_SHOT(
        60,
        30,
        30,
        40,
        40,
        40,
        40,
        40
    ), 
	SHUTTLING(
        30,
        30,
        60,
        20,
        40,
        20,
        20,
        20
    ),
	SHOOTING_HUB(
        30,
        30,
        60,
        20,
        40,
        20,
        20,
        20
    ),
	SHOOTING_FIXED(
        30,
        30,
        60,
        20,
        40,
        20,
        20,
        20
    ),
	EXTENDING_CLIMBER(
        80,
        40,
        40,
        40,
        40,
        40,
        40,
        40
    ),
	RETRACTING_CLIMBER(
        80,
        40,
        40,
        40,
        40,
        40,
        40,
        40
    );
    private int driveLimit;
    private int turnLimit;
    private int shooterLimit;
    private int pivotLimit;
    private int intakeWheelLimit;
    private int hoodLimit;
    private int spindexerLimit;
    private int kickerLimit;

    private CurrentLimiterStates(int driveLimit, int turnLimit, int shooterLimit, int pivotLimit, int intakeWheelLimit, int hoodLimit, int spindexerLimit, int kickerLimit) {
        this.driveLimit = driveLimit;
        this.turnLimit = turnLimit;
        this.shooterLimit = shooterLimit;
        this.pivotLimit = pivotLimit;
        this.intakeWheelLimit = intakeWheelLimit;
        this.hoodLimit = hoodLimit;
        this.spindexerLimit = spindexerLimit;
        this.kickerLimit = kickerLimit;
    }
    
    public int getDriveLimit() {
        return driveLimit;
    }
    
    public int getTurnLimit() {
        return turnLimit;
    }
    
    public int getShooterLimit() {
        return shooterLimit;
    }
    
    public int getPivotLimit() {
        return pivotLimit;
    }
    
    public int getIntakeWheelLimit() {
        return intakeWheelLimit;
    }
    
    public int getHoodLimit() {
        return hoodLimit;
    }
    
    public int getSpindexerLimit() {
        return spindexerLimit;
    }
    
    public int getKickerLimit() {
        return kickerLimit;
    }
}
