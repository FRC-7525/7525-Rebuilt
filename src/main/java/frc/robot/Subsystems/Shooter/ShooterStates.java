package frc.robot.Subsystems.Shooter;

import static frc.robot.Subsystems.Shooter.ShooterConstants.HUB_POSES;
import static frc.robot.Subsystems.Shooter.ShooterConstants.LEFT_DEEP_POSES;
import static frc.robot.Subsystems.Shooter.ShooterConstants.LEFT_SHALLOW_POSES;
import static frc.robot.Subsystems.Shooter.ShooterConstants.RIGHT_DEEP_POSES;
import static frc.robot.Subsystems.Shooter.ShooterConstants.RIGHT_SHALLOW_POSES;

import org.team7525.subsystem.SubsystemStates;

import frc.robot.Subsystems.Drive.AutoAlign.PosePair;

public enum ShooterStates implements SubsystemStates{
    IDLE("Idle", PosePair.kZero),
    STANDBY("Standing By", PosePair.kZero),
    SHOOT_FIXED("Shooting Fixed Shot", PosePair.kZero),
    SHOOT_HUB("Shooting Hub", HUB_POSES),
    SHOOT_ALLIANCE_LEFT_DEEP("Shooting Leftg Side Deep", LEFT_DEEP_POSES),
    SHOOT_ALLIANCE_LEFT_SHALLOW("Shooting Left Side Shallow", LEFT_SHALLOW_POSES),
    SHOOT_ALLIANCE_RIGHT_DEEP("Shooting Right Side Deep", RIGHT_DEEP_POSES),
    SHOOT_ALLIANCE_RIGHT_SHALLOW("Shooting Right Side Shallow", RIGHT_SHALLOW_POSES);

    private String stateString;
    private PosePair targetPoses;

    ShooterStates(String stateString, PosePair targetPoses) {
        this.stateString = stateString;
        this.targetPoses = targetPoses;
    }

    @Override
    public String getStateString() {
        return stateString;
    }

    public PosePair getTargetPoses() {
        return targetPoses;
    }
}
