package frc.robot.Subsystems.LEDs;

import static frc.robot.Subsystems.LEDs.LEDsConstants.*;

import org.team7525.subsystem.Subsystem;

import frc.robot.GlobalConstants;
import frc.robot.Robot;
import frc.robot.Subsystems.Drive.Drive;

public class LEDs extends Subsystem<LEDStates> {
    private static LEDs instance;
    private final LEDIO io;

    public static LEDs getInstance() {
        if (instance == null) {
			switch (GlobalConstants.ROBOT_MODE) {
				case REAL -> instance = new LEDs(new LEDIOReal());
				case SIM -> instance = new LEDs(new LEDIOSim());
				case TESTING -> instance = new LEDs(new LEDIOReal());
			}
        }
        return instance;
    }

    private LEDs(LEDIO io) {
        super(SUBSYSTEM_NAME, LEDStates.IDLE);
        this.io = io;
    }
    
    @Override
    public void runState() {
        if (!Robot.isDisabled) {
            io.setManagerPattern(getState().getPattern());
        } else {
            io.setManagerPattern(DISABLED_PATTERN);
        }

        //TODO: lowkey seems like a geeked way of doing this but idk how else i would do it
        switch (Drive.getInstance().getState()) {
            case NORMAL:
                io.setDrivePattern(NORMAL_PATTERN);
				break;
			case AIMLOCK_ALLIANCE_LEFT_SHALLOW:
                io.setDrivePattern(AUTOALIGN_PATTERN);
				break;
			case AIMLOCK_ALLIANCE_LEFT_DEEP:
                io.setDrivePattern(AUTOALIGN_PATTERN);
				break;
			case AIMLOCK_ALLIANCE_RIGHT_DEEP:
                io.setDrivePattern(AUTOALIGN_PATTERN);
				break;
			case AIMLOCK_ALLIANCE_RIGHT_SHALLOW:
                io.setDrivePattern(AUTOALIGN_PATTERN);
				break;
			case AIMLOCK_HUB:
                if (Drive.getInstance().shooterToTargetAngle > LOCKED_IN_DEGREES) io.setDrivePattern(AIMLOCKING_PATTERN);
                else io.setDrivePattern(AIMLOCKED_PATTERN);
				break;
			case AA_NEUTRAL:
                io.setDrivePattern(AUTOALIGN_PATTERN);
				break;
			case AA_TOWER_LEFT:
                io.setDrivePattern(AUTOALIGN_PATTERN);
				break;
			case AA_TOWER_RIGHT:
                io.setDrivePattern(AUTOALIGN_PATTERN);
				break;
			case SNAKE_DRIVE:
                io.setDrivePattern(SNAKE_PATTERN);
				break;
			case AUTO:
				break;
			case LOCKED_X_POSE:
                io.setDrivePattern(X_POSE_PATTERN);
				break;
        }

        io.setLEDData();
    }
}
