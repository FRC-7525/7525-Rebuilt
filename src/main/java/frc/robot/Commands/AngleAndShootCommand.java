package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveStates;
import frc.robot.Subsystems.Manager.Manager;
import frc.robot.Subsystems.Manager.ManagerStates;

public class AngleAndShootCommand extends Command {

	// needed because shooting is a continuous action that you cannot be driving whilst
	// (we can investigate SOTM but for now this is regular)
	private final Timer timer;

	public AngleAndShootCommand() {
		this.timer = new Timer();
	}

	@Override
	public void initialize() {
		Manager.getInstance().setState(ManagerStates.SHOOTING_HUB);
		Drive.getInstance().setState(DriveStates.AIMLOCK_HUB);
		timer.start();
	}

	@Override
	public boolean isFinished() {
		boolean finished = timer.hasElapsed(1.5);
		if (finished) {
			Drive.getInstance().setState(DriveStates.NORMAL);
			Manager.getInstance().setState(ManagerStates.EXTENDED_IDLE);
		}
		return finished;
	}
}
