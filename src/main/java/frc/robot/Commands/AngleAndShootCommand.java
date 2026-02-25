package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.Manager.ManagerStates;

public class AngleAndShootCommand extends Command {

	// needed because shooting is a continuous action that you cannot be driving whilst
	// (we can investigate SOTM but for now this is regular)
	Robot robot;

	public AngleAndShootCommand(Robot robot) {
		this.robot = robot;
	}

	@Override
	public void initialize() {
		robot.getManager().setState(ManagerStates.SHOOTING_HUB);
	}

	@Override
	public boolean isFinished() {
		return robot.getManager().getState() == ManagerStates.IDLE;
	}
}
