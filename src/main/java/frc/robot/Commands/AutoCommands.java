package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Subsystems.Manager.ManagerStates;

public class AutoCommands {

	Robot robot;

	public AutoCommands(Robot robot) {
		this.robot = robot;
	}

	public Command intake() {
		return new InstantCommand(() -> robot.getManager().setState(ManagerStates.INTAKING));
	}

	public Command returnToIdle() {
		return new InstantCommand(() -> robot.getManager().setState(ManagerStates.IDLE));
	}

	public Command startWindingUp() {
		return new InstantCommand(() -> robot.getManager().setState(ManagerStates.WINDING_UP));
	}

	public Command windAndIntake() {
		return new InstantCommand(() -> robot.getManager().setState(ManagerStates.WINDING_AND_INTAKING));
	}
}
