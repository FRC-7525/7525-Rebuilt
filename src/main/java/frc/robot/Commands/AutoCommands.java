package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Manager.Manager;
import frc.robot.Subsystems.Manager.ManagerStates;

public class AutoCommands {

	Robot robot;

	public AutoCommands(Robot robot) {
		this.robot = robot;
	}

	public Command intake() {
		return new InstantCommand(() -> {Manager.getInstance().setState(ManagerStates.INTAKING); System.out.println("hi");});
	}

	public Command returnToIdle() {
		return new InstantCommand(() -> {Manager.getInstance().setState(ManagerStates.IDLE); System.out.println("hi");});
	}

	public Command startWindingUp() {
		return new InstantCommand(() -> Manager.getInstance().setState(ManagerStates.WINDING_UP));
	}

	public Command windAndIntake() {
		return new InstantCommand(() -> {Manager.getInstance().setState(ManagerStates.WINDING_AND_INTAKING); System.out.println("hi \n hi \n hi \n hi");});
	}

	public Command test() {
		return new InstantCommand(() -> {System.out.println("hiIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIiIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIiIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIiIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII");});
	}

	public Command allowAimlock() {
		return new InstantCommand(() -> Drive.getInstance().setAutoAimlocking(true));
	}

	public Command disallowAimlock() {
		return new InstantCommand(() -> Drive.getInstance().setAutoAimlocking(false));
	}
}
