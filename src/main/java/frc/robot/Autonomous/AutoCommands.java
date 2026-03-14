package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Manager.Manager;
import frc.robot.Subsystems.Manager.ManagerStates;

public class AutoCommands {

	public Command intake() {
		return new InstantCommand(() -> {Manager.getInstance().setState(ManagerStates.INTAKING);});
	}

	public Command intakeAndPass() {
		return new InstantCommand(() -> Manager.getInstance().setState(ManagerStates.INTAKE_AND_PASS_AUTO));
	}

	public Command returnToIdle() {
		return new InstantCommand(() -> {Manager.getInstance().setState(ManagerStates.IDLE);});
	}

	public Command windToScore() {
		return new InstantCommand(() -> Manager.getInstance().setState(ManagerStates.WINDING_TO_SCORE_AUTO));
	}

	public Command windToShuttle() {
		return new InstantCommand(() -> Manager.getInstance().setState(ManagerStates.WINDING_TO_SHUTTLE_AUTO));
	}

	public Command allowAimlock() {
		return new InstantCommand(() -> Drive.getInstance().setAutoAimlock(true));
	}

	public Command disallowAimlock() {
		return new InstantCommand(() -> Drive.getInstance().setAutoAimlock(false));
	}

    public Command stopRobot() {
        return new InstantCommand(() -> Drive.getInstance().executeDriveInstruction(0, 0, 0, false));
    }

	public Command waitXSeconds(double seconds) {
		return new WaitCommand(seconds);
	}

    public Command printTest() {
        return new InstantCommand(() -> System.out.println("HIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII \n HIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII"));
    }

}