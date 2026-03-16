package frc.robot.Autonomous;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.ChoreoTrajectories.ChoreoTraj;
import frc.robot.Subsystems.Drive.Drive;

public class AutoRoutines {

	private AutoFactory autoFactory;
	private AutoCommands autoCommands = new AutoCommands();

	public AutoRoutines() {
		autoFactory = new AutoFactory(Drive.getInstance()::getPose, Drive.getInstance()::resetPose, Drive.getInstance()::driveRobotAutonomous, true, Drive.getInstance());

		autoFactory.bind("Idle", autoCommands.returnToIdle());
		autoFactory.bind("Intake", autoCommands.intake());
		autoFactory.bind("Intake And Pass", autoCommands.intakeAndPass());
		autoFactory.bind("Wind To Shuttle", autoCommands.windToShuttle());
		autoFactory.bind("Wind To Score", autoCommands.windToScore());
		autoFactory.bind("Allow Aimlock", autoCommands.allowAimlock());
		autoFactory.bind("Disallow Aimlock", autoCommands.disallowAimlock());
		autoFactory.bind("StopRobot", autoCommands.stopRobot());
		autoFactory.bind("PrintTest", autoCommands.printTest());
	}

	public AutoRoutine Right2CycleRoutine() {
		AutoRoutine routine = autoFactory.newRoutine("Right2CycleRoutine");
		AutoTrajectory part1 = ChoreoTraj.Right2Cycle_P1.asAutoTraj(routine);
		AutoTrajectory part2 = ChoreoTraj.Right2Cycle_P2.asAutoTraj(routine);

		routine.active().onTrue(
			Commands.print("AUTO STARTED")
			.andThen(part1.resetOdometry())
			.andThen(part1.cmd())
			.andThen(new WaitCommand(4))
			.andThen(part2.cmd())
			.andThen(new WaitCommand(4))
		);

		return routine;
	}

	public AutoRoutine Left2CycleRoutine() {
		AutoRoutine routine = autoFactory.newRoutine("Left2CycleRoutine");
		AutoTrajectory part1 = ChoreoTraj.Left2Cycle_P1.asAutoTraj(routine);
		AutoTrajectory part2 = ChoreoTraj.Left2Cycle_P2.asAutoTraj(routine);

		routine.active().onTrue(
			Commands.print("AUTO STARTED")
			.andThen(part1.resetOdometry())
			.andThen(part1.cmd())
			.andThen(new WaitCommand(4))
			.andThen(part2.cmd())
			.andThen(new WaitCommand(4))
		);

		return routine;
	}

	public AutoRoutine sweeperRight1Cycle() {
		AutoRoutine routine = autoFactory.newRoutine("SweeperRight1Cycle");
		AutoTrajectory part1 = ChoreoTraj.SweeperRight1Cycle_P1.asAutoTraj(routine);
		AutoTrajectory part2 = ChoreoTraj.SweeperRight1Cycle_P2.asAutoTraj(routine);

		routine.active().onTrue(
			Commands.print("AUTO STARTED")
			.andThen(part1.resetOdometry())
			.andThen(part1.cmd())
			.andThen(new WaitCommand(4))
			.andThen(part2.cmd())
			.andThen(new WaitCommand(4))
		);

		return routine;
	}

	public AutoRoutine sweeperLeft1Cycle() {
		AutoRoutine routine = autoFactory.newRoutine("SweeperRight1Cycle");
		AutoTrajectory part1 = ChoreoTraj.SweeperLeft1Cycle_P1.asAutoTraj(routine);
		AutoTrajectory part2 = ChoreoTraj.SweeperLeft1Cycle_P2.asAutoTraj(routine);

		routine.active().onTrue(
			Commands.print("AUTO STARTED")
			.andThen(part1.resetOdometry())
			.andThen(part1.cmd())
			.andThen(new WaitCommand(4))
			.andThen(part2.cmd())
			.andThen(new WaitCommand(4))
		);

		return routine;
	}

	public AutoRoutine right1CycleDepot() {
		AutoRoutine routine = autoFactory.newRoutine("Right1CycleDepot");
		AutoTrajectory traj = ChoreoTraj.Right1CycleDepot.asAutoTraj(routine);

		routine.active().onTrue(Commands.print("AUTO STARTED").andThen(traj.resetOdometry()).andThen(traj.cmd()));

		return routine;
	}
	

	public AutoRoutine driveStraight() {
		AutoRoutine routine = autoFactory.newRoutine("Drive1");
		AutoTrajectory traj = ChoreoTraj.Drive1.asAutoTraj(routine);


		routine.active().onTrue(Commands.print("AUTO STARTED").andThen(traj.resetOdometry()).andThen(traj.cmd()));

		return routine;
	}
}
