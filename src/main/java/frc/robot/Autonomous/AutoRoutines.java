package frc.robot.Autonomous;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.ChoreoTrajectories.ChoreoTraj;
import frc.robot.Subsystems.Drive.Drive;

public class AutoRoutines {
    
    private AutoFactory autoFactory;
    private AutoCommands autoCommands = new AutoCommands();

    public AutoRoutines() {
        autoFactory = new AutoFactory(
            Drive.getInstance()::getPose,
            Drive.getInstance()::resetPose,
            Drive.getInstance()::driveRobotAutonomous,
            true,
            Drive.getInstance()
        );

        autoFactory.bind("Idle", autoCommands.returnToIdle());
        autoFactory.bind("Intake", autoCommands.intake());
        autoFactory.bind("Intake And Pass", autoCommands.intakeAndPass());
        autoFactory.bind("Wind To Shuttle", autoCommands.windToShuttle());
        autoFactory.bind("Wind To Score", autoCommands.windToScore());
        autoFactory.bind("Allow Aimlock", autoCommands.allowAimlock());
        autoFactory.bind("Disallow Aimlock", autoCommands.disallowAimlock());
        autoFactory.bind("StopRobot", autoCommands.stopRobot());
        autoFactory.bind("PrintTest", autoCommands.printTest());
        autoFactory.bind("Wait 5 Seconds", autoCommands.waitXSeconds(5));
        
    }

    public AutoRoutine Right2CycleRoutine() {
        AutoRoutine routine = autoFactory.newRoutine("Right2CycleRoutine");
        AutoTrajectory traj = ChoreoTraj.Right2Cycle.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.print("AUTO STARTED")
            .andThen(traj.resetOdometry())
            .andThen(traj.cmd())
        );

        return routine;
    }

    public AutoRoutine Left2CycleRoutine() {
        AutoRoutine routine = autoFactory.newRoutine("Left2CycleRoutine");
        AutoTrajectory traj = ChoreoTraj.Left2Cycle.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.print("AUTO STARTED")
            .andThen(traj.resetOdometry())
            .andThen(traj.cmd())
        );

        return routine;
    }

    public AutoRoutine sweeperRight1Cycle() {
        AutoRoutine routine = autoFactory.newRoutine("SweeperRight1Cycle");
        AutoTrajectory traj = ChoreoTraj.SweeperRight1Cycle.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.print("AUTO STARTED")
            .andThen(traj.resetOdometry())
            .andThen(traj.cmd())
        );

        return routine;
    }

    public AutoRoutine sweeperLeft1Cycle() {
        AutoRoutine routine = autoFactory.newRoutine("SweeperLeft1Cycle");
        AutoTrajectory traj = ChoreoTraj.SweeperLeft1Cycle.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.print("AUTO STARTED")
            .andThen(traj.resetOdometry())
            .andThen(traj.cmd())
        );

        return routine;
    }

    public AutoRoutine right1CycleDepot() {
        AutoRoutine routine = autoFactory.newRoutine("Right1CycleDepot");
        AutoTrajectory traj = ChoreoTraj.Right1CycleDepot.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.print("AUTO STARTED")
            .andThen(traj.resetOdometry())
            .andThen(traj.cmd())
        );

        return routine;
    }
}
