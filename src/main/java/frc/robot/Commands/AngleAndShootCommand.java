package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.Manager.ManagerStates;

public class AngleAndShootCommand {
    // needed because shooting is a continuous action that you cannot be driving whilst 
    // (we can investigate SOTM but for now this is regular)
    
    public class Shooting extends Command {
    Robot robot;

    public Shooting(Robot robot) {
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

}
