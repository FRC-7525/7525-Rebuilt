package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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
		// For this path needs to setup the correct orientation for this. You can do math based on this formula:
		//Translation2d shooterOffset = new Translation2d(Ox, Oy);

		// // shooter position in field frame
		// Translation2d shooterPos =
		//     robotPose.getTranslation().plus(
		//         shooterOffset.rotateBy(robotPose.getRotation())
		//     );

		// // vector to target
		// double dx = targetX - shooterPos.getX();
		// double dy = targetY - shooterPos.getY();

		// // shooter field angle
		// double targetAngle = Math.atan2(dy, dx);

		// // robot field angle required
		// double robotTargetAngle = targetAngle - shooterRotationOffset;

		// // robot-relative turn
		// double turn = MathUtil.angleModulus(robotTargetAngle - robotPose.getRotation().getRadians());
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
