package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Shooter.ShooterConstants.ROBOT_TO_SHOOTER;
import static frc.robot.Subsystems.Shooter.ShooterConstants.STANDBY_ANGLE;
import static frc.robot.Subsystems.Shooter.ShooterConstants.STANDBY_SPEED;
import static frc.robot.Subsystems.Shooter.ShooterConstants.SUBSYSTEM_NAME;
import static frc.robot.Subsystems.Shooter.ShooterConstants.calculateShotInstruction;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.GlobalConstants;
import frc.robot.Robot;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Shooter.ShooterConstants.ShotInstruction;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOOutputs;

public class Shooter extends Subsystem<ShooterStates> {

    private static Shooter instance;
	private final ShooterIO io;
	private ShooterIOOutputs outputs;

	private Pose2d currentTarget = Pose2d.kZero;
	private Pose2d shooterPosition = Pose2d.kZero;
	private Pose2d shooterToTarget = Pose2d.kZero;
	private ShotInstruction shotInstruction = new ShotInstruction(RotationsPerSecond.of(0), Degree.of(0));


	public static Shooter getInstance() {
		if (instance == null) {
			switch (GlobalConstants.ROBOT_MODE) {
				case REAL -> instance = new Shooter(new ShooterIOReal());
				case SIM -> instance = new Shooter(new ShooterIOSim());
				case TESTING -> instance = new Shooter(new ShooterIOReal());
			}
		}
		return instance;
	}

	private Shooter(ShooterIO io) {
		super(SUBSYSTEM_NAME, ShooterStates.IDLE);
		this.io = io;
		outputs = new ShooterIOOutputs();
	}

    
    @Override
    protected void runState() {
		currentTarget = Robot.isRedAlliance ? getState().getTargetPoses().getRedPose(): getState().getTargetPoses().getBluePose();
		shooterPosition = Drive.getInstance().getPose().plus(new Transform2d(ROBOT_TO_SHOOTER.getTranslation().toTranslation2d(), ROBOT_TO_SHOOTER.getRotation().toRotation2d()));
		shooterToTarget = currentTarget.relativeTo(shooterPosition);
		shotInstruction = calculateShotInstruction(Meters.of(shooterToTarget.getTranslation().getDistance(Translation2d.kZero)));

		if (getState() == ShooterStates.IDLE) shotInstruction = new ShotInstruction(RotationsPerSecond.of(0), Degrees.of(0));
		if (getState() == ShooterStates.STANDBY) shotInstruction = new ShotInstruction(STANDBY_SPEED, STANDBY_ANGLE);
		

		io.setHoodAngle(shotInstruction.hoodAngle());
		io.setWheelVelocity(shotInstruction.flywheelSpeed());

        io.logOutputs(outputs);
		Logger.recordOutput(SUBSYSTEM_NAME + "/LeftWheelVelocity", outputs.leftWheelVelocity.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/RightWheelVelocity", outputs.rightWheelVelocity.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/WheelSetpoint", outputs.wheelSetpoint.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/HoodAngle", outputs.hoodAngle.in(Degrees));
		Logger.recordOutput(SUBSYSTEM_NAME + "/HoodSetpoint", outputs.hoodSetpoint.in(Degrees));
		Logger.recordOutput(SUBSYSTEM_NAME + "/state", getState().getStateString());
		Logger.recordOutput(SUBSYSTEM_NAME + "/ReadyToShoot", readyToShoot());
    }

    public boolean readyToShoot() {
		return io.atHoodAngleSetpoint() && io.atWheelVelocitySetpoint();
	}

	public Pose2d getShooterToTarget() {
		return shooterToTarget;
	}
    
}
