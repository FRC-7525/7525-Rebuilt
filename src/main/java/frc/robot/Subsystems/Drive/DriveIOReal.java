package frc.robot.Subsystems.Drive;

import static frc.robot.Subsystems.Drive.TunerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Subsystems.Drive.TunerConstants.TunerSwerveDrivetrain;

public class DriveIOReal implements DriveIO {

	public DriveIOOutputs outputs = new DriveIOOutputs();
	private final TunerSwerveDrivetrain drivetrain = new TunerSwerveDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);

	@Override
	public void logOutputs(DriveIOOutputs outputs) {
		outputs.speeds = drivetrain.getState().Speeds;
		outputs.setPoints = drivetrain.getState().ModuleTargets;
		outputs.odometryFrequency = drivetrain.getOdometryFrequency();
		outputs.timestamp = drivetrain.getState().Timestamp;
		outputs.failedDataAquisitions = drivetrain.getState().FailedDaqs;
		outputs.robotAngleDeg = drivetrain.getState().Pose.getRotation().getDegrees();
		outputs.fullRobotRotation = drivetrain.getRotation3d();
		outputs.gyroAngleDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();
	}

	@Override
	public TunerSwerveDrivetrain getDrive() {
		return drivetrain;
	}

	@Override
	public void zeroGyro() {
		drivetrain.resetRotation(new Rotation2d());
	}

	@Override
	public void setControl(SwerveRequest request) {
		drivetrain.setControl(request);
	}

	@Override
	public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviaton) {
		drivetrain.addVisionMeasurement(pose, timestamp, standardDeviaton);
	}

	@Override
	public TalonFX[] getDriveMotors() {
		TalonFX[] motors = new TalonFX[drivetrain.getModules().length];
		for (int i = 0; i < drivetrain.getModules().length; i++) {
			motors[i] = drivetrain.getModules()[i].getDriveMotor();
		}
		return motors;
	}

	/**
	 * Gets the steer motors for the SwerveDrivetrain.
	 * @return An array of TalonFX objects representing the steer motors.
	 */
	@Override
	public TalonFX[] getSteerMotors() {
		TalonFX[] motors = new TalonFX[drivetrain.getModules().length];
		for (int i = 0; i < drivetrain.getModules().length; i++) {
			motors[i] = drivetrain.getModules()[i].getSteerMotor();
		}
		return motors;
	}
}
