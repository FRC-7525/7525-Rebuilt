package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Subsystems.Drive.TunerConstants.TunerSwerveDrivetrain;

public interface DriveIO {
	public class DriveIOOutputs {
		ChassisSpeeds speeds = new ChassisSpeeds();
		SwerveModuleState[] setPoints = new SwerveModuleState[4];
		Rotation3d fullRobotRotation = new Rotation3d();
		double odometryFrequency = 0;
		double timestamp = 0;
		double failedDataAquisitions = 0;
		double robotAngleDeg = 0;
		double gyroAngleDeg = 0;
	}

	public default void logOutputs(DriveIOOutputs outputs) {}

	public default TunerSwerveDrivetrain getDrive() {
		return null;
	}

	public default void zeroGyro() {}

	public default void setControl(SwerveRequest request) {}

	public default void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviaton) {}

	public TalonFX[] getDriveMotors();

	public TalonFX[] getSteerMotors();
}
