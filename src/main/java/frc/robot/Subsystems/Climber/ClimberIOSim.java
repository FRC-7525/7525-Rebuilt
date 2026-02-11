package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSim extends ClimberIOReal {

	protected TalonFXSimState leftMotorSim;
	protected TalonFXSimState rightMotorSim;
	protected ElevatorSim climbSim;
	protected Angle positionSetpoint;

	public ClimberIOSim() {
		rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Aligned));

		leftMotorSim = new TalonFXSimState(leftMotor);
		rightMotorSim = new TalonFXSimState(rightMotor);
		climbSim = new ElevatorSim(
			LinearSystemId.createElevatorSystem(DCMotor.getFalcon500(1), ClimberConstants.CLIMBER_MASS, ClimberConstants.CLIMBER_RADIUS, ClimberConstants.CLIMBER_GEARING),
			DCMotor.getFalcon500(1),
			ClimberConstants.START_HEIGHT,
			ClimberConstants.END_HEIGHT,
			true,
			ClimberConstants.START_HEIGHT
		);

		positionSetpoint = ClimberConstants.IDLE_SETPOINT;
	}

	@Override
	public void logOutputs(ClimberIOOutputs outputs) {
		climbSim.update(GlobalConstants.SIMULATION_PERIOD);

		leftMotorSim.setRawRotorPosition(climbSim.getPositionMeters());
		leftMotorSim.setRotorVelocity(climbSim.getVelocityMetersPerSecond());
		rightMotorSim.setRawRotorPosition(climbSim.getPositionMeters());
		rightMotorSim.setRotorVelocity(climbSim.getVelocityMetersPerSecond());

		outputs.leftPosition = leftMotor.getPosition().getValue();
		outputs.rightPosition = rightMotor.getPosition().getValue();
		outputs.setpoint = positionSetpoint;

		Logger.recordOutput(ClimberConstants.SUBSYSTEM_NAME + "/SimLeftRot", outputs.leftPosition.in(Rotations));
		Logger.recordOutput(ClimberConstants.SUBSYSTEM_NAME + "/SimRightRot", outputs.rightPosition.in(Rotations));
		Logger.recordOutput(ClimberConstants.SUBSYSTEM_NAME + "/SimSetpointRot", outputs.setpoint.in(Rotations));
		Logger.recordOutput(ClimberConstants.SUBSYSTEM_NAME + "/Pose", new Pose3d());
	}

	@Override
	public void setPosition(Angle position) {
		this.positionSetpoint = position;
		climbSim.setInput(pid.calculate(leftMotor.getPosition().getValue().in(Rotations), positionSetpoint.in(Rotations)));
	}

	@Override
	public boolean atPositionSetpoint() {
		double currentRot = leftMotor.getPosition().getValue().in(Rotations);
		return Math.abs(currentRot - positionSetpoint.in(Rotations)) < ClimberConstants.CLIMB_POSITION_TOLERANCE.in(Rotations);
	}
}
