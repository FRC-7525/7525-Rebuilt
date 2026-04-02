package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Rotations;

import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Climber extends Subsystem<ClimberStates> {

	private static Climber instance;
	private final ClimberIO io;
	private ClimberIO.ClimberIOOutputs outputs;

	public static Climber getInstance() {
		if (instance == null) {
			switch (GlobalConstants.ROBOT_MODE) {
				case REAL -> instance = new Climber(new ClimberIOReal());
				case SIM -> instance = new Climber(new ClimberIOSim());
				case TESTING -> instance = new Climber(new ClimberIOReal());
			}
		}
		return instance;
	}

	private Climber(ClimberIO io) {
		super(ClimberConstants.SUBSYSTEM_NAME, ClimberStates.IDLE);
		this.io = io;
		this.outputs = new ClimberIO.ClimberIOOutputs();
	}

	@Override
	public void runState() {
		io.setPosition(getState().getClimberSetpoint());
		io.logOutputs(outputs);

		Logger.recordOutput(getName() + "/AngularPosRot", outputs.angularPos.in(Rotations));
		Logger.recordOutput(getName() + "/SetpointRot", outputs.angularSetpoint.in(Rotations));
		Logger.recordOutput(getName() + "/state", getState().getStateString());
	}

	public boolean readyToClimb() {
		return io.atPositionSetpoint();
	}
}
