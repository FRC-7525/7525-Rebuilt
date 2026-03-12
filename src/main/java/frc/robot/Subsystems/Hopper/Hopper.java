package frc.robot.Subsystems.Hopper;

import static frc.robot.GlobalConstants.ROBOT_MODE;

import java.util.List;

import frc.robot.Subsystems.Hopper.HopperIO.HopperIOOutputs;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import com.ctre.phoenix6.hardware.TalonFX;

public class Hopper extends Subsystem<HopperStates> {

	private static Hopper instance;
	private final HopperIO io;
	private final HopperIOOutputs outputs;

	private Hopper() {
		super(HopperConstants.SUBSYSTEM_NAME, HopperStates.IDLE);
		outputs = new HopperIOOutputs();
		this.io = switch (ROBOT_MODE) {
			case REAL -> new HopperIOReal();
			case SIM -> new HopperIOReal();
			case TESTING -> new HopperIOReal();
		};
	}

	public static Hopper getInstance() {
		if (instance == null) {
			instance = new Hopper();
		}
		return instance;
	}

	public TalonFX getSpinMotor() {
		return io.getSpinMotor();
	}
	public List<TalonFX> getKickerMotors() {
		return io.getKickerMotors();
	}

	@Override
	protected void runState() {
		io.setTargetSpinVelocity(getState().getSpinVelocity());
		io.setTargetKickerVelocity(getState().getKickVelocity());
		io.updateOutputs(outputs);

		Logger.recordOutput(HopperConstants.SUBSYSTEM_NAME + "/SpinVelocityRPS", outputs.spinVelocityRPS);
		Logger.recordOutput(HopperConstants.SUBSYSTEM_NAME + "/KickVelocityRPS", outputs.kickVelocityRPS);
		Logger.recordOutput(HopperConstants.SUBSYSTEM_NAME + "/TargetSpinVelocity", outputs.targetSpinVelocity);
		Logger.recordOutput(HopperConstants.SUBSYSTEM_NAME + "/TargetKickVelocity", outputs.targetKickVelocity);
		Logger.recordOutput(HopperConstants.SUBSYSTEM_NAME + "/SpinSupplyCurrent", outputs.spindexerCurrent);
		Logger.recordOutput(HopperConstants.SUBSYSTEM_NAME + "/Kicker1SupplyCurrent", outputs.kicker1Current);
		Logger.recordOutput(HopperConstants.SUBSYSTEM_NAME + "/Kicker2SupplyCurrent", outputs.kicker2Current);
	}
}
