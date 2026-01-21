package frc.robot.subsystems.Intake;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Intake extends Subsystem<IntakeStates> {

	private static Intake instance;
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

	private Intake() {
		super(SUBSYSTEM_NAME, IntakeStates.IN);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new IntakeIOSim();
			case REAL -> new IntakeIOTalonFX();
			case TESTING -> new IntakeIOTalonFX();
		};
	}

	public static Intake getInstance() {
		if (instance == null) {
			instance = new Intake();
		}
		return instance;
	}

	@Override
	protected void runState() {
		io.setLinearPosition(getState().linearPos);
		io.setSpinVoltage(getState().spinSpeed * SET_TO_VOLTS_CF);
		io.updateInputs(inputs);
		Logger.processInputs(SUBSYSTEM_NAME, inputs);

		Logger.recordOutput(SUBSYSTEM_NAME + "/Stator Current", io.getSpinMotor().getStatorCurrent().getValueAsDouble());
	}

	public TalonFX getSpinMotor() {
		return io.getSpinMotor();
	}

	public double getStateTime() {
		return super.getStateTime();
	}
}
