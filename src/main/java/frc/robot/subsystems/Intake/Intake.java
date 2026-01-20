package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Intake extends Subsystem<IntakeStates> {

	private static Intake instance;
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
	private final Debouncer debouncer;

	private Intake() {
		super(SUBSYSTEM_NAME, IntakeStates.IDLE);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new IntakeIOSim();
			case REAL -> new IntakeIOTalonFX();
			case TESTING -> new IntakeIOTalonFX();
		};

		debouncer = new Debouncer(DEBOUNCE_TIME.in(Seconds), DebounceType.kRising);
	}

	public static Intake getInstance() {
		if (instance == null) {
			instance = new Intake();
		}
		return instance;
	}

	@Override
	protected void runState() {
		io.setVelocity(getState().getVelocitySupplier().get());
		io.updateInputs(inputs);
		Logger.processInputs(SUBSYSTEM_NAME, inputs);

		Logger.recordOutput(SUBSYSTEM_NAME + "/Stator Current", io.getMotor().getStatorCurrent().getValueAsDouble());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Has Gamepiece", this.hasGamepiece());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Current Sensed?", this.currentSenseGamepiece());
	}

	public boolean currentSenseGamepiece() {
		return debouncer.calculate(io.currentLimitReached());
	}

	public boolean gamepieceLeft() {
		return io.gamepieceLeft();
	}

	public boolean hasGamepiece() {
		return io.hasGamepiece();
	}

	public TalonFX getMotor() {
		return io.getMotor();
	}

	public double getStateTime() {
		return super.getStateTime();
	}
}