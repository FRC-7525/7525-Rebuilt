package frc.robot.subsystems.Intake;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.subsystems.Intake.IntakeConstants.*;
import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Intake extends Subsystem<IntakeStates> {

	private static Intake instance;
	private final IntakeIO io;
	private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
	private final SimpleMotorFeedforward spinFF = new SimpleMotorFeedforward(SPIN_kS, SPIN_kV, SPIN_kA);

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

		//asking for units explicitly
		double linearPos = getState().linearPos.in(Meters);
		double spinSpeed = getState().spinSpeed.in(RotationsPerSecond);

		
		io.setLinearPosition(linearPos);
		io.setSpinVelocity(spinSpeed, spinFF.calculate(spinSpeed));
		io.updateInputs(inputs);
		
		Logger.recordOutput(SUBSYSTEM_NAME + "/SpinVelocityRPS", inputs.spinVelocityRPS);
		Logger.recordOutput(SUBSYSTEM_NAME + "/SpinAppliedVolts", inputs.spinAppliedVolts);
		Logger.recordOutput(SUBSYSTEM_NAME + "/SpinCurrentAmps", inputs.spinCurrentAmps);
		Logger.recordOutput(SUBSYSTEM_NAME + "/LinearPositionMeters", inputs.linearPositionMeters);
		Logger.recordOutput(SUBSYSTEM_NAME + "/LinearVelocityMetersPerSec", inputs.linearVelocityMetersPerSec);
		Logger.recordOutput(SUBSYSTEM_NAME + "/LinearAppliedVolts", inputs.linearAppliedVolts);
		Logger.recordOutput(SUBSYSTEM_NAME + "/LinearCurrentAmps", inputs.linearCurrentAmps);
	}

	public TalonFX getSpinMotor() {
		return io.getSpinMotor();
	}

	public double getStateTime() {
		return super.getStateTime();
	}
}