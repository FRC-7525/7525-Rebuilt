package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;
import frc.robot.Subsystems.Intake.IntakeConstants.Sim;

public class IntakeIOSim implements IntakeIO {

	private DCMotorSim spinSim;
	private TalonFXSimState spinSimState;
	private TalonFX spinMotor;

	private DCMotorSim linearSim;
	private TalonFXSimState linearSimState;
	private TalonFX linearMotor;
	private final PIDController linearController;

	public IntakeIOSim() {
		spinSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), Sim.MOTOR_MOI, 1), DCMotor.getFalcon500(1));
		spinMotor = new TalonFX(Real.SPIN_MOTOR_ID);
		spinSimState = new TalonFXSimState(spinMotor);

		linearSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), Sim.MOTOR_MOI, LINEAR_GEARING), DCMotor.getFalcon500(1));
		linearMotor = new TalonFX(Real.LINEAR_ACTUATOR_ID);
		linearSimState = new TalonFXSimState(linearMotor);
		linearController = new PIDController(Sim.LINEAR_PID.kP(), Sim.LINEAR_PID.kI(), Sim.LINEAR_PID.kD());
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		spinSim.update(GlobalConstants.SIMULATION_PERIOD);
		spinSimState.setRotorVelocity(spinSim.getAngularVelocityRPM() / 60);
		spinSimState.setRawRotorPosition(spinSim.getAngularPositionRotations());
		spinSimState.setSupplyVoltage(12.0);

		linearSim.update(GlobalConstants.SIMULATION_PERIOD);
		linearSimState.setRotorVelocity((linearSim.getAngularVelocityRPM() / 60) * LINEAR_GEARING);
		linearSimState.setRawRotorPosition(linearSim.getAngularPositionRotations() * LINEAR_GEARING);
		linearSimState.setSupplyVoltage(12.0);

		inputs.spinVelocityRPS = spinMotor.getVelocity().getValue().in(RotationsPerSecond);
		inputs.linearPositionMeters = linearSim.getAngularPositionRotations() * LINEAR_METERS_PER_ROTATION;
	}

	@Override
	public void setSpinVelocity(double rps, double feedforward) {
		spinSim.setInputVoltage(feedforward);
	}

	@Override
	public void setLinearPosition(double meters) {
		linearSim.setInputVoltage(linearController.calculate(linearSim.getAngularPositionRotations(), meters / LINEAR_METERS_PER_ROTATION));
	} //pid only for sim, irl uses talon pid

	@Override
	public TalonFX getSpinMotor() {
		return spinMotor;
	}
}
