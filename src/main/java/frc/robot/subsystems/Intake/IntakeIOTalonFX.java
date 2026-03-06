package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Intake.IntakeConstants.Real;

public class IntakeIOTalonFX implements IntakeIO {

	protected final TalonFX spinMotor;
	protected final TalonFX pivotMotor;
	private final PIDController pivotController;
	private double speed;
	private Angle setpoint;

	public IntakeIOTalonFX() {
		pivotController = PIVOT_PID.get();
		spinMotor = new TalonFX(Real.SPIN_MOTOR_ID);
		spinMotor.setNeutralMode(NeutralModeValue.Coast);
		pivotMotor = new TalonFX(Real.PIVOT_MOTOR_ID);
		pivotMotor.setNeutralMode(NeutralModeValue.Brake);
		pivotMotor.setPosition(0);
		speed = 0;
		setpoint = Degrees.of(0);
	}

	@Override
	public void logOutputs(IntakeIOOutputs outputs) {
		outputs.spinVelocity = spinMotor.getVelocity().getValue().in(RotationsPerSecond);
		outputs.spinAppliedVolts = spinMotor.getMotorVoltage().getValue();
		outputs.spinCurrentAmps = spinMotor.getStatorCurrent().getValue();
		outputs.angularPosition = Rotations.of(pivotMotor.getPosition().getValueAsDouble() / GEARING);
		outputs.angularSetpoint = setpoint.div(GEARING);
		SmartDashboard.putData("PIVOT_CONTROLLER", pivotController);
	}

	@Override
	public void setSpinVelocity(double speed) {
		spinMotor.set(speed);
		this.speed = speed;
	}

	@Override
	public void setAngularPosition(Angle setpoint) {
		this.setpoint = setpoint;
		pivotMotor.set(pivotController.calculate(pivotMotor.getPosition().getValueAsDouble() / GEARING, setpoint.in(Rotations) / GEARING));
	}

	@Override
	public TalonFX getSpinMotor() {
		return spinMotor;
	}
}
