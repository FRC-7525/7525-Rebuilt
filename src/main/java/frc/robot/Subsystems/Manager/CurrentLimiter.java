package frc.robot.Subsystems.Manager;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Hopper.Hopper;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import org.team7525.subsystem.Subsystem;

public class CurrentLimiter extends Subsystem<CurrentLimiterStates> {

	private static CurrentLimiter instance;

	// private PowerDistribution currentLimiterPdh = new PowerDistribution(0, PowerDistribution.ModuleType.kRev);
	public CurrentLimiter() {
		super("Current Limits", CurrentLimiterStates.IDLE);
		instance = this;
		var limitConfigs = new CurrentLimitsConfigs();
		limitConfigs.SupplyCurrentLimit = getState().getHoodLimit();
		Shooter.getInstance().getHoodMotor().getConfigurator().apply(limitConfigs);
	}

	public static CurrentLimiter getInstance() {
		if (instance == null) {
			new CurrentLimiter();
		}
		return instance;
	}

	@Override
	protected void runState() {
		//     Logger.recordOutput("BATTERY_DATA/CURRENT", currentLimiterPdh.getTotalCurrent());
		//     Logger.recordOutput("BATTERY_DATA/VOLTAGE", currentLimiterPdh.getVoltage());
		//     Logger.recordOutput("BATTERY_DATA/POWER", currentLimiterPdh.getTotalPower());
	}

	@Override
	protected void stateInit() {
		var limitConfigs = new CurrentLimitsConfigs();
		limitConfigs.SupplyCurrentLimit = getState().getDriveLimit();
		limitConfigs.SupplyCurrentLimitEnable = true;
		Drive.getInstance()
			.getDriveMotors()
			.forEach(motor -> {
				motor.getConfigurator().apply(limitConfigs);
			});
		limitConfigs.SupplyCurrentLimit = getState().getTurnLimit();
		Drive.getInstance()
			.getTurnMotors()
			.forEach(motor -> {
				motor.getConfigurator().apply(limitConfigs);
			});
		limitConfigs.SupplyCurrentLimit = getState().getShooterLimit();
		Shooter.getInstance()
			.getShooterMotors()
			.forEach(motor -> {
				motor.getConfigurator().apply(limitConfigs);
			});
		limitConfigs.SupplyCurrentLimit = getState().getSpindexerLimit();
		Hopper.getInstance().getSpinMotor().getConfigurator().apply(limitConfigs);
		limitConfigs.SupplyCurrentLimit = getState().getKickerLimit();
		Hopper.getInstance().getKickerMotor1().getConfigurator().apply(limitConfigs);
		limitConfigs.SupplyCurrentLimit = getState().getKickerLimit2();
		Hopper.getInstance().getKickerMotor2().getConfigurator().apply(limitConfigs);
		limitConfigs.SupplyCurrentLimit = getState().getIntakeWheelLimit();
		Intake.getInstance().getSpinMotor().getConfigurator().apply(limitConfigs);
		limitConfigs.SupplyCurrentLimit = getState().getPivotLimit();
		Intake.getInstance().getPivotMotor().getConfigurator().apply(limitConfigs);
		System.out.println("Current limits updated: " + getState().toString());
	}
}
