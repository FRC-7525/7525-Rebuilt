package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.subsystems.Hopper.HopperConstants.*;

public class HopperIOReal implements HopperIO {
    private final TalonFX spindexerMotor;
    double targetVelocity;

    public HopperIOReal() {
        spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID);
        targetVelocity = 0.0;
    }

    @Override
    public void updateInput(HopperIOInputs inputs) {
        inputs.inputVoltage = spindexerMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorVelocityRPS = spindexerMotor.getVelocity().getValue().in(RotationsPerSecond);
        inputs.targetVelocity = targetVelocity;
    }

    @Override
    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
        spindexerMotor.set(targetVelocity);
    }
}