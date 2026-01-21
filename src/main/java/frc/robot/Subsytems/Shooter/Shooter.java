package frc.robot.Subsytems.Shooter;

import static frc.robot.Subsytems.Shooter.ShooterConstants.*;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;
import frc.robot.GlobalConstants;

public class Shooter extends Subsystem<ShooterStates>{
    private static Shooter instance;
    private final ShooterIO io;
    private ShooterIOInputsAutoLogged inputs;

    public static Shooter getInstance() {
        if (instance == null) {
            switch (GlobalConstants.ROBOT_MODE) {
                case REAL -> instance = new Shooter(new ShooterIOReal());
                case SIM -> instance = new Shooter(new ShooterIOSim());
                case TESTING -> instance = new Shooter(new ShooterIOReal());
            };
        }
        return instance;
    }

    private Shooter(ShooterIO io) {
        super(SUBSYSTEM_NAME, ShooterStates.OFF);
        this.io = io;
        inputs = new ShooterIOInputsAutoLogged();
    }

    @Override
    public void runState() {
        io.setHoodAngle(getState().getHoodAngle());
        io.setWheelVelocity(getState().getWheelVelocity());
        io.updateInputs(inputs);
        
        Logger.processInputs(SUBSYSTEM_NAME, inputs);
        Logger.recordOutput(SUBSYSTEM_NAME + "/state", getState().getStateString());
        Logger.recordOutput(SUBSYSTEM_NAME + "/ReadyToShoot", readyToShoot());
    }

    public boolean readyToShoot() {
        return io.atHoodAngleSetpoint() && io.atWheelVelocitySetpoint();
    }
}
