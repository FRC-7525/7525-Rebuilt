package frc.robot.subsystems.Hopper;

import org.team7525.subsystem.Subsystem;
import static frc.robot.GlobalConstants.ROBOT_MODE;

import org.littletonrobotics.junction.Logger;


public class Hopper extends Subsystem<HopperStates>{
    private static Hopper instance;
    private final HopperIO io;
    private final HopperIOInputsAutoLogged inputs;


    private Hopper() {
        super(HopperConstants.SUBSYSTEM_NAME, HopperStates.IDLE);
        inputs = new HopperIOInputsAutoLogged();
        this.io = switch(ROBOT_MODE){
            case REAL -> new HopperIOReal();
            case SIM -> new HopperIOSim();
            case TESTING -> new HopperIOReal();
        };
    }

    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }



    @Override
	protected void runState() {
		io.setTargetVelocity(getState().getVelocity());
		io.updateInput(inputs);

		Logger.processInputs(HopperConstants.SUBSYSTEM_NAME, inputs);
	}

}
