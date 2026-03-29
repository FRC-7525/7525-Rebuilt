package frc.robot.Subsystems.Vision;

import java.util.Set;

import org.team7525.subsystem.SubsystemStates;

public enum VisionStates implements SubsystemStates {
    NORMAL("Normal Vision", Set.of()),
    IGNORE_BL("Ignoring Back Left Camera",Set.of(0)),
    IGNORE_BR("Ignoring Back Right Camera", Set.of(1)),
    IGNORE_BL_BR("Ignoring Back Left and Back Right Cameras", Set.of(0, 1));

    String stateString;
    VisionStates(String stateString, Set<Integer> ignoreID ) {
        
    }
}
