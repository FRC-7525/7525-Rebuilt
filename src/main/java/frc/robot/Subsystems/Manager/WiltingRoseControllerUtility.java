package frc.robot.Subsystems.Manager;

import static frc.robot.GlobalConstants.Controllers.*;

class WiltingRoseControllerUtility {

	public static void resetWiltingRoseControllers() {
		// Read each controller press once (this clears the internal "pressed" cache).		DRIVER_CONTROLLER.getAButtonPressed();
		DRIVER_CONTROLLER.getBButtonPressed();
		DRIVER_CONTROLLER.getXButtonPressed();
		DRIVER_CONTROLLER.getYButtonPressed();
		DRIVER_CONTROLLER.getBackButtonPressed();
		DRIVER_CONTROLLER.getStartButtonPressed();
		DRIVER_CONTROLLER.getLeftBumperButtonPressed();
		DRIVER_CONTROLLER.getRightBumperButtonPressed();
		DRIVER_CONTROLLER.getLeftStickButtonPressed();
		DRIVER_CONTROLLER.getRightStickButtonPressed();
		OPERATOR_CONTROLLER.getAButtonPressed();
		OPERATOR_CONTROLLER.getBButtonPressed();
		OPERATOR_CONTROLLER.getXButtonPressed();
		OPERATOR_CONTROLLER.getYButtonPressed();
		OPERATOR_CONTROLLER.getBackButtonPressed();
		OPERATOR_CONTROLLER.getStartButtonPressed();
		OPERATOR_CONTROLLER.getLeftBumperButtonPressed();
		OPERATOR_CONTROLLER.getRightBumperButtonPressed();
		OPERATOR_CONTROLLER.getLeftStickButtonPressed();
		OPERATOR_CONTROLLER.getRightStickButtonPressed();
	}
}
