package frc.robot.Subsystems.Vision;

import static frc.robot.Subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

	private static VisionSystemSim visionSim;

	private final Supplier<Pose2d> poseSupplier;
	private final PhotonCameraSim cameraSim;

	public VisionIOPhotonVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
		super(name, robotToCamera);
		this.poseSupplier = poseSupplier;

		if (visionSim == null) {
			visionSim = new VisionSystemSim("main");
			visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);
		}

		// Add sim camera
		var cameraProperties = new SimCameraProperties();
		// get rid of if weird suff happens
		cameraProperties.setAvgLatencyMs(AVG_LATENCY_MS);
		cameraProperties.setLatencyStdDevMs(LATENCY_STD_DEV_MS);
		cameraProperties.setCalibError(CALIB_ERROR_AVG, CALIB_ERROR_STD_DEV);
		cameraProperties.setFPS(CAMERA_FPS);
		cameraSim = new PhotonCameraSim(camera, cameraProperties);
		visionSim.addCamera(cameraSim, robotToCamera);
	}

	@Override
	public void logOutputs(VisionIOOutputs outputs) {
		visionSim.update(poseSupplier.get());
		super.logOutputs(outputs);
	}
}
