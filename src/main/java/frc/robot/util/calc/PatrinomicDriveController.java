package frc.robot.util.calc;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class PatrinomicDriveController {
	private final PIDController xController;
	private final PIDController yController;
	private final ProfiledPIDController thetaController;
	private boolean firstRun = true;
	public PatrinomicDriveController(
		PIDController xController,
		PIDController yController,
		ProfiledPIDController thetaController
	) {
		this.xController = xController;
		this.yController = yController;
		this.thetaController = thetaController;
		this.thetaController.enableContinuousInput(0, 2 * Math.PI);
	}

	public ChassisVelocities calculateNextPosition(
		Pose2d currentPose,
		Pose2d trajectoryPose,
		double desiredVelocity,
		Rotation2d desiredRotation
	) {
		if (firstRun) {
			thetaController.reset(currentPose.getRotation().getRadians());
			firstRun = false;
		}

		double xFeedForward = desiredVelocity * trajectoryPose.getRotation().getCos();
		double yFeedForward = desiredVelocity * trajectoryPose.getRotation().getSin();
		double thetaFeedForward = thetaController.calculate(
			currentPose.getRotation().getRadians(),
			desiredRotation.getRadians()
		);

		double xFeedback = xController.calculate(currentPose.getX(), trajectoryPose.getX());
		double yFeedback = yController.calculate(currentPose.getY(), trajectoryPose.getY());

		return ChassisVelocityCalculations.fromFieldRelativeSpeeds(
			xFeedForward + xFeedback,
			yFeedForward + yFeedback,
			thetaFeedForward,
			currentPose.getRotation()
		);
	}

	public PIDController getXController() {
		return xController;
	}

	public PIDController getYController() {
		return yController;
	}

	public ProfiledPIDController getThetaController() {
		return thetaController;
	}
}
