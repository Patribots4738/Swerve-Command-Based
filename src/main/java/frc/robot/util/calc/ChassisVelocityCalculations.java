package frc.robot.util.calc;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class ChassisVelocityCalculations {
	public static ChassisVelocities fromFieldRelativeSpeeds(
			double vxMetersPerSecond,
			double vyMetersPerSecond,
			double omegaRadiansPerSecond,
			Rotation2d robotAngle) {
		var rotated =
				new Translation2d(vxMetersPerSecond, vyMetersPerSecond).rotateBy(robotAngle.unaryMinus());
		return new ChassisVelocities(rotated.getX(), rotated.getY(), omegaRadiansPerSecond);
	}
	
	public static ChassisVelocities fromRobotRelativeSpeeds(
			ChassisVelocities robotRelativeSpeeds, Rotation2d robotAngle) {
		return fromFieldRelativeSpeeds(
				robotRelativeSpeeds.vx,
				robotRelativeSpeeds.vy,
				robotRelativeSpeeds.omega,
				robotAngle);
	}
}
