package frc.robot.Subsystems.SwerveModule;

import static frc.robot.Constants.SwerveConstants.LOOKAHEAD_S;

import java.util.List;

import com.pathplanner.lib.GeometryUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utilities;
import frc.robot.Constants.SwerveConstants;

public class SwerveMath {
    public static Debouncer debouncer = new Debouncer(0.5, DebounceType.kRising);

    public static Trajectory trajectoryBuilder(Pose2d start, Pose2d end, TrajectoryConfig config, List<Translation2d> waypoints) {
        return TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    } 

    public static double ticksToAngle(double ticks, double gearRatio) {
        double angle = (ticks % SwerveConstants.TICKS_PER_REV_ANALOG_CODER) / gearRatio;

        double result = (angle / (SwerveConstants.TICKS_PER_REV_ANALOG_CODER / 2)) * 180;

        if (result > 180) {
            result -= 360;
        }

        return result;
    }

    public static double absolutePositionToAngle(double absPos) {
        return absPos * 360;
    }

    public static Twist2d log(final Pose2d transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < SwerveConstants.kEps) {
          halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
          halftheta_by_tan_of_halfdtheta =
              -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
        }
        final Translation2d translation_part =
            transform
                .getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
    }

    public static ChassisSpeeds skewCompensation(ChassisSpeeds originalSpeeds){
        Pose2d predictedNextRobotPos = 
            new Pose2d(
                originalSpeeds.vxMetersPerSecond*SwerveConstants.LOOKAHEAD_S,
                originalSpeeds.vyMetersPerSecond*SwerveConstants.LOOKAHEAD_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond*LOOKAHEAD_S));
        Twist2d twist2d = log(predictedNextRobotPos);
        ChassisSpeeds updatedChassisSpeeds = new ChassisSpeeds(
                        twist2d.dx / LOOKAHEAD_S, 
                        twist2d.dy/LOOKAHEAD_S, 
                        twist2d.dtheta/LOOKAHEAD_S);
        return updatedChassisSpeeds;     
    }

    public static double compensateAngularError(double currAngle, double targetAngle) {
        double turnAmount = Math.IEEEremainder(targetAngle - currAngle,180); // only applies if angles are not clamped between 0-360
        // if (Math.abs(turnAmount) >= 180) { // if conditional is valid, we inverse control
        //     // dir = 180 + (-(Math.signum(dir) * 360) + dir); //the inverse of the direction
        //     // (signum)
        //     turnAmount = -(Math.signum(turnAmount) * 360) + turnAmount;
        // }
        return turnAmount;
    }

    public static double[] calculateFastestTurn(double currRadians, double targetRadians, double driveSpeed) {
        double errorInHeading = targetRadians - currRadians;
        double absErrorInHeading = Math.abs(errorInHeading);

        driveSpeed *= (Math.abs(Math.IEEEremainder(errorInHeading+Math.PI, 2 * Math.PI)) > Math.PI / 2 ? -1 : 1);

        if (absErrorInHeading > Math.PI / 2) {
            errorInHeading = absErrorInHeading > (3*Math.PI) / 2 ? -1*(absErrorInHeading -= 2 * Math.PI)
                    : Math.IEEEremainder((Math.PI + errorInHeading), 2 * Math.PI);
        }


        return new double[] { errorInHeading, driveSpeed };
    }

    public static boolean canBeginSkewCompensation(ChassisSpeeds chassisSpeeds) {
        return debouncer.calculate(chassisSpeeds.omegaRadiansPerSecond==0);
    }

    public static double compensateForSkewAngular(double cycleMs, double currHeading, double targetHeading, PIDController compensationPID) {
        double err = Math.IEEEremainder(Math.toRadians(targetHeading), 2*Math.PI) * cycleMs - 
                Math.IEEEremainder(Math.toRadians(currHeading), 2*Math.PI)  * cycleMs;
    
        double compensate = compensationPID.calculate(err);
    
        double new_vel = compensate / cycleMs;
    
        double radius = Math.sqrt(Math.pow(SwerveConstants.TRACK_WIDTH, 2)+Math.pow(SwerveConstants.WHEEL_BASE, 2));
        return Utilities.notWithin(new_vel / radius, -.03, .03) ? new_vel / radius : 0;
    }


    public static double clamp(double angle) {
        if (angle < -180) {
            return angle + 360;
        } else if (angle > 180) {
            return angle - 360;
        }
        return angle;
    }

    public static double clampRadians(double radians){
        if (radians > Math.PI / 2) {
            return Math.abs(radians-Math.PI*2);
        }
        return radians;
    }

    public static Translation2d actualModuleVelocity(Translation2d robotVelocity, Translation2d moduleOffset, double robotAngularVelocity) {
        Translation2d thetaVelocity = moduleOffset.times(robotAngularVelocity);

        Translation2d moduleVelocity = robotVelocity.plus(thetaVelocity);

        return moduleVelocity;
    }
    public static ChassisSpeeds getFieldRelativeChassisSpeeds(ChassisSpeeds robotCentricSpeeds, Rotation2d robotAngle) {
        return new ChassisSpeeds(
            robotCentricSpeeds.vxMetersPerSecond * robotAngle.getCos()
                        - robotCentricSpeeds.vyMetersPerSecond * robotAngle.getSin(),
            robotCentricSpeeds.vyMetersPerSecond * robotAngle.getCos()
                        + robotCentricSpeeds.vxMetersPerSecond * robotAngle.getSin(),
            robotCentricSpeeds.omegaRadiansPerSecond);
    }

    public static ChassisSpeeds getActualRobotSpeeds(SwerveDriveKinematics kinematics, SwerveModuleState ... states) {
        return kinematics.toChassisSpeeds(states);
    }
}
