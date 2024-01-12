package frc.robot.Subsystems.SwerveModule;

import java.util.Map;
import java.util.function.Supplier;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants.ModuleNames;
import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;
import frc.robot.Subsystems.Networking.NetworkEntry;
import static frc.robot.Constants.SwerveConstants.*;

public abstract class SwerveModuleBase implements ModuleConfiguration {

    protected ModuleNames moduleName;
    protected Supplier<Double> encoderOffset;

    protected NetworkEntry swerveModuleTargetHeading, headingSlider, 
    moduleState, swerveModuleHeading, distanceTravelled, driveVelocity, targetVelocity;

    protected SimpleMotorFeedforward driveFeedForward = DRIVE_FEEDFORWARD;
    protected SimpleMotorFeedforward angularFeedForward = TURN_FEEDFORWARD;

    protected double predictedModuleVelocity, actualModuleVelocity;


    private Supplier<Double> initialVelo, initialAngle;
    private ChassisSpeeds chassisSpeeds;
    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");


    protected void calibrate(ModuleNames moduleName, Supplier<Double> encoderOffset) {
        this.moduleName = moduleName;
        this.encoderOffset = encoderOffset;

        initialVelo = () -> getModuleVelocity();
        initialAngle = () -> getModuleAngle();

        updateDrivePIDs(DRIVE_PID_CONTROLLER, MODULE_HEADING_PID_CONTROLLER, SPARK_PID_TURN_FF);

        SwerveDrivetrain.moduleWheelPos.put(moduleName, getWheelPosition());

        configureShuffleboard();
    }

    protected void configureShuffleboard() {
        headingSlider = new NetworkEntry(tab,
                "slider",
                BuiltInWidgets.kNumberSlider, Map.of("min", 0, "max", 360), 0,
                moduleName.toString());

        swerveModuleTargetHeading = new NetworkEntry(tab,
                "target heading view",
                BuiltInWidgets.kGyro, null, 0, moduleName.toString());
        moduleState = new NetworkEntry(tab,
                "module state",
                BuiltInWidgets.kTextView, null, "", moduleName.toString());

        swerveModuleHeading = new NetworkEntry(tab,
                "current heading view",
                BuiltInWidgets.kGyro, null, initialAngle.get(), moduleName.toString());

        driveVelocity = new NetworkEntry(tab,
                "drive velocity",
                BuiltInWidgets.kTextView, null, initialVelo.get(), moduleName.toString());

        distanceTravelled = new NetworkEntry(tab,
                "distance travelled",
                BuiltInWidgets.kTextView, null, 0, moduleName.toString());
        
        targetVelocity = new NetworkEntry(tab, 
                "target drive velocity", 
                BuiltInWidgets.kTextView, null, 0, moduleName.toString());
        
    };

    protected SwerveModulePosition getWheelPosition() {
        return new SwerveModulePosition(getDistanceTravelled(), Rotation2d.fromDegrees(getModuleAngle()));
    }

    protected double getModuleAngle() {
        double unceiledAngle = SwerveMath.clamp(getAbsPosition() * 360);
        return Math.round(unceiledAngle);
    }

    protected double[] moduleSpeeds() {
        double moduleVelocityUnVectorized = getModuleVelocity();
        double moduleVelX = Math.cos(Math.toRadians(getModuleAngle())) * moduleVelocityUnVectorized;
        double moduleVelY = Math.sin(Math.toRadians(getModuleAngle())) * moduleVelocityUnVectorized;
        double moduleVelTheta = getAngularModuleVelocity();

        return new double[] {moduleVelX, moduleVelY,moduleVelTheta};
    }

    protected double getTargetAng() {
        return swerveModuleTargetHeading.getEntry().getDouble(initialAngle.get());
    }

    protected void setTargetAng(double newTargetAng) {
        swerveModuleTargetHeading.getEntry().setDouble(newTargetAng);
    }

    public void updateRobotSpeeds(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;

        Translation2d predictedModuleVelocity2Plane = 
            SwerveMath.actualModuleVelocity(new 
            Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond),
            offsetMap.get(getModuleName()),
            chassisSpeeds.omegaRadiansPerSecond);
        
        predictedModuleVelocity = Math.sqrt(
                    Math.pow(predictedModuleVelocity2Plane.getX(),2)+
                    Math.pow(predictedModuleVelocity2Plane.getY(), 2));

        SmartDashboard.putNumber(getModuleName().name()+"velocity: ", 
        predictedModuleVelocity);
    }

    public void updateIndividualModuleSpeeds(double[] speeds) {

        Translation2d actualModuleVelocity2Plane =  
            SwerveMath.actualModuleVelocity(new 
            Translation2d(speeds[0], speeds[1]),
            offsetMap.get(getModuleName()),
            speeds[2]);

        actualModuleVelocity = 
                Math.sqrt(
                    Math.pow(actualModuleVelocity2Plane.getX(),2)+
                    Math.pow(actualModuleVelocity2Plane.getY(), 2));

        SmartDashboard.putNumber(getModuleName().name()+"actual velocity: ", 
        actualModuleVelocity);
    }

    public ModuleNames getModuleName() {
        return moduleName;
    }

    public void setDesiredState(SwerveModuleState currState) {
        double target = Math.round(SwerveMath.clamp(currState.angle.getDegrees()));
        setTargetAng(target);

        /**
         * These are telemetry data for shuffleboard; disable them when not troubleshooting to reduce latency
         */
        // swerveModuleHeading.getEntry().setDouble(SwerveMath.clamp(getModuleAngle()));
        // targetVelocity.getEntry().setDouble(currState.speedMetersPerSecond);
        // moduleState.setNetworkEntryValue(currState.toString());

        // //convert velocity to speed
        final double[] constrainedTurning = SwerveMath.calculateFastestTurn(
                    Math.toRadians(getModuleAngle()),
                    Math.toRadians(target), currState.speedMetersPerSecond);
        
        setModule(constrainedTurning[1], constrainedTurning[0]);
    }

    public void updateDrivePIDs(PIDController drivePID, PIDController angularPID, double angularFF) {
        DRIVE_PID_ARRAY = new double[] {drivePID.getP(), drivePID.getI(), drivePID.getD()};
        ANGULAR_PID_ARRAY = new double[] {angularPID.getP(), angularPID.getI(), angularPID.getD()};
        SPARK_PID_TURN_FF = angularFF;
    }

    public void singlePointTo() {
        double currTargetHeading = headingSlider.getEntry().getDouble(0);
        swerveModuleHeading.getEntry().setDouble(getModuleAngle());

        setTargetAng(currTargetHeading);
        SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(currTargetHeading));

        setDesiredState(state);
    }

    public void updateEntries() {
        updateIndividualModuleSpeeds(moduleSpeeds());
        SwerveDrivetrain.moduleWheelPos.replace(moduleName, getWheelPosition());

        driveVelocity.getEntry().setDouble(getModuleVelocity());
        distanceTravelled.getEntry().setDouble(getDistanceTravelled());
        swerveModuleHeading.getEntry().setDouble(getModuleAngle());
    }
}
