package frc.robot.Subsystems.SwerveModule;

import java.util.Map;
import java.util.function.Supplier;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
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

    private Supplier<Double> initialVelo, initialAngle;
    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    protected void calibrate(ModuleNames moduleName, Supplier<Double> encoderOffset) {
        this.moduleName = moduleName;
        this.encoderOffset = encoderOffset;

        initialVelo = () -> getModuleVelocity();
        initialAngle = () -> getModuleAngle();

        updateDrivePIDs(DRIVE_PID_CONTROLLER, MODULE_HEADING_PID_CONTROLLER);

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
        double unceiledAngle = getAbsPosition() * 360;
        return Math.round(unceiledAngle);
    }

    protected double getTargetAng() {
        return swerveModuleTargetHeading.getEntry().getDouble(initialAngle.get());
    }

    protected void setTargetAng(double newTargetAng) {
        swerveModuleTargetHeading.getEntry().setDouble(newTargetAng);
    }

    public ModuleNames getModuleName() {
        return moduleName;
    }

    public void setDesiredState(SwerveModuleState currState) {
        double target = SwerveMath.clamp(currState.angle.getDegrees());
        setTargetAng(target);

        swerveModuleHeading.getEntry().setDouble(getModuleAngle());
        targetVelocity.getEntry().setDouble(currState.speedMetersPerSecond);
        moduleState.setNetworkEntryValue(currState.toString());

        // //convert velocity to speed
        final double[] constrainedTurning = SwerveMath.calculateFastestTurn(
                    Math.toRadians(getModuleAngle()),
                    Math.toRadians(target), currState.speedMetersPerSecond);
        
        setModule(constrainedTurning[1], constrainedTurning[0]);
    }

    public void updateDrivePIDs(PIDController drivePID, ProfiledPIDController angularPID) {
        DRIVE_PID_ARRAY = new double[] {drivePID.getP(), drivePID.getI(), drivePID.getD()};
        ANGULAR_PID_ARRAY = new double[] {angularPID.getP(), angularPID.getI(), angularPID.getD()};
    }

    public void singlePointTo() {
        double currTargetHeading = headingSlider.getEntry().getDouble(0);
        swerveModuleHeading.getEntry().setDouble(getModuleAngle());

        setTargetAng(currTargetHeading);
        SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(currTargetHeading));

        setDesiredState(state);
    }

    public void updateEntries() {
        SwerveDrivetrain.moduleWheelPos.replace(moduleName, getWheelPosition());

        driveVelocity.getEntry().setDouble(getModuleVelocity());
        distanceTravelled.getEntry().setDouble(getDistanceTravelled());
        swerveModuleHeading.getEntry().setDouble(getModuleAngle());
    }
}
