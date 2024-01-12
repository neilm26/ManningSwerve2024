package frc.robot.Subsystems.SwerveModule;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Utilities;
import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;
import frc.robot.Subsystems.Networking.NetworkTableContainer;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveModuleMaxSwerve extends SwerveModuleBase {

    //should have an interface for motor types as well.
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;
    private AbsoluteEncoder turnEncoder;

    private SparkMaxPIDController drivePIDController, angularPIDController;
    private PIDController offSidePIDController = COMPENSATION_ANGULAR_PID_CONTROLLER;
    private ProfiledPIDController offSideVeloPIDController = MODULE_VELOCITY_PID_CONTROLLER;
    private double headingError = 0;
    private double velocityError = 0;

    private RelativeEncoder driveEncoder;

    @Override
    public double getAbsPosition() {
        return Math.abs(turnEncoder.getPosition());
    }

    @Override
    public double getModuleVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getAngularModuleVelocity() {
        return turnEncoder.getVelocity();
    }

    @Override
    public double getDistanceTravelled() {
        return driveEncoder.getPosition(); }

    @Override
    public void configureSettings() {
        SwerveDrivetrain.preAssignedModules.add(this);

        Utilities.attemptToConfigureThrow(driveMotor.restoreFactoryDefaults(), "cannot factory reset spark max!");

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();
        
        driveMotor.setSmartCurrentLimit(40);
        driveMotor.setControlFramePeriodMs(100);

        turnMotor.setSmartCurrentLimit(20);

        driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION);
        driveEncoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION);
        turnEncoder.setPositionConversionFactor(1.005);
        turnEncoder.setVelocityConversionFactor(TURN_ENCODER_VELO_FACTOR);

        drivePIDController = driveMotor.getPIDController();
        angularPIDController  = turnMotor.getPIDController();

        drivePIDController.setFeedbackDevice(driveEncoder);

        angularPIDController.setFeedbackDevice(turnEncoder);

        angularPIDController.setPositionPIDWrappingEnabled(true);
        angularPIDController.setPositionPIDWrappingMinInput(-TURN_ENCODER_POS_FACTOR);
        angularPIDController.setPositionPIDWrappingMaxInput(TURN_ENCODER_POS_FACTOR);

        angularPIDController.setFF(SPARK_PID_TURN_FF);
        drivePIDController.setFF(SPARK_PID_DRIVE_FF);


        turnEncoder.setInverted(TURNING_ENCODER_INVERTED);

        turnMotor.setIdleMode(TURN_MOTOR_IDLE_MODE);
        driveMotor.setIdleMode(DRIVE_MOTOR_IDLE_MODE);

        driveMotor.burnFlash(); turnMotor.burnFlash();
    }

    @Override
    public void setModule(double drive, double turn) {

        //Disable when NOT USING
        
        // angularPIDController.setP(ANGULAR_PID_ARRAY[0]);
        // angularPIDController.setI(ANGULAR_PID_ARRAY[1]);
        // angularPIDController.setD(ANGULAR_PID_ARRAY[2]);

        // angularPIDController.setFF(SPARK_PID_TURN_FF);


        // offSideVeloPIDController.setP(DRIVE_PID_ARRAY[0]);
        // offSideVeloPIDController.setI(DRIVE_PID_ARRAY[1]);
        // offSideVeloPIDController.setD(DRIVE_PID_ARRAY[2]);

        if (Math.abs(turn) < 0.2) {
           headingError = offSidePIDController.calculate(SwerveMath.compensateAngularError(getTargetAng(), getModuleAngle()));
           turnMotor.set(headingError);
        }
        else {
            angularPIDController.setReference(turn, CANSparkMax.ControlType.kPosition);
        }

        drivePIDController.setReference(drive, CANSparkMax.ControlType.kVelocity);

    }

    public void networkTableDrive() {
        driveMotor.getPIDController().setReference((Double) NetworkTableContainer.entries.get("Auto Forward Power").getNetworkTblValue(), ControlType.kVelocity);
        turnMotor.set(0);
    }

    public void initalize(boolean isReversedDrive, 
        boolean isReversedTurn, 
        int driveId, int turnId) {
        driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        driveEncoder = driveMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

        driveMotor.setInverted(isReversedDrive);
        turnMotor.setInverted(isReversedTurn);

        configureSettings();
    }
}
