package frc.robot.Subsystems.SwerveModule;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Utilities;
import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;
import frc.robot.Subsystems.Networking.NetworkTableContainer;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveModuleMaxSwerve extends SwerveModuleBase {

    //should have an interface for motor types as well.
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;
    private AnalogEncoder turnEncoder;
    private Encoder driveEncoder;

    @Override
    public double getAbsPosition() {
        return turnEncoder.getAbsolutePosition();
    }

    @Override
    public double getModuleVelocity() {
        return driveMotor.getClosedLoopRampRate();
    }

    @Override
    public double getDistanceTravelled() {
        return driveEncoder.getDistance(); }

    @Override
    public void configureSettings() {
        SwerveDrivetrain.preAssignedModules.add(this);

        Utilities.attemptToConfigureThrow(driveMotor.restoreFactoryDefaults(), "cannot factory reset spark max!");

        //I call this: the config flood
        //seriously though, ugly.
        driveMotor.setSmartCurrentLimit(100, 20);
        driveMotor.setControlFramePeriodMs(100);
        
        double velConvFactor = MAX_DRIVE_RPM * GEAR_RATIO * Units.inchesToMeters(Math.PI * WHEEL_DIAMETER) / 60;
        driveMotor.getEncoder(Type.kHallSensor, 42)
            .setVelocityConversionFactor(velConvFactor);

        turnMotor.setSmartCurrentLimit(30);
        turnMotor.setControlFramePeriodMs(250);
    }

    @Override
    public void easyMotion(double drive, double turn) { 
        driveMotor.getPIDController().setReference(drive, ControlType.kVelocity);
        turnMotor.set(turn);
    }

    public void networkTableDrive() {
        driveMotor.getPIDController().setReference((Double) NetworkTableContainer.entries.get("Auto Forward Power").getNetworkTblValue(), ControlType.kVelocity);
        turnMotor.set(0);
    }

    @Override
    public void initalize(boolean isReversedDrive, 
        boolean isReversedTurn, 
        int driveId, int turnId, int analogEncoderId, Pair<Integer,Integer> channels) {
        driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);
        turnEncoder = new AnalogEncoder(analogEncoderId);
        driveEncoder = new Encoder(channels.getFirst(), channels.getSecond());
        driveMotor.setInverted(isReversedDrive);
        turnMotor.setInverted(isReversedTurn);

        configureSettings();

        driveMotor.burnFlash();
    }
}
