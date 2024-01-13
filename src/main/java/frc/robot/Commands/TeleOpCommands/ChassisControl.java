// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleOpCommands;

import java.util.Map.Entry;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.SwerveConstants.*;

import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;
import frc.robot.Subsystems.SwerveModule.SwerveMath;

public class ChassisControl extends Command {
  /** Creates a new ChassisControl. */
  private SwerveDrivetrain drivetrain;
  private Supplier<Double> leftXAxis, leftYAxis, leftTrigger, rightTrigger;
  private Supplier<Boolean> fieldCentric;
  private Supplier<Trigger> speedShift;

  private ChassisSpeeds scaledSpeeds;

  private Translation2d pivot;

  private boolean changeSpeedMagnitude = false;
  private double maxSpeed = MAX_SPEED;

  public ChassisControl(SwerveDrivetrain drivetrain,
      Supplier<Double> leftXAxis,
      Supplier<Double> leftYAxis,
      Supplier<Double> leftTrigger,
      Supplier<Double> rightTrigger,
      Supplier<Trigger> speedShift,
      Supplier<Boolean> fieldCentric) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.leftXAxis = leftXAxis;
    this.leftYAxis = leftYAxis;
    this.leftTrigger = leftTrigger;
    this.rightTrigger = rightTrigger;
    this.speedShift = speedShift;
    this.fieldCentric = fieldCentric;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speedShift.get().toggleOnTrue(new InstantCommand(
      () -> changeSpeedMagnitude = true
    ));

    speedShift.get().toggleOnFalse(new InstantCommand(
      () -> changeSpeedMagnitude = false
    ));
    
    maxSpeed = (changeSpeedMagnitude ? MAX_CORRECTION_SPEED : MAX_SPEED);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-leftYAxis.get(), leftXAxis.get(),
          Math.pow(rightTrigger.get() - leftTrigger.get(), 3));
    

    if (fieldCentric.get()) {

      //CHASSIS SPEEDS AFTER NOTHER FOOKING CHASSIS SPEEDS
      //MY GOD THIS NEEDS TO BE CHANGED ONCE I FIND THE SOLUTION

      ChassisSpeeds fieldCentricSpeeds = SwerveMath.getFieldRelativeChassisSpeeds(chassisSpeeds,
          drivetrain.getPigeonRotation2dEM());
      

      //fieldCentricSpeeds are percentage based
      scaledSpeeds = new ChassisSpeeds(fieldCentricSpeeds.vxMetersPerSecond * maxSpeed,
          fieldCentricSpeeds.vyMetersPerSecond * maxSpeed, fieldCentricSpeeds.omegaRadiansPerSecond * maxSpeed*MAX_TURN_SPEED_SCALE);
   
      // SmartDashboard.putNumber("distance travelled X: ", drivetrain.getOdometry().getPoseMeters().getX());
      // SmartDashboard.putNumber("distance travelled Y: ", drivetrain.getOdometry().getPoseMeters().getY());

      drivetrain.updateSkew(SwerveMath.canBeginSkewCompensation(scaledSpeeds));
      
    } else {
        scaledSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond * maxSpeed, 
        chassisSpeeds.vyMetersPerSecond * maxSpeed, chassisSpeeds.omegaRadiansPerSecond * MAX_TURN_SPEED_SCALE);
    }
    
    drivetrain.setCentralMotion(scaledSpeeds, pivot);

    // for (Entry<ModuleNames, SwerveModuleState> state : SwerveDrivetrain.stateMap.entrySet()) {
    //   SmartDashboard.putNumberArray(state.getKey().toString(),
    //       new Double[] { state.getValue().angle.getDegrees(), state.getValue().speedMetersPerSecond });
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.reface(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
