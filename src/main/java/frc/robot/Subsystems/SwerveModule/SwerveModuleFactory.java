package frc.robot.Subsystems.SwerveModule;

import edu.wpi.first.math.Pair;
import frc.robot.Constants.SwerveConstants.ModuleNames;

public class SwerveModuleFactory {

    public static SwerveModuleBase createNeoMk4invertedModule(ModuleNames moduleName, boolean isReversedDrive, 
                    boolean isReversedTurn, int driveId, int turnId) {
        SwerveModuleMaxSwerve maxSwerve = new SwerveModuleMaxSwerve();

        maxSwerve.initalize(isReversedDrive, isReversedTurn, driveId, turnId);
        maxSwerve.calibrate(moduleName, ()->0.0);
        maxSwerve.configureSettings();

        return maxSwerve;
    }

    public static SwerveModuleBase createHybridMk1Mode(ModuleNames moduleName, double encoderAbsOffset,
                                                boolean isReversedDrive, boolean isReversedTurn, int driveId, int turnId, int analogId) {
        SwerveModuleHybridMK1 hybridMK1 = new SwerveModuleHybridMK1();
        
        hybridMK1.initalize(isReversedDrive, isReversedTurn, driveId, turnId, analogId, 
            new Pair<Integer,Integer>(null, null));
        hybridMK1.calibrate(moduleName, ()->encoderAbsOffset);
        hybridMK1.configureSettings();
    
        return hybridMK1;
    }
}
