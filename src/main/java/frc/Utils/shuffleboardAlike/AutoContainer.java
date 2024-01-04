// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Utils.shuffleboardAlike;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class AutoContainer {
    private CommandSelector autoSelector;
    private Map<String, Command> autoPaths;

    public AutoContainer(){
        this.autoPaths = new HashMap<String, Command>(); 

        // this.autoPaths.put("FF", swerve.stopModulescCommand().andThen(new FeedForwardCharacterization(
        //     swerve, true,
        //     new FeedForwardCharacterizationData("swerve"),
        //     swerve::runCharacterizationVolts, swerve::getCharacterizationVelocity)));
        this.autoPaths.put("redBumpConeCube", null);


        //this.autoPaths.put("cubeAndDrive", gripper.setShouldGripConeCommand(false).andThen( new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm)).andThen(swerve.driveUntilMeters(1.5, 5, false)));
        //this.autoPaths.put("coneAndDrive", gripper.setShouldGripConeCommand(true).andThen( new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm)).andThen(swerve.driveUntilMeters(1.5, 5, false)));
        // this.autoPaths.put("cubeCharge", gripper.setShouldGripConeCommand(false).andThen(new DriveUntilOtherSide(swerve, true)).andThen(new FastCharge(false, swerve)).andThen(new PIDBalance(swerve)));
        // this.autoPaths.put("coneCharge", gripper.setShouldGripConeCommand(true).andThen(new DriveUntilOtherSide(swerve, true)).andThen(new FastCharge(false, swerve)).andThen(new PIDBalance(swerve)));
            
        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
    }
}
    