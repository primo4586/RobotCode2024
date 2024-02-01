// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Utils.AllianceFlipUtil;
import frc.Utils.PathPlanner.PathPlannerHelper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourNote extends SequentialCommandGroup {
  /** Creates a new FourNote. */
  // bottom to top
  private AutoCommands.CircularRegion spikeIntakeRegion(int i) {
    return new AutoCommands.CircularRegion(
        FieldConstants.StagingLocations.spikeTranslations[i], 2);// TODO: const
  }

  private AutoCommands.CircularRegion centerlineIntakeRegion(int i) {
    return new AutoCommands.CircularRegion(
        FieldConstants.StagingLocations.centerlineTranslations[i], 2);
  }

  PathPlannerHelper pathPlannerHelper = PathPlannerHelper.getInstace();

  public FourNote() {

    Command sequenceIntake = Commands.sequence(
        AutoCommands.intakeWhileInRegion(
            () -> AllianceFlipUtil.apply(spikeIntakeRegion(2))),
        AutoCommands.intakeWhileInRegion(
            () -> AllianceFlipUtil.apply(spikeIntakeRegion(1))),
        AutoCommands.intakeWhileInRegion(
            () -> AllianceFlipUtil.apply(centerlineIntakeRegion(4))));

    var secondShotRegion = new AutoCommands.RectangularRegion(new Translation2d(2.11, 6.37), 0.5, 0.1);
    var thirdShotRegion = new AutoCommands.RectangularRegion(new Translation2d(3.83, 5.99), 0.1, 0.5);
    var fourthShotRegion = new AutoCommands.RectangularRegion(new Translation2d(4.08, 7.18), 0.1, 0.5);
    Command sequenceShots = Commands.sequence(
        AutoCommands.moveWhileShooting(),
        AutoCommands.waitForRegion(() -> AllianceFlipUtil.apply(secondShotRegion)),
        AutoCommands.moveWhileShooting(),
        AutoCommands.waitForRegion(() -> AllianceFlipUtil.apply(thirdShotRegion)),
        AutoCommands.moveWhileShooting(),
        AutoCommands.waitForRegion(() -> AllianceFlipUtil.apply(fourthShotRegion)),
        AutoCommands.moveWhileShooting());
    addCommands(
        pathPlannerHelper.followChoreoPath("4NoteGood"),
        Commands.parallel(sequenceIntake, sequenceShots)
    );
  }
}
