package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TraySubsystem;

public class AutoCommands extends Command {

    public static Command moveLeft(TraySubsystem tray) {
        return Commands.sequence(
            new InstantCommand(() -> tray.moveToUpPosition()),
            new PathPlannerAuto("Left"),
            Commands.waitSeconds(3),
            new InstantCommand(() -> tray.moveToDownPosition()),
            Commands.waitSeconds(1),
            new InstantCommand(() -> tray.moveToUpPosition())
        );
    }

    public static Command moveRight(TraySubsystem tray) {
        return Commands.sequence(
            new InstantCommand(() -> tray.moveToUpPosition()),
            new PathPlannerAuto("Right"),
            Commands.waitSeconds(3),
            new InstantCommand(() -> tray.moveToDownPosition()),
            Commands.waitSeconds(1),
            new InstantCommand(() -> tray.moveToUpPosition())
        );
    }

    public static Command moveMiddleToSide(TraySubsystem tray) {
        return Commands.sequence(
            new InstantCommand(() -> tray.moveToUpPosition()),
            Commands.waitSeconds(9),
            new PathPlannerAuto("Middle To Side"),
            Commands.waitSeconds(1),
            new InstantCommand(() -> tray.moveToDownPosition()),
            Commands.waitSeconds(1),
            new InstantCommand(() -> tray.moveToUpPosition())
        );
    }
}
