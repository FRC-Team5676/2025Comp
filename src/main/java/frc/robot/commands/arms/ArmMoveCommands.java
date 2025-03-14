package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BallScrewSubsystem;

public class ArmMoveCommands extends Command {

    private final BallScrewSubsystem m_BallScrew;
    private final ArmSubsystem m_Arm;

    public ArmMoveCommands(BallScrewSubsystem ballScrew,
            ArmSubsystem arm) {

        m_BallScrew = ballScrew;
        m_Arm = arm;

        addRequirements(ballScrew, arm);
    }

    public Command moveToClimb() {
        return Commands.sequence(
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveRotateArm(-100)),
                new InstantCommand(() -> m_Arm.moveLinearArmRetracted()),
                new InstantCommand(() -> m_Arm.moveWrist(-90))
            ),
            Commands.waitSeconds(0.5),
            new InstantCommand(() -> m_BallScrew.moveToDownPosition())
        );
    }

    public Command moveToZero() {
        return Commands.sequence(
            new InstantCommand(() -> m_Arm.moveWrist(0)),
            new InstantCommand(() -> m_Arm.moveLinearArmRetracted()),
            Commands.waitSeconds(1),
            new InstantCommand(() -> m_Arm.moveRotateArm(0)),
            Commands.waitSeconds(0.5),
            new InstantCommand(() -> m_BallScrew.moveToDownPosition())
        );
    }

    public Command moveToHome() {
        return Commands.sequence(
            new InstantCommand(() -> m_Arm.moveRotateArm(-32)),
            new InstantCommand(() -> m_Arm.moveLinearArmPickup()),
            Commands.waitSeconds(0.5),
            new InstantCommand(() -> m_Arm.moveWrist(90)),
            Commands.waitSeconds(0.5),
            new InstantCommand(() -> m_BallScrew.moveToDownPosition())
        );
    }

    public Command pickupCoral() {
        return Commands.sequence(
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveRotateArm(3)),
                new InstantCommand(() -> m_Arm.moveWrist(120))
            ),
            Commands.waitSeconds(0.75),
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveRotateArm(-32)),
                new InstantCommand(() -> m_Arm.moveWrist(90))
            )
        );
}

    public Command moveToL2() {
        return Commands.sequence(
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveRotateArm(-197)),
                new InstantCommand(() -> m_Arm.moveLinearArmL2()),
                new InstantCommand(() -> m_Arm.moveWrist(-66))
            )
        );
    }

    public Command moveToL3() {
        return Commands.sequence(
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveRotateArm(-109)),
                new InstantCommand(() -> m_Arm.moveLinearArmL3()),
                new InstantCommand(() -> m_Arm.moveWrist(10))
            )
        );
    }

    public Command moveToL4() {
        return Commands.sequence(
            new InstantCommand(() -> m_BallScrew.moveToUpPosition()),
            Commands.waitSeconds(1),
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveRotateArm(-161)),
                new InstantCommand(() -> m_Arm.moveLinearArmL4()),
                new InstantCommand(() -> m_Arm.moveWrist(-140))
            )
        );
    }

    public Command placeL4() {
        return Commands.sequence(
            new InstantCommand(() -> m_Arm.moveWrist(-115)),
            Commands.waitSeconds(0.25),
            new InstantCommand(() -> m_Arm.moveLinearArmRetracted()),
            Commands.waitSeconds(1),
            new InstantCommand(() -> m_Arm.moveRotateArm(-140))
        );
    }

}
