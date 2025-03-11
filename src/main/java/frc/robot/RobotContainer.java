// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import frc.robot.commands.arms.ArmMoveCommands;
import frc.robot.commands.arms.DefaultArmCommand;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.commands.climber.DefaultClimberCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BallScrewSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TraySubsystem;
import frc.robot.utils.AutonManager;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity

    private final BallScrewSubsystem ballScrew = new BallScrewSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem(ballScrew);
    private final TraySubsystem tray = new TraySubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final AutonManager autonManager = new AutonManager();
    private final CommandJoystick driver = new CommandJoystick(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final SwerveSubsystem drivetrain = TunerConstants.createDrivetrain();

     public RobotContainer() {
        addAutonomousChoices();
        autonManager.displayChoices();

        configureBindings();
    }
        
    public Command getAutonomousCommand() {
        return autonManager.getSelected();
    }

    private void addAutonomousChoices() {
        autonManager.addDefaultOption("Middle To Side", AutoCommands.moveMiddleToSide(tray));
        autonManager.addOption("Left", AutoCommands.moveLeft(tray));
        autonManager.addOption("Right", AutoCommands.moveRight(tray));
        
      }
    
    private void configureBindings() {

        var armCommands = new ArmMoveCommands(ballScrew, arm);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(getY())
                    .withVelocityY(getX())
                    .withRotationalRate(getTwist())
                ));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Robot centric driving *** PRESS AND HOLD BUTTON ***
        driver.button(12).whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(getY(), getX()))));

        // Reset the field-centric heading
        driver.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Variable Position Commands
        climber.setDefaultCommand(new DefaultClimberCommand(climber, driver));
        arm.setDefaultCommand(new DefaultArmCommand(arm, operator));
        //ballScrew.setDefaultCommand(new DefaultBallScrewCommand(ballScrew, operator));
        //tray.setDefaultCommand(new DefaultTrayCommand(tray, operator));

        // Ball Screw
        operator.button(XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(ballScrew::moveToDownPosition));
        operator.button(XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(ballScrew::moveToUpPosition));

        // Tray
        operator.povUp().onFalse(new InstantCommand(tray::moveToUpPosition));
        operator.povUp().whileTrue(new InstantCommand(tray::moveToDownPosition));

        /* Move Arms To Zero */
        operator.button(XboxController.Button.kBack.value).onTrue(Commands.sequence(
            new InstantCommand(tray::moveToUpPosition),
            armCommands.moveToZero()
        ));

        /* Move Arms To Home */
        operator.povCenter()
            .and(operator.button(XboxController.Button.kX.value))
            .onTrue(Commands.sequence(
                new InstantCommand(tray::moveToUpPosition),
                armCommands.moveToHome()
            ));

            /* Move Arms To Pickup Position */
        operator.povLeft()
            .and(operator.button(XboxController.Button.kX.value))
            .onTrue(armCommands.pickupCoral());

        /* Move Arms To L2 Position */
        operator.button(XboxController.Button.kA.value).onTrue(armCommands.moveToL2());

        /* Move Arms To L3 Position */
        operator.button(XboxController.Button.kB.value).onTrue(armCommands.moveToL3());

        /* Move Arms To L4 Position */
        operator.povCenter()
            .and(operator.button(XboxController.Button.kY.value))
            .onTrue(armCommands.moveToL4());

        /* Place L4 Position */
        operator.povLeft()
            .and(operator.button(XboxController.Button.kY.value))
            .onTrue(armCommands.placeL4());
        
        /* Move Arms To Climb */
        operator.povDown().onTrue(Commands.sequence(
            Commands.parallel(
                armCommands.moveToClimb(),
                new InstantCommand(tray::moveToDownPosition)
            )
        ));
    }

    private double getX() {
        double xDeadband = 0.1;
        double x = -driver.getX(); // Drive left with negative X (left)
        
        if (Math.abs(x) < xDeadband) {
            return 0;
        } else {
            return (1 / (1 - xDeadband)) * (x + (-Math.signum(x) * xDeadband)) * MaxSpeed;
        }
    }

    private double getY() {
        double xDeadband = 0.1;
        double y = -driver.getY(); // Drive forward with negative Y (forward)
        
        if (Math.abs(y) < xDeadband) {
            return 0;
        } else {
            return (1 / (1 - xDeadband)) * (y + (-Math.signum(y) * xDeadband)) * MaxSpeed;
        }
    }

    private double getTwist() {
        double deadband;
        double twist = -driver.getTwist(); // Drive counterclockwise with negative twist (CCW)
        
        if (Math.signum(twist) < 0) {
            // CCW
            deadband = 0.7; // larger on this side because of joystick sensitivity on CCW rotation
        } else if (Math.signum(twist) > 0) {
            // CW
            deadband = 0.1;
        } else {
            return 0;
        }

        if (Math.abs(twist) < deadband) {
            return 0;
        } else {
            return (1 / (1 - deadband)) * (twist + (-Math.signum(twist) * deadband)) * MaxAngularRate;
        }
    }
}
