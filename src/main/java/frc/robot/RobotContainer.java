// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.arms.ArmMoveCommands;
import frc.robot.commands.arms.DefaultArmCommand;
import frc.robot.commands.climber.DefaultClimberCommand;
import frc.robot.commands.tray.TrayCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BallScrewSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.RotateAlgaeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TraySubsystem;
import frc.robot.utils.AutonManager;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    private final BallScrewSubsystem ballScrew = new BallScrewSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem(ballScrew);
    private final TraySubsystem tray = new TraySubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final RotateAlgaeSubsystem rotateAlgae = new RotateAlgaeSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.35) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    //private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final AutonManager autonManager = new AutonManager();
    private final CommandJoystick driver = new CommandJoystick(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final SwerveSubsystem drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

     public RobotContainer() {
        addAutonomousChoices();
        autonManager.displayChoices();
    
        configureBindings();
    }
        
    public Command getAutonomousCommand() {
        return autonManager.getSelected();
    }

    private void addAutonomousChoices() {
        autonManager.addDefaultOption("Move Out", new PathPlannerAuto("Move Out"));
        autonManager.addOption("Coral Bottom Right - BOTTOM", new PathPlannerAuto("Coral Bottom Right - BOTTOM"));
        autonManager.addOption("Coral Bottom Right - TOP", new PathPlannerAuto("Coral Bottom Right - TOP"));
        autonManager.addOption("Coral Middle Right - BOTTOM", new PathPlannerAuto("Coral Middle Right - BOTTOM"));
        autonManager.addOption("Coral Middle Right - TOP", new PathPlannerAuto("Coral Middle Right - TOP"));
        autonManager.addOption("Coral Top Right - BOTTOM", new PathPlannerAuto("Coral Top Right - BOTTOM"));
        autonManager.addOption("Coral Top Right - TOP", new PathPlannerAuto("Coral Top Right - TOP"));
        autonManager.addOption("Verify-1", new PathPlannerAuto("Verify-1"));
      }
    
    private void configureBindings() {

        var armCommands = new ArmMoveCommands(ballScrew, arm);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getY() * MaxSpeed) // Drive forward with
                                                                                                 // negative Y (forward)
                        .withVelocityY(-driver.getX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getTwist() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
                ));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Robot centric driving
        driver.button(12).whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getY(), -driver.getX()))));

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
        NamedCommands.registerCommand("L1 coral", TrayCommands.dumpTray(tray));

        // Move Arms
        operator.button(XboxController.Button.kBack.value).onTrue(armCommands.moveToZero());
        operator.povCenter()
            .and(operator.button(XboxController.Button.kX.value))
            .onTrue(armCommands.moveToHome());
        operator.povLeft()
            .and(operator.button(XboxController.Button.kX.value))
            .onTrue(armCommands.pickupCoral());
        operator.button(XboxController.Button.kA.value).onTrue(armCommands.moveToL2());
        operator.button(XboxController.Button.kB.value).onTrue(armCommands.moveToL3());
        operator.povCenter()
            .and(operator.button(XboxController.Button.kY.value))
            .onTrue(armCommands.moveToL4());
        operator.povLeft()
            .and(operator.button(XboxController.Button.kY.value))
            .onTrue(armCommands.placeL4());

        // Algea
        operator.povDown().onTrue(new InstantCommand(rotateAlgae::intakeAlgea));
        operator.povDown().onFalse(new InstantCommand(rotateAlgae::stop));
        driver.button(2).onTrue(new InstantCommand(rotateAlgae::shootAlgea));
        driver.button(2).onFalse(new InstantCommand(rotateAlgae::stop));
    }
}
