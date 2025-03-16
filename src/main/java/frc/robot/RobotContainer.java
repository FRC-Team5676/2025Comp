// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
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
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 0.75 rotations per second max angular velocity

    private final BallScrewSubsystem ballScrew = new BallScrewSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem(ballScrew);
    private final TraySubsystem tray = new TraySubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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
                drivetrain.applyRequest(() -> driveField
                    .withVelocityX(getY())
                    .withVelocityY(getX())
                    .withRotationalRate(getTwist())
                ));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Limelight driving
        driver.button(1).whileTrue(
                drivetrain.applyRequest(() -> driveRobot
                        .withVelocityX(limelightForward())
                        .withVelocityY(limelightSideToSide())
                        .withRotationalRate(limelightRotation())));

        // Robot centric driving
        driver.button(12).whileTrue(
                drivetrain.applyRequest(() -> driveRobot
                        .withVelocityX(getY())
                        .withVelocityY(getX())
                        .withRotationalRate(getTwist())));

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
        operator.povRight().onTrue(new InstantCommand(tray::moveToCoralPlacePosition));

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
        double deadband = 0.05;
        double value = -driver.getX(); // Drive left with negative X (left)

        double multiplier = 1;
        if (driver.button(11).getAsBoolean()) {
            multiplier = 0.1;
        }

        value = MathUtil.applyDeadband(value, deadband);
        value = Math.signum(value) * Math.pow(value, 2);
        return value * MaxSpeed * multiplier;
    }

    private double getY() {
        double deadband = 0.05;
        double value = -driver.getY(); // Drive forward with negative Y (forward)

        double multiplier = 1;
        if (driver.button(11).getAsBoolean()) {
            multiplier = 0.1;
        }

        value = MathUtil.applyDeadband(value, deadband);
        value = Math.signum(value) * Math.pow(value, 2);
        return value * MaxSpeed * multiplier;
    }

    private double getTwist() {
        double deadband;
        double value = -driver.getTwist(); // Drive counterclockwise with negative twist (CCW)

        double multiplierButton = 1;
        if (driver.button(11).getAsBoolean()) {
            multiplierButton = 0.1;
        }

        // x = Axis 4: 1 bottom to -1 top
        // translate to 1 bottom to 2 top using formula
        // y = -0.5x + 1.5
        double multiplier = -0.5 * driver.getRawAxis(3) + 1.5;

        if (Math.signum(value) <= 0) {
            // CCW
            deadband = 0.5; // larger on this side because of joystick sensitivity on CCW rotation
        } else if (Math.signum(value) > 0) {
            // CW
            deadband = 0.0;
        } else {
            return 0;
        }

        value = MathUtil.applyDeadband(value, deadband);
        value = Math.signum(value) * Math.pow(value, 2);
        return value * MaxAngularRate * multiplier * multiplierButton;
    }

    private double limelightRotation() {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our
        // proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
        // rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.

        double kP = .035;
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // Drive counterclockwise with negative twist (CCW)
        return targetingAngularVelocity * MaxAngularRate;
    }

    private double limelightForward() {
        double kP = 0.1;
        double currentDistance = DistanceToTargetForward();
        double desiredDistance = Units.inchesToMeters(12);

        double distanceError = desiredDistance - currentDistance;
        double forwardSpeed = distanceError * kP;

        // Drive forward with negative Y (forward)
        return forwardSpeed * MaxSpeed;
    }

    private double DistanceToTargetForward() {
        // d = (h2-h1) / tan(a1+a2)
        // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance

        double h1 = Units.inchesToMeters(28); // height of the camera
        double h2 = Units.inchesToMeters(12); // height of the target
        double a1 = Units.degreesToRadians(-23); // y angle of the camera
        double a2 = Units.degreesToRadians(LimelightHelpers.getTY("limelight")); // y angle of the target

        return (h2 - h1) / Math.tan(a1 + a2) - Units.inchesToMeters(22);
    }

    private double limelightSideToSide() {
        double kP = 0.1;
        double currentDistance = DistanceToTargetSideToSide();
        double desiredDistance = Units.inchesToMeters(0);

        double distanceError = desiredDistance - currentDistance;
        double sideSpeed = distanceError * kP;

        // Drive left with negative X (left)
        return sideSpeed * MaxSpeed;
    }

    private double DistanceToTargetSideToSide() {
        // h2 = h1 + d * tan(a1 + a2)
        // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance

        double h1 = Units.inchesToMeters(0); // side offset of the camera
        double d = DistanceToTargetForward(); // distance forward to the target
        double a1 = Units.degreesToRadians(0); // x angle of the camera
        double a2 = Units.degreesToRadians(LimelightHelpers.getTX("limelight")); // x angle of the target

        return h1 + d * Math.tan(a1 + a2);
    }
}
