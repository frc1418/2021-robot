/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                                                */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANEncoder;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
<<<<<<< HEAD
import frc.robot.commands.AutomaticShootCommand;
=======
import frc.robot.commands.AlignCommand;
>>>>>>> Add Align Button Functionality
import frc.robot.commands.ChargeAutoCommand;
import frc.robot.common.ControlPanelColor;
import frc.robot.common.ControlPanelColorSensor;
import frc.robot.common.Limelight;
import frc.robot.common.Odometry;
import frc.robot.common.TrajectoryLoader;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.common.LEDDriver;

import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.logging.Logger;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final Logger logger = Logger.getLogger("Robot");
    private final Limelight limelight = new Limelight();

    // DRIVE
    private final CANSparkMax frontLeftMotor = new CANSparkMax(FRONT_LEFT_MOTOR,
        MotorType.kBrushless);
    private final CANSparkMax frontRightMotor = new CANSparkMax(FRONT_RIGHT_MOTOR,
        MotorType.kBrushless);
    private final CANSparkMax rearLeftMotor = new CANSparkMax(REAR_LEFT_MOTOR,
        MotorType.kBrushless);
    private final CANSparkMax rearRightMotor = new CANSparkMax(REAR_RIGHT_MOTOR,
        MotorType.kBrushless);

    //Odometry
    private final Gyro gyro = new AHRS(SPI.Port.kMXP);

    private final CANEncoder leftEncoder = frontLeftMotor.getEncoder();
    private final CANEncoder rightEncoder = frontRightMotor.getEncoder();

    private final DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    private final Odometry odometry = new Odometry(gyro, driveOdometry, leftEncoder, rightEncoder);

    // DRIVE SUBSYSTEM
    private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeftMotor, rearLeftMotor);
    private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRightMotor, rearRightMotor);

    private final DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(driveTrain, leftMotors, rightMotors, odometry);
    

    // SHOOTER SUBSYSTEM
    private final CANSparkMax shooterMotor1 = new CANSparkMax(SHOOTER_MOTOR_1, MotorType.kBrushed);
    private final CANEncoder shooterEncoder = shooterMotor1.getEncoder(EncoderType.kQuadrature, 8192);
    private final CANSparkMax shooterMotor2 = new CANSparkMax(Constants.SHOOTER_MOTOR_2,
        MotorType.kBrushed);
    private final Solenoid shooterSolenoid = new Solenoid(SHOOTER_SOLENOID_PORT);
    private final Ultrasonic ballSensor = new Ultrasonic(SHOOTER_ULTRASONIC_TRIG,
        SHOOTER_ULTRASONIC_ECHO, Unit.kInches);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(shooterMotor1,
        shooterMotor2, shooterSolenoid, shooterEncoder, ballSensor);
    private final double TARGET_VELOCITY = 4750;
    private final int NUM_BALLS_LOADED = 3;

    // INTAKE SUBSYSTEM
    private final WPI_VictorSPX upperIntakeMotor = new WPI_VictorSPX(UPPER_INTAKE_MOTOR);
    private final CANSparkMax lowerIntakeMotor = new CANSparkMax(BOTTOM_INTAKE_MOTOR,
        MotorType.kBrushed);
    private final DigitalInput intakeSwitch = new DigitalInput(INTAKE_SWITCH);
    private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(INTAKE_SOLENOID_FWD,
        INTAKE_SOLENOID_REV);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(upperIntakeMotor,
        lowerIntakeMotor, intakeSwitch, intakeSolenoid);

    // CONTROL PANEL SUBSYSTEM
    private final ColorMatch colorMatcher = new ColorMatch();
    private final ControlPanelColorSensor colorSensor = new ControlPanelColorSensor(colorMatcher,
        new ColorSensorV3(Port.kOnboard));
    private final DoubleSolenoid cpSolenoid = new DoubleSolenoid(CONTROL_PANEL_SOLENOID_FWD,
        CONTROL_PANEL_SOLENOID_REV);
    private final WPI_VictorSPX cpMotor = new WPI_VictorSPX(CONTROL_PANEL_MOTOR);
    private final ControlPanelSubsystem controlPanelSubsystem = new ControlPanelSubsystem(
        cpSolenoid, cpMotor, colorSensor, DriverStation.getInstance());


    // TRAJECTORIES
    private final TrajectoryLoader trajectoryLoader = new TrajectoryLoader();

    // LED
    private final LEDDriver ledDriver = new LEDDriver(1);

    // NAVX
    private final AHRS navx = new AHRS(SPI.Port.kMXP);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        //Configure the button bindings
        configureButtonBindings();
        configureObjects();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // Joysticks
        Joystick leftJoystick = new Joystick(0);
        Joystick rightJoystick = new Joystick(1);
        Joystick altJoystick = new Joystick(2);

        JoystickButton btnLauncherSolenoid = new JoystickButton(altJoystick, 1);
        JoystickButton btnAlign = new JoystickButton(leftJoystick, 1);
        JoystickButton btnIntakeIn = new JoystickButton(altJoystick, 3);
        JoystickButton btnIntakeOut = new JoystickButton(altJoystick, 4);
        JoystickButton btnCPExtend = new JoystickButton(leftJoystick,
            4); // toggle: use "toggleWhenPressed" method to set command
        JoystickButton btnWinch = new JoystickButton(altJoystick, 8);
        JoystickButton btnCPMotor = new JoystickButton(leftJoystick, 3);
        JoystickButton btnLauncherMotor = new JoystickButton(altJoystick, 12);
        JoystickButton btnLauncherIdle = new JoystickButton(altJoystick,
            10); // toggle: use "toggleWhenPressed" method to set command
        JoystickButton btnLauncherMotorClose = new JoystickButton(altJoystick, 11);
        JoystickButton btnLauncherMotorDynamic = new JoystickButton(altJoystick, 9);
        JoystickButton btnSlowMovement = new JoystickButton(rightJoystick, 1);
        JoystickButton btnIntakeSolenoid = new JoystickButton(altJoystick,
            2); // toggle: use "toggleWhenPressed" method to set command
        JoystickButton btnScissorExtend = new JoystickButton(altJoystick, 7);
        JoystickButton btnColorSensor = new JoystickButton(leftJoystick, 5);
        JoystickButton btnCPStop = new JoystickButton(leftJoystick, 2);
        JoystickButton btnInvertYAxis = new JoystickButton(leftJoystick, 6);
        JoystickButton btnRotationSensitivity = new JoystickButton(rightJoystick, 1);
        JoystickButton btnIntakeBottomOut = new JoystickButton(altJoystick, 6);
        JoystickButton btnLED = new JoystickButton(altJoystick, 5);

        driveSubsystem.setDefaultCommand(new RunCommand(
            () -> driveSubsystem.drive(-leftJoystick.getY() * 0.7, rightJoystick.getX() * 0.7),
            driveSubsystem));

        btnLauncherSolenoid.whenPressed(
            new AutomaticShootCommand(TARGET_VELOCITY, NUM_BALLS_LOADED, shooterSubsystem)
        );

        btnIntakeSolenoid.whenPressed(new InstantCommand(intakeSubsystem::extend, intakeSubsystem))
        .whenInactive(new InstantCommand(intakeSubsystem::retract, intakeSubsystem));
      
        btnIntakeOut.whenHeld(new InstantCommand(() -> intakeSubsystem.spin(0.5, 0.7), intakeSubsystem))
            .whenInactive(new InstantCommand(() -> intakeSubsystem.spin(0, 0), intakeSubsystem), true);
        btnIntakeIn.whenHeld(new InstantCommand(() -> intakeSubsystem.spin(-0.5, -0.7), intakeSubsystem))
            .whenInactive(new InstantCommand(() -> intakeSubsystem.spin(0, 0), intakeSubsystem), true);

        // btnLauncherMotor.whenHeld(new InstantCommand(() -> shooterSubsystem.shootVelocity(6000), shooterSubsystem))
            // .whenInactive(new InstantCommand(() -> shooterSubsystem.shootVoltage(0), shooterSubsystem), true); 

        btnLED.whenPressed(new InstantCommand(() -> ledDriver.set(ledDriver.AUTONOMOUS)));
    }
    // random pattern -> -.99


    public void configureObjects() {
        frontLeftMotor.setIdleMode(IdleMode.kBrake);
        frontRightMotor.setIdleMode(IdleMode.kBrake);
        rearLeftMotor.setIdleMode(IdleMode.kBrake);
        rearRightMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        gyro.reset();
        HashMap<String, Trajectory> trajectories = trajectoryLoader.loadTrajectories();
        Trajectory testTrajectory = trajectories.get("Test");

        // Create a voltage constraint to ensure we don't accelerate too fast
        DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                DriveSubsystem.FEED_FORWARD,
                DriveSubsystem.KINEMATICS,
                MAX_GENERATION_VOLTAGE);
        
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(MAX_GENERATION_VELOCITY,
                                MAX_GENERATION_ACCELERATION)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveSubsystem.KINEMATICS)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
        
        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                //TODO fix turning
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1, 0, new Rotation2d(0)),
            // Pass config
            config
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
            testTrajectory,
            odometry::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(DRIVE_KS,
                                    DRIVE_KV,
                                    DRIVE_KA),
            DriveSubsystem.KINEMATICS,
            odometry::getWheelSpeeds,
            new PIDController(0.06, 0, 0),
            new PIDController(0.06, 0, 0),
            // RamseteCommand passes volts to the callback
            driveSubsystem::tankDriveVolts,
            driveSubsystem
        );

        // Reset odometry to the starting pose of the trajectory.
        odometry.reset(testTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> driveTrain.tankDrive(0, 0));
    }

    public LEDDriver getLEDDriver() {
        return ledDriver;
    }
}
