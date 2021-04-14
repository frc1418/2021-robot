/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                                                */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.BOTTOM_INTAKE_MOTOR;
import static frc.robot.Constants.CONTROL_PANEL_MOTOR;
import static frc.robot.Constants.CONTROL_PANEL_SOLENOID_FWD;
import static frc.robot.Constants.CONTROL_PANEL_SOLENOID_REV;
import static frc.robot.Constants.FRONT_LEFT_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MOTOR;
import static frc.robot.Constants.INTAKE_SOLENOID_FWD;
import static frc.robot.Constants.INTAKE_SOLENOID_REV;
import static frc.robot.Constants.INTAKE_SWITCH;
import static frc.robot.Constants.REAR_LEFT_MOTOR;
import static frc.robot.Constants.REAR_RIGHT_MOTOR;
import static frc.robot.Constants.SHOOTER_MOTOR_1;
import static frc.robot.Constants.SHOOTER_SOLENOID_PORT;
import static frc.robot.Constants.SHOOTER_ULTRASONIC_ECHO;
import static frc.robot.Constants.SHOOTER_ULTRASONIC_TRIG;
import static frc.robot.Constants.UPPER_INTAKE_MOTOR;

import java.util.HashMap;
import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignWithGyroCommand;
import frc.robot.commands.AutomaticShootCommand;
import frc.robot.commands.ToggleIntakePistonCommand;
import frc.robot.commands.autonomous.BarrelRacing;
import frc.robot.commands.autonomous.Bounce;
import frc.robot.commands.autonomous.Slalom;
import frc.robot.common.ControlPanelColorSensor;
import frc.robot.common.LEDDriver;
import frc.robot.common.Odometry;
import frc.robot.common.TrajectoryLoader;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final RobotBase robot;
    private final Logger logger = Logger.getLogger("Robot");
    private final Limelight limelight = new Limelight();

    // NETWORKTABLES
    private final Field2d field = new Field2d();
    private final Timer timer = new Timer();


    // DRIVE
    private final CANSparkMax frontLeftMotor = new CANSparkMax(FRONT_LEFT_MOTOR,
        MotorType.kBrushless);
    private final CANSparkMax frontRightMotor = new CANSparkMax(FRONT_RIGHT_MOTOR,
        MotorType.kBrushless);
    private final CANSparkMax rearLeftMotor = new CANSparkMax(REAR_LEFT_MOTOR,
        MotorType.kBrushless);
    private final CANSparkMax rearRightMotor = new CANSparkMax(REAR_RIGHT_MOTOR,
        MotorType.kBrushless);
    private double xSpeedMultiplier = 0.8;

    // LED
    private final LEDDriver ledDriver = new LEDDriver(1);

    // ODOMETRY
    private final Gyro gyro = new AHRS(SPI.Port.kMXP);

    private final CANEncoder leftEncoder = frontLeftMotor.getEncoder();
    private final CANEncoder rightEncoder = frontRightMotor.getEncoder();

    private final DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    private final Odometry odometry = new Odometry(gyro, driveOdometry, leftEncoder, rightEncoder);

    // DRIVE SUBSYSTEM
    private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeftMotor, rearLeftMotor);
    private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRightMotor, rearRightMotor);

    private final DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(driveTrain, leftMotors, rightMotors, odometry, field, timer);
    

    // SHOOTER SUBSYSTEM
    private final CANSparkMax shooterMotor1 = new CANSparkMax(SHOOTER_MOTOR_1, MotorType.kBrushed);
    private final CANEncoder shooterEncoder = shooterMotor1.getEncoder(EncoderType.kQuadrature, 8192);
    private final CANSparkMax shooterMotor2 = new CANSparkMax(frc.robot.Constants.SHOOTER_MOTOR_2,
        MotorType.kBrushed);
    private final Solenoid shooterSolenoid = new Solenoid(SHOOTER_SOLENOID_PORT);
    private final Ultrasonic ballSensor = new Ultrasonic(SHOOTER_ULTRASONIC_TRIG,
        SHOOTER_ULTRASONIC_ECHO, Unit.kInches);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(shooterMotor1,
        shooterMotor2, shooterSolenoid, shooterEncoder, ballSensor, ledDriver);

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

    // CLIMB SUBSYSTEM
        private final SpeedControllerGroup winchMotors = new SpeedControllerGroup(new CANSparkMax(8, MotorType.kBrushless), new CANSparkMax(9, MotorType.kBrushless));
        private final DoubleSolenoid scissorSolenoid = new DoubleSolenoid(6, 7);
        private final WPI_VictorSPX hookMotor = new WPI_VictorSPX(3);

        private final ClimbSubsystem climbSubsystem = new ClimbSubsystem(winchMotors, scissorSolenoid, hookMotor);


    // TRAJECTORIES
    private final TrajectoryLoader trajectoryLoader = new TrajectoryLoader();
    // private final HashMap<String, Trajectory> trajectories = trajectoryLoader.loadTrajectories();

    // NAVX
    private final AHRS navx = new AHRS(SPI.Port.kMXP);

    // SENDABLE CHOOSER
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    private final Command automaticShootCommand = new AutomaticShootCommand(20, 3, shooterSubsystem);
    private final Command automaticShootCommand2 = new AutomaticShootCommand(25, 3, shooterSubsystem);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(RobotBase robot) {
        this.robot = robot;

        m_chooser.setDefaultOption("Auto Shoot Command", automaticShootCommand);
        m_chooser.addOption("Auto Shoot Command 2", automaticShootCommand2);
        SmartDashboard.putData(m_chooser);

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
        JoystickButton btnBoost = new JoystickButton(leftJoystick, 1);
        JoystickButton btnIntakeIn = new JoystickButton(altJoystick, 3);
        JoystickButton btnIntakeOut = new JoystickButton(altJoystick, 4);
        JoystickButton btnIntakeUpperIn = new JoystickButton(altJoystick, 5);
        JoystickButton btnIntakeUpperOut = new JoystickButton(altJoystick, 6);
        JoystickButton btnCPExtend = new JoystickButton(leftJoystick,
            4); // toggle: use "toggleWhenPressed" method to set command
        JoystickButton btnWinch = new JoystickButton(altJoystick, 8);
        JoystickButton btnCPMotor = new JoystickButton(leftJoystick, 3);
        JoystickButton btnLauncherMotor = new JoystickButton(altJoystick, 12);
        JoystickButton btnLauncherIdle = new JoystickButton(altJoystick,
            10); // toggle: use "toggleWhenPressed" method to set command
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

        JoystickButton btnTestAlign = new JoystickButton(rightJoystick, 1);

        Trigger rightPOV = new Trigger(() -> altJoystick.getPOV(0) == 90);
        Trigger leftPOV = new Trigger(() -> altJoystick.getPOV(0) == 270);

        driveSubsystem.setDefaultCommand(new RunCommand(
            () -> {
                if (robot.isOperatorControlEnabled()) {
                    driveSubsystem.joystickDrive(-leftJoystick.getY() * xSpeedMultiplier, rightJoystick.getX() * 0.65);
                } else {
                    driveSubsystem.drive(0, 0);
                }
            },
            driveSubsystem));

        btnLauncherSolenoid
            .whenHeld(
                new AutomaticShootCommand(0, -1, shooterSubsystem).perpetually()
            );

        btnLauncherMotor.whenPressed(new InstantCommand(() -> {
            if (limelight.getPlaneDistance() > -1)
                shooterSubsystem.shootVelocity(shooterSubsystem.getDistanceToRPM((int)(limelight.getPlaneDistance())));
        }, shooterSubsystem));
        btnLauncherMotor.whenReleased(new InstantCommand(() -> shooterSubsystem.shootVoltage(0), shooterSubsystem));

        btnLauncherIdle
            .whenPressed(new InstantCommand(() -> shooterSubsystem.shootVelocity(3000), shooterSubsystem))
            .whenReleased(new InstantCommand(() -> shooterSubsystem.shootVoltage(0), shooterSubsystem));

        btnBoost
                .whenPressed(new InstantCommand(() -> xSpeedMultiplier = 0.85))
                .whenReleased(new InstantCommand(() -> xSpeedMultiplier = 0.8));
    
        btnIntakeSolenoid.toggleWhenPressed(new ToggleIntakePistonCommand(intakeSubsystem), true);            
      
        btnIntakeOut.whileHeld(new InstantCommand(() -> intakeSubsystem.spin(7, 5), intakeSubsystem))
            .whenInactive(new InstantCommand(() -> intakeSubsystem.spin(0, 0), intakeSubsystem), true);
        btnIntakeIn.whileHeld(new InstantCommand(() -> intakeSubsystem.spin(-7, -5.75), intakeSubsystem))
            .whenInactive(new InstantCommand(() -> intakeSubsystem.spin(0, 0), intakeSubsystem), true);

        btnIntakeUpperOut.whileHeld(new InstantCommand(() -> intakeSubsystem.spin(7, 0), intakeSubsystem))
            .whenInactive(new InstantCommand(() -> intakeSubsystem.spin(0, 0), intakeSubsystem), true);
        btnIntakeUpperIn.whileHeld(new InstantCommand(() -> intakeSubsystem.spin(-7, 0), intakeSubsystem))
            .whenInactive(new InstantCommand(() -> intakeSubsystem.spin(0, 0), intakeSubsystem), true);

        // btnLauncherMotor.whenHeld(new InstantCommand(() -> shooterSubsystem.shootVelocity(6000), shooterSubsystem))
            // .whenInactive(new InstantCommand(() -> shooterSubsystem.shootVoltage(0), shooterSubsystem), true); 

        btnLED.whenPressed(new InstantCommand(() -> ledDriver.set(ledDriver.AUTONOMOUS)));

        btnTestAlign.whenHeld(new AlignWithGyroCommand(navx, driveSubsystem, 0));

        btnWinch.whenPressed(new InstantCommand(() -> climbSubsystem.setWinchMotors(0.65)));
        btnWinch.whenReleased(new InstantCommand(() -> climbSubsystem.setWinchMotors(0)));

        btnScissorExtend.whenPressed(new InstantCommand(() -> climbSubsystem.setScissorSolenoid(DoubleSolenoid.Value.kForward)));
        btnScissorExtend.whenReleased(new InstantCommand(() -> climbSubsystem.setScissorSolenoid(DoubleSolenoid.Value.kReverse)));

        rightPOV.whenActive(new InstantCommand(() -> climbSubsystem.setHookMotor(0.5)));
        leftPOV.whenActive(new InstantCommand(() -> climbSubsystem.setHookMotor(-0.5)));
        rightPOV.or(leftPOV).whenInactive(new InstantCommand(() -> climbSubsystem.setHookMotor(0)));
    }
    // random pattern -> -.99


    public void configureObjects() {
        frontLeftMotor.setIdleMode(IdleMode.kBrake);
        frontRightMotor.setIdleMode(IdleMode.kBrake);
        rearLeftMotor.setIdleMode(IdleMode.kBrake);
        rearRightMotor.setIdleMode(IdleMode.kBrake);

        odometry.zeroHeading();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        odometry.zeroHeading();
        return new Slalom(driveSubsystem, odometry, limelight, navx);
    }

    public Timer getTimer() {
        return timer;
    }

    public LEDDriver getLEDDriver() {
        return ledDriver;
    }

    public Odometry getOdometry() {
        return odometry;
    }

}
