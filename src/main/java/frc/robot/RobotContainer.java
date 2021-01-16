/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                                                */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChargeAutoCommand;
import frc.robot.common.ControlPanelColor;
import frc.robot.common.ControlPanelColorSensor;
import frc.robot.common.Limelight;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.logging.Logger;

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

    // DRIVE SUBSYSTEM
    private final CANSparkMax frontLeftMotor = new CANSparkMax(FRONT_LEFT_MOTOR,
        MotorType.kBrushless);
    private final CANSparkMax frontRightMotor = new CANSparkMax(FRONT_RIGHT_MOTOR,
        MotorType.kBrushless);
    private final CANSparkMax rearLeftMotor = new CANSparkMax(REAR_LEFT_MOTOR,
        MotorType.kBrushless);
    private final CANSparkMax rearRightMotor = new CANSparkMax(REAR_RIGHT_MOTOR,
        MotorType.kBrushless);
    private final DifferentialDrive driveTrain = new DifferentialDrive(
        new SpeedControllerGroup(frontLeftMotor, rearLeftMotor),
        new SpeedControllerGroup(frontRightMotor, rearRightMotor));
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(driveTrain);

    // SHOOTER SUBSYSTEM
    private final CANSparkMax shooterMotor1 = new CANSparkMax(SHOOTER_MOTOR_1, MotorType.kBrushed);
    private final CANSparkMax shooterMotor2 = new CANSparkMax(Constants.SHOOTER_MOTOR_2,
        MotorType.kBrushed);
    private final Solenoid shooterSolenoid = new Solenoid(SHOOTER_SOLENOID_PORT);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(shooterMotor1,
        shooterMotor2, shooterSolenoid);

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


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        //Configure the button bindings
        configureButtonBindings();
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

        driveSubsystem.setDefaultCommand(new RunCommand(
            () -> driveSubsystem.drive(-leftJoystick.getY() * 0.7, rightJoystick.getX() * 0.7),
            driveSubsystem));
        shooterSubsystem.setDefaultCommand(new RunCommand(() -> {
            shooterSubsystem.shoot(Math.abs(altJoystick.getY()));
            System.out.println("Spinning");
        }, shooterSubsystem));

        btnLauncherSolenoid.whenPressed(
            new InstantCommand(shooterSubsystem::activatePiston, shooterSubsystem))
            .whenInactive(new InstantCommand(shooterSubsystem::lowerPiston, shooterSubsystem));

        btnIntakeOut.whenHeld(new InstantCommand(() -> intakeSubsystem.spin(0.5), intakeSubsystem))
            .whenInactive(new InstantCommand(() -> intakeSubsystem.spin(0), intakeSubsystem), true);
        btnIntakeIn.whenHeld(new InstantCommand(() -> intakeSubsystem.spin(-0.5), intakeSubsystem))
            .whenInactive(new InstantCommand(() -> intakeSubsystem.spin(0), intakeSubsystem), true);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        Command charge = new ChargeAutoCommand(driveSubsystem, 0.2).withTimeout(1);
        return charge;
    }
}
