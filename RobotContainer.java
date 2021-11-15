// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  // Assumes a gamepad plugged into channnel 0
  private final Joystick m_controller = new Joystick(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    onboardButtonA
        //.whenActive(new PrintCommand("Button A Pressed"))
        .whenActive(new AutonomousDistance(m_drivetrain))
        .whenInactive(new PrintCommand("Button A Released"));

    // Example of how to use the onboard IO
    Button onboardButtonB = new Button(m_onboardIO::getButtonBPressed);
    onboardButtonB
        .whenActive(new PrintCommand("Button A Pressed"))
        .whenActive(new AutonomousDistance(m_drivetrain))
        .whenInactive(new PrintCommand("Button B Released"));
        new JoystickButton(m_controller, 1)
        .whenPressed(
          autonomousScenario1(m_drivetrain)
        );
        

        new JoystickButton(m_controller, 2)
        .whenPressed(
            autonomousScenario2(m_drivetrain)
        );
        new JoystickButton(m_controller, 3)
        .whenPressed(
            autonomousScenario3(m_drivetrain)
        );
    //     new JoystickButton(m_controller, 4)
    //     .whenPressed(new SequentialCommandGroup(
    //         new TurnDegrees(-0.5, 90, m_drivetrain),
    //         new DriveDistance(0.5, 17, m_drivetrain)
    //     ));

    // Shayna start: Code from slides 
    // B/Red: Begin recording a path
    // new JoystickButton(m_controller, 3)
    // .whenPressed(() -> startMotionRecord());
    // // Y/Yellow: End recording a path
    // new JoystickButton(m_controller, 4)
    // .whenPressed(() -> stopMotionRecord());
    // // A/Green: Play back the recorded path
    // new JoystickButton(m_controller, 2)
    // .whenPressed(() -> playMotionRecord());
    // // X/Blue: Recording the current pose as a waypoint:
    // new JoystickButton(m_controller, 1)
    // .whenPressed(() -> motionRecordPose());
        
    // POV button to issue drive-by-Ramsete commands:
    new POVButton(m_controller, 0, 0)
    .whenPressed(() -> {translateDrive(0.10, 0.0);});
    new POVButton(m_controller, 90, 0)
    .whenPressed(() -> {translateDrive(0.10, Math.PI/4);});
    new POVButton(m_controller, 180, 0)
    .whenPressed(() -> {translateDrive(-0.10, 0.0);});
    new POVButton(m_controller, 270, 0)
    .whenPressed(() -> {translateDrive(0.10, -Math.PI/4);});

    // Shayna end: Code from slides



    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getRawAxis(1), () -> m_controller.getRawAxis(4));
  }

  //copy pasted code!
  private Trajectory trajectoryForPath(List<Pose2d> path, boolean reversed) {
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(DriveConstants.ksVolts,
    DriveConstants.kvVoltSecondsPerMeter,
    DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics,
    10);
    TrajectoryConfig config =
    new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(DriveConstants.kDriveKinematics)
    .addConstraint(autoVoltageConstraint)
    .setReversed(reversed);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    path,
    config);
    return trajectory;
  }

  private RamseteCommand ramseteCommandForTrajectory(Trajectory trajectory) {
      RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
      m_drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
      DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      m_drivetrain::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      m_drivetrain::tankDriveVolts,
      m_drivetrain);
      return ramseteCommand;
  }
  
  private Command ramseteCommandForPath(List<Pose2d> path, boolean reversed) {
      Trajectory trajectory = trajectoryForPath(path, reversed);
      RamseteCommand ramseteCommand = ramseteCommandForTrajectory(trajectory);
      // Set up a sequence of commands
      // First, we want to reset the drivetrain odometry
      return new InstantCommand(() -> m_drivetrain.resetOdometry(trajectory.getInitialPose()), m_drivetrain)
      // next, we run the actual ramsete command
      .andThen(ramseteCommand)
      // Finally, we make sure that the robot stops
      .andThen(new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain));
  }
  
  private RamseteCommand immediateRamseteCommand(List<Pose2d> path, boolean reversed) {
      Trajectory trajectory = trajectoryForPath(path, reversed);
      RamseteCommand ramseteCommand = ramseteCommandForTrajectory(trajectory);
      // Since this method is meant to be used for generating continuous drive control paths,
      // we don't interrupt/reset the odometry or stop the motors at the end of the path.
      return ramseteCommand;
  } 

  // 1153: Path Recorder functionality:
  private boolean m_record = false;
  private Pose2d m_prevPose = new Pose2d();
  private final double kRecordDelta = 0.001; // 0.01 = 1cm
  private List<Pose2d> m_recordPath = new ArrayList<Pose2d>();

  // Record the current robot location as a waypoint:
  private void recordWaypoint() {
    Pose2d pose = m_drivetrain.getPose();
    m_recordPath.add(pose);
  }
  // Erase any previously recorded waypoints:
  private void eraseMotionRecord() {
    m_recordPath = new ArrayList<Pose2d>();
    m_prevPose = new Pose2d();
    m_recordPath.add(m_prevPose);
    m_drivetrain.resetOdometry(m_prevPose);
  }
  // Print the recorded path to the console:
  private void printMotionRecordPath() {
    System.out.println("List.of(");
    for (Pose2d pose : m_recordPath) {
      System.out.println(" new Pose2d("+pose.getX()+", "+pose.getY()
      +"), new Rotation2d("+pose.getRotation().getRadians()+")),");
    }
    System.out.println("); // TODO: Remove the trailing comma from the line above");
  }

  // Begin recording a new path:
  private void startMotionRecord() {
    eraseMotionRecord();
    m_record = true;
  }
  // End recording:
  private void stopMotionRecord() {
    m_record = false;
  }

  // Play back the recorded path:
  private void playMotionRecord() {
    stopMotionRecord();
    printMotionRecordPath();
    ramseteCommandForPath(m_recordPath, false).schedule();
  }

  // If the robot has moved and we're recording, capture a new waypoint:
 private void motionRecordPose() {
    if (m_record)
    {
        Pose2d pose = m_drivetrain.getPose();
        double distance = m_prevPose.getTranslation().getDistance(pose.getTranslation());
        if (distance > kRecordDelta) {
          m_prevPose = pose;
          recordWaypoint();
        }
    }
  }
  
  public void teleopPeriodic() {
    // 1153: Recordable trajectory support:
      motionRecordPose();
    }
  

 private Command ramseteCommandForMySavedPath() {
  return ramseteCommandForPath(
  // List of waypoints copied from the console into a Java function:
  List.of(
  new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  new Pose2d(0.010377342492793486, 2.6626632801004613E-4, new Rotation2d(0.08429630782081529)),
  new Pose2d(0.020304816923615306, 0.0014845293441094367, new Rotation2d(0.12854580149869685)),
  new Pose2d(0.030380183147045197, 0.0027582181871425764, new Rotation2d(0.13751835708718269)),
  new Pose2d(0.04043942388648978, 0.004153498851978208, new Rotation2d(0.13218750719569722)),
  new Pose2d(0.051325035933770774, 0.005703636512249553, new Rotation2d(0.14925765165738694)),
  new Pose2d(0.06345928369800993, 0.007674885693290004, new Rotation2d(0.1731994110386956)),
  new Pose2d(0.07500219174726039, 0.009915518019660025, new Rotation2d(0.2127550936025266)),
  new Pose2d(0.08601740168044585, 0.012439798441654852, new Rotation2d(0.23734738781060172))
  ),
 false);
  }

  //TeleOp Trajectories
  private Command m_driveCommand = null;
  void drive(Command driveCommand) {
  // Preempt any existing command:
  if (m_driveCommand != null) {
  m_driveCommand.cancel();
  }
  // Schedule and take note of the new drive command:
  m_driveCommand = driveCommand;
  if (m_driveCommand != null) {
  m_driveCommand.schedule();
  } else {
  // No command? then stop.
  m_drivetrain.tankDriveVolts(0, 0);
  }
  }
  
  // Generate and immediately execute a short Ramsete trajectory:
 private void translateDrive(double distance, double angle) {
    boolean reversed = (distance < 0.0)? true : false;
    Rotation2d turn = new Rotation2d(angle);
    Translation2d vector = new Translation2d(distance, 0.0).rotateBy(turn);
    Transform2d delta = new Transform2d(vector, turn);
    Pose2d here = m_drivetrain.getPose();
    drive(immediateRamseteCommand(
    List.of(
    here,
    here.plus(delta)
    ),
    reversed));
    }
    // Halt any drive command that may be running:
    private void translateDriveHalt() {
    drive(null);
    }

   

    public Command autonomousScenario1(Drivetrain drivetrain) {
      return new SequentialCommandGroup(
        new DriveDistance(0.5, 72, drivetrain),
        new TurnDegrees(0.4, 180, drivetrain),
        //shoot()
        //pause
        new TurnDegrees(0.4, 90, drivetrain),
        new DriveDistance(0.5, 10, drivetrain)
      );
    }

      public Command autonomousScenario2(Drivetrain drivetrain) {
        return new SequentialCommandGroup(
          new DriveDistance(0.5, 15, drivetrain),
          new TurnDegrees(-0.4, 90, drivetrain),
          new DriveDistance(0.5, 30, drivetrain),
          new TurnDegrees(0.4, 90, drivetrain),
          new DriveDistance(0.5, 45, drivetrain),
          new TurnDegrees(0.4, 140, drivetrain)
          //insert trajectory 
        ); }
      
        public Command autonomousScenario3(Drivetrain drivetrain) {
          return new SequentialCommandGroup(
            new DriveDistance(0.5, 37, drivetrain),
            new TurnDegrees(-0.4, 90, drivetrain),
            new DriveDistance(0.5, 30, drivetrain),
            new TurnDegrees(0.4, 90, drivetrain),
            new DriveDistance(0.5, 25, drivetrain),
            new TurnDegrees(0.4, 140, drivetrain)
            //insert trajectory 
          );
      
         
          
    }
    
    }

 
  