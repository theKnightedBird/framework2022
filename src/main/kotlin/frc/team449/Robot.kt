package frc.team449

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team449.robot2022.RobotContainer2022
import io.github.oblarg.oblog.Logger

/** The main class of the robot, constructs all the subsystems and initializes default commands. */
class Robot : TimedRobot(.005) {

  private val robotContainer: RobotContainerBase = RobotContainer2022()
  private var autoCommand: Command? = null

  init {
    LiveWindow.disableAllTelemetry()
  }
  override fun robotInit() {
    // Yes this should be a print statement, it's useful to know that robotInit started.
    println("Started robotInit.")

    if (RobotBase.isSimulation()) {
      // Don't complain about joysticks if there aren't going to be any
      DriverStation.silenceJoystickConnectionWarning(true)
    }

    robotContainer.robotInit()

    Logger.configureLoggingAndConfig(robotContainer, false)
    Shuffleboard.setRecordingFileNameFormat("log-\${time}")
    Shuffleboard.startRecording()
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()

    Logger.updateEntries()

    robotContainer.robotPeriodic()
  }

  override fun autonomousInit() {
    robotContainer.autonomousInit()
  }

  override fun autonomousPeriodic() {
    robotContainer.autonomousPeriodic()
  }

  override fun teleopInit() {
    if (autoCommand != null) {
      CommandScheduler.getInstance().cancel(autoCommand)
    }
    robotContainer.teleopInit()
  }

  override fun teleopPeriodic() {
    robotContainer.teleopPeriodic()
  }

  override fun disabledInit() {
    robotContainer.disabledInit()
  }

  override fun disabledPeriodic() {
  }

  override fun testInit() {
    if (autoCommand != null) {
      CommandScheduler.getInstance().cancel(autoCommand)
    }
  }

  override fun testPeriodic() {}

  override fun simulationInit() {
    robotContainer.simulationInit()
  }

  override fun simulationPeriodic() {
    robotContainer.simulationPeriodic()
  }
}
