#include "Aria.h"
#include "iostream"

void robotAlign(ArRobot &robot, ArPose &dest)
{
  while (robot.findAngleTo(dest) > 0.001)
  {
    std::cout << robot.findAngleTo(dest) << std::endl;
    robot.lock();
    robot.setRotVel(10);
    robot.unlock();
    ArUtil::sleep(1000);
  }
}

void displayLaserInfo(ArRobot &robot, const ArRobotParams *params)
{
  for (size_t i = 1; i <= robot.getNumLasers(); ++i)
  {
    if (robot.findLaser(i))
    {
      int x = params->getLaserX(i);
      int y = params->getLaserY(i);
      int z = params->getLaserZ(i);
      int th = params->getLaserTh(i);
      ArLog::log(ArLog::Normal, "\tlaser #%d (%d, %d, %d) th:%f", x, y, z, th);
    }
  }
  ArUtil::sleep(1000);
}

void displayLaserStat(ArRobot &robot, const ArRobotParams *params)
{
  ArPose *dest;
  int numLasers = 0;
  robot.lock();
  std::map<int, ArLaser*> *lasers = robot.getLaserMap();
  ArLog::log(ArLog::Normal, "Carto project: %d ArLaser.", lasers->size());
  auto robotPose = robot.getPose();

  for (std::map<int, ArLaser*>::const_iterator i = lasers->begin();
      i != lasers->end(); ++i)
  {
    int laserIndex = i->first;
    ArLaser *laser = i->second;
    if (laser)
    {
      ++numLasers;
      laser->lockDevice();

      std::list<ArPoseWithTime*> *currentReadings = laser->getCurrentBuffer();
      ArLog::log(ArLog::Normal, "Carto project: %d readings.",
        currentReadings->size());
      
      ArLog::log(ArLog::Normal, "Carto project: robot X: %2.4f Y:%2.4f th:%3.0f",
        robotPose.getX() / 1000, robotPose.getY() / 1000, robotPose.getTh());
      for(auto pose : *currentReadings)
      {
/*        ArLog::log(ArLog::Normal,
          "Carto project: X: %2.4f Y:%2.4f th:%3.0f, dist:%2.4f, angle: %3.0f",
          pose->getX() / 1000,
          pose->getY() / 1000,
          pose->getTh(),
          pose->findDistanceTo(robotPose) / 1000,
          pose->findAngleTo(robotPose));*/
        ArLog::log(ArLog::Normal,
          "%2.4f,",
          pose->getY() / 1000);
      }
      dest = currentReadings->back();
      laser->unlockDevice();
    }
  }
  robot.unlock();
  ArLog::log(ArLog::Normal, "Carto project: Stat Laser END.");
  ArUtil::sleep(1000);
  robotAlign(robot, *dest);
}

int main(int argc, char **argv)
{

  Aria::init();
  ArRobot robot;
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  // ArRobotConnector connects to the robot, get some initial data from it such as type and name,
  // and then loads parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);

  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "simpleMotionCommands: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
        return 1;
    }
  }
  if (!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::exit(1);
    return 1;
  }
  
  ArLog::log(ArLog::Normal, "Carto project: Connected to robot.");

  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the 
  // run loop ends.
  robot.runAsync(true);

  // Print out some data from the SIP.  

  // We must "lock" the ArRobot object
  // before calling its methods, and "unlock" when done, to prevent conflicts
  // with the background thread started by the call to robot.runAsync() above.
  // See the section on threading in the manual for more about this.
  // Make sure you unlock before any sleep() call or any other code that will
  // take some time; if the robot remains locked during that time, then
  // ArRobot's background thread will be blocked and unable to communicate with
  // the robot, call tasks, etc.
  
  /*
   * Connect to laser and error handling
   */

  if (!laserConnector.connectLasers())
  {
    ArLog::log(ArLog::Terse, "Carto project: could not connect to lasers.");
    Aria::exit(3);
    return 3;
  }
  ArLog::log(ArLog::Terse, "Carto project: Connected to lasers.");
  ArLog::log(ArLog::Normal, "Carto project: connected to %d lasers", robot.getNumLasers());
  const ArRobotParams *params = robot.getRobotParams();
  displayLaserInfo(robot, params);
  displayLaserStat(robot, params);

  robot.lock();
  ArLog::log(ArLog::Normal,
    "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(),
    robot.getTh(), robot.getVel(),
    robot.getRotVel(), robot.getBatteryVoltage());
  robot.unlock();

  // Sleep for 3 seconds.
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Will start driving in 3 seconds...");
  ArUtil::sleep(3000);

  // Set forward velocity to 50 mm/s
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Driving forward at 250 mm/s for 5 sec...");
  robot.lock();
  robot.enableMotors();
  robot.setVel(250);
  robot.unlock();
  ArUtil::sleep(5000);
  //displayLaserStat(robot, params);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Stopping.");
  robot.lock();
  robot.stop();
  robot.unlock();
  ArUtil::sleep(1000);
/*
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Rotating at 10 deg/s for 5 sec...");
  robot.lock();
  robot.setRotVel(10);
  robot.unlock();
  ArUtil::sleep(5000);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Rotating at -10 deg/s for 10 sec...");
  robot.lock();
  robot.setRotVel(-5);
  robot.unlock();
  ArUtil::sleep(10000);
*/
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Driving forward at 150 mm/s for 5 sec...");
  robot.lock();
  robot.setRotVel(0);
  robot.setVel(150);
  robot.unlock();
  ArUtil::sleep(5000);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Stopping.");
  robot.lock();
  robot.stop();
  robot.unlock();
  ArUtil::sleep(1000);

  //displayLaserInfo(robot, params);


  // Other motion command functions include move(), setHeading(),
  // setDeltaHeading().  You can also adjust acceleration and deceleration
  // values used by the robot with setAccel(), setDecel(), setRotAccel(),
  // setRotDecel().  See the ArRobot class documentation for more.

  
  robot.lock();
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
  robot.unlock();

  
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Ending robot thread...");
  robot.stopRunning();

  // wait for the thread to stop
  robot.waitForRunExit();

  // exit
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  Aria::exit(0);
  return 0;
}
