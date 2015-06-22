#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>
using namespace std;

/**
 * This program shows the distances of the primary directions of the LaserScanner.
 */

// Begin Magic Numbers
const uint NROFDIRS       = 9;
// if this is changed, printMainDirections has to be changed as well!
const uint NROFDIGITS     = 3;
const uint SCANNERANGLE   = 180;
const uint NROFDATAPOINTS = 640;
// End Magic Numbers

/**
 * Is called upon receiveing a LaserScan message.
 */
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
  const uint seqNr            = data->header.seq;
  const uint nrOfMeasurements = sizeof(data->ranges)/sizeof(data->ranges[0]);

  // put range values in array
  float laserArray[NROFDATAPOINTS] = {};
  for (uint i=0; i<NROFDATAPOINTS; i++)
  {
    laserArray[i] = data->ranges[i];
  }

  // take only main directions
  float directions[NROFDIRS] = {};
  for (uint i=0; i<NROFDIRS; i++)
  {
    if (i != 0)
      directions[i] = laserArray[(int) (i*NROFDATAPOINTS/(NROFDIRS-1))-1];
    else
      directions[i] = laserArray[0];
  }

  // print out values
  // prepare separator
  string lineString      = "+";
  for (uint i=0; i<NROFDIRS; i++)
  {
    lineString  += "-------+";
  }
  lineString += "\n";

  printf("\nSequence Number: %10u", seqNr);
//   cout << "\nSequence Number: " << seqNr << "\n";
  cout << "\n" << lineString;
  // header - angles
  cout << "|";
  for (uint i=0; i<NROFDIRS; i++)
  {
    printf(" %5u |", i*SCANNERANGLE / (NROFDIRS-1));
  }
  cout << "\n";
  cout << lineString;
  // body - ranges
  cout << "|";
  for (uint i=0; i<NROFDIRS; i++)
  {
    printf(" %1.3f |", directions[i]);
  }
  cout << "\n";
  cout << lineString;
}

int main(int argc, char **argv)
{
  // Initialize ROS system
  ros::init(argc, argv, "scanner_viz");

  // Initialize node
  ros::NodeHandle n;

  // Subscribe to the LaserScan topic
  ros::Subscriber laserSub = n.subscribe("/youbot/scan_front", 5, laserCallback);

  // keep alive
  ros::spin();

  return 0;
}
