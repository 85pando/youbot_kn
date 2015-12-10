#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

void usage(char *argv[])
{
  cerr << "Usage " << argv[0] << " [CONFIG.yaml] INPUT.csv/.vtk OUTPUT.csv/.vtk" << endl;
  cerr << endl << "Example:" << endl;
  cerr << argv[0] << " ./default-convert.yaml ./cloud.00000 /tmp/output.vtk" << endl << endl;
}

int main(int argc, char *argv[])
{
  
  if (argc < 3)
  {
    usage(argv);
    return 1;
  }
  
  // create logger for output
  setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
  
  // DataPoints object to store the data points
  DP d(DP::load(argv[argc-2]));
  
  if (argc == 4)
  {
    //configuration file is valid file
    ifstream ifs(argv[1]);
    if (!ifs.good())
    {
      cerr << "Cannot open config file " << argv[1] << endl;
      usage(argv);
      return 2;
    }
    //use default data points filters
    PM::DataPointsFilters f(ifs);
    f.apply(d);
  }
  
  d.save(argv[argc-1]);
  return 0;
}