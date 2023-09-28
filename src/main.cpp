#include "cmaes.h"
#include "parameters.h"
#include <cxxopts.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/icp.h>
#include <limits>

// 2d Point cloud datatype
typedef pcl::PointCloud<pcl::PointXY> PCl2D;
// Shared pointer to a 2d pointcloud
typedef PCl2D::Ptr PClPtr;

// Runtime-modified settings of the program
struct Settings
{
  // Translation according to the screen input using mouse and evolutionary fit
  float g_f_translate_x, g_f_translate_y;
  // Scaling according to the screen input using mouse and evolutionary fit
  float g_f_stretch_x, g_f_stretch_y;
  // Scaling according to our evolutionary fit
  float g_f_shear_x, g_f_shear_y;

  // Mouse button event flags
  bool g_bEventLButtonDown;
  bool g_bEventRButtonDown;
  // Flag to perform point cloud optimization
  bool g_bDoCmaes;
};

// Configuration options loaded at start
struct Configuration
{
  float g_f_stretch_x_min = -std::numeric_limits<float>::max();
  float g_f_stretch_x_max = std::numeric_limits<float>::max();
  float g_f_stretch_y_min = -std::numeric_limits<float>::max();
  float g_f_stretch_y_max = std::numeric_limits<float>::max();

  float g_f_shear_x_min = -std::numeric_limits<float>::max();
  float g_f_shear_x_max = std::numeric_limits<float>::max();
  float g_f_shear_y_min = -std::numeric_limits<float>::max();
  float g_f_shear_y_max = std::numeric_limits<float>::max();

  float g_f_loadtime_scaling = 1.0;

  std::string positions_before_path; // Path to before point positions
  std::string positions_after_path;  // Path to after point positions
  std::string area_before_path;      // Path to before area file (unused)
  std::string area_after_path;       // Path to after area file (unused)
  std::string output_prefix;         // Path to output file
  std::string config_file_path;      // Path to the config file

  // Debugging options
  bool debugging_enabled;
  bool verbose_output;
};

// Global setting object
Settings settings;

// Rendering targets for the output of the program
cv::Mat imgFixed = cv::Mat::zeros(1024, 1024, CV_8UC3);

// String keys for finding the options in the configuration file.
const std::string config_key_xscale_minimum = "stretch_x_min";
const std::string config_key_xscale_maximum = "stretch_x_max";
const std::string config_key_yscale_minimum = "stretch_y_min";
const std::string config_key_yscale_maximum = "stretch_y_max";
const std::string config_key_xshear_minimum = "shear_x_min";
const std::string config_key_xshear_maximum = "shear_x_max";
const std::string config_key_yshear_minimum = "shear_y_min";
const std::string config_key_yshear_maximum = "shear_y_max";
const std::string config_key_scaling_factor = "loadtime_scaling";

// Read configuration file
bool readConfigFile(const std::string config_file_pathName, Configuration &config)
{
  std::string sKey;
  std::string sValue;
  std::ifstream f(config_file_pathName);

  if (f.is_open())
  {
    while (!f.eof())
    {
      f >> sKey >> sValue;
      f.ignore();
      // Read scaling/stretching config
      if (sKey == config_key_xscale_minimum)
      {
        config.g_f_stretch_x_min = atof(sValue.data());
      }
      if (sKey == config_key_xscale_maximum)
      {
        config.g_f_stretch_x_max = atof(sValue.data());
      }
      if (sKey == config_key_yscale_minimum)
      {
        config.g_f_stretch_y_min = atof(sValue.data());
      }
      if (sKey == config_key_yscale_maximum)
      {
        config.g_f_stretch_y_max = atof(sValue.data());
      }

      // Read shear config
      if (sKey == config_key_xshear_minimum)
      {
        config.g_f_shear_x_min = atof(sValue.data());
      }
      if (sKey == config_key_xshear_maximum)
      {
        config.g_f_shear_x_max = atof(sValue.data());
      }
      if (sKey == config_key_yshear_minimum)
      {
        config.g_f_shear_y_min = atof(sValue.data());
      }
      if (sKey == config_key_yshear_maximum)
      {
        config.g_f_shear_y_max = atof(sValue.data());
      }

      // Read load time scale config
      if (sKey == config_key_scaling_factor)
      {
        config.g_f_loadtime_scaling = atof(sValue.data());
      }
    }
    f.close();
    return true;
  }
  else
  {
    std::cout << "Can't open file: " << config_file_pathName << std::endl;
    return false;
  }
}

// Mouse control
void MouseCallBackFunc(int event, int x, int y, int flags, void *userdata)
{
  // Current and previous mouse pointer positions for distance calculations
  cv::Vec2f pntCur, pntBegin;
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    settings.g_bEventLButtonDown = true;
    pntCur[0] = x;
    pntCur[1] = y;
    pntBegin[0] = x;
    pntBegin[1] = y;
  }
  else if (event == cv::EVENT_LBUTTONUP)
  {
    settings.g_bEventLButtonDown = false;
  }
  else if (event == cv::EVENT_RBUTTONDOWN)
  {
    settings.g_bEventRButtonDown = true;
    pntCur[0] = x;
    pntCur[1] = y;
    pntBegin[0] = x;
    pntBegin[1] = y;
  }
  else if (event == cv::EVENT_RBUTTONUP)
  {
    settings.g_bEventRButtonDown = false;
  }
  else if (event == cv::EVENT_MOUSEMOVE)
  {
    pntCur[0] = x;
    pntCur[1] = y;
    if (settings.g_bEventLButtonDown == true)
    {
      settings.g_f_translate_x = pntCur[0] - pntBegin[0];
      settings.g_f_translate_y = pntCur[1] - pntBegin[1];
    }

    if (settings.g_bEventRButtonDown == true)
    {
      settings.g_f_stretch_x = settings.g_f_stretch_x + (pntCur[0] - pntBegin[0]) * 0.001;
      settings.g_f_stretch_y = settings.g_f_stretch_y + (pntCur[1] - pntBegin[1]) * 0.001;
    }
  }
  else if (event == cv::EVENT_LBUTTONDBLCLK)
  {
    settings.g_bDoCmaes = true;
  }
}

// Transformation calculation
void transformPointCloud(const PCl2D &pcIn, PCl2D &pcOut,
                         float translate_x, float translate_y,
                         float stretch_x, float stretch_y,
                         float shear_x, float shear_y)
{
  cv::Mat matTransfo = cv::Mat::eye(3, 3, CV_32F);
  // configure shear
  matTransfo.at<float>(0, 1) += shear_x;
  matTransfo.at<float>(1, 0) += shear_y;
  matTransfo.at<float>(0, 0) += shear_y * shear_x;

  // configure stretch
  matTransfo.at<float>(0, 0) *= stretch_x;
  matTransfo.at<float>(1, 0) *= stretch_x;

  matTransfo.at<float>(0, 1) *= stretch_y;
  matTransfo.at<float>(1, 1) *= stretch_y;

  // configure displacement
  matTransfo.at<float>(0, 2) = translate_x;
  matTransfo.at<float>(1, 2) = translate_y;

  cv::Mat pntIn = cv::Mat::ones(3, 1, CV_32F);
  cv::Mat pntOut;
  pcOut.resize(pcIn.size());
  for (int i = 0; i < pcIn.size(); ++i)
  {
    pntIn.at<float>(0, 0) = pcIn.at(i).x;
    pntIn.at<float>(1, 0) = pcIn.at(i).y;
    pntIn.at<float>(2, 0) = 1.0;

    pntOut = matTransfo * pntIn;
    pcOut.at(i).x = pntOut.at<float>(0, 0) / pntOut.at<float>(2, 0);
    pcOut.at(i).y = pntOut.at<float>(1, 0) / pntOut.at<float>(2, 0);
  }
}

// Calculation of cost function, tx,ty: translation; sx, sy: scaling
float costFunction(const PClPtr &sourcePtr, const PClPtr &targetPtr, float translate_x, float translate_y, float stretch_x, float stretch_y, float shear_x, float shear_y, const Configuration &config)
{
  // Range limitation. Output infinite cost if cell outside of range of scales
  if (stretch_x < config.g_f_stretch_x_min || stretch_x > config.g_f_stretch_x_max || stretch_y < config.g_f_stretch_y_min || stretch_y > config.g_f_stretch_y_max || shear_x < config.g_f_shear_x_min || shear_x > config.g_f_shear_x_max || shear_y < config.g_f_shear_y_min || shear_y > config.g_f_shear_y_max)
    return FLT_MAX;

  cv::Mat imgMerged = imgFixed.clone();

  PCl2D target_New;
  transformPointCloud(*targetPtr, target_New, translate_x, translate_y, stretch_x, stretch_y, shear_x, shear_y);
  for (int i = 0; i < target_New.size(); ++i)
  {
    cv::circle(imgMerged, cv::Point(target_New.at(i).x, target_New.at(i).y),
               3, cv::Scalar(0, 255, 255), -1);
  }

  pcl::registration::CorrespondenceEstimation<pcl::PointXY, pcl::PointXY> est;
  pcl::Correspondences all_correspondences;

  cv::imshow("Merge", imgMerged);
  char szKey = cv::waitKey(10);

  est.setInputCloud(sourcePtr);

  est.setInputTarget(targetPtr);

  // Determine all reciprocal correspondences
  est.determineReciprocalCorrespondences(all_correspondences);

  float err = 0;
  for (int i = 0; i < all_correspondences.size(); ++i)
  {
    err += all_correspondences.at(i).distance;
  }
  err /= (all_correspondences.size() + 1) *
         (float(all_correspondences.size()) / float(sourcePtr->size())) *
         (float(all_correspondences.size()) / float(targetPtr->size()));
  // err/=all_correspondences.size()+1; //not work for large cluster
  std::cout << all_correspondences.size() << "	" << err << "\n";
  return err;
}

// CMA-ES optimization
void doCMAES(const PClPtr &source, const PClPtr &target, float translate_x, float translate_y, float stretch_x, float stretch_y, float shear_x, float shear_y, const Configuration &config)
{
  CMAES<float> evo;  // the optimizer
  float *const *pop; // sampled population
  float *fitvals;    // objective function values of sampled population
  float fbestever = 10000;
  float *xbestever = NULL; // store best solution
  float fmean;
  int irun;
  int lambda = 50;    // offspring population size, 0 invokes default
  int countevals = 0; // used to set for restarts

  // for (irun = 0; irun < nrestarts+1; ++irun)

  /* Parameters can be set in two ways. Here as input parameter to evo.init()
          and as value read from signals.par by calling evo.readSignals
          explicitely.
    */
  const int dim = 6;
  float xstart[dim];
  xstart[0] = translate_x;
  xstart[1] = translate_y;
  xstart[2] = stretch_x;
  xstart[3] = stretch_y;
  xstart[4] = shear_x;
  xstart[5] = shear_y;

  float lb[dim];
  lb[0] = xstart[0] - 20;
  lb[1] = xstart[1] - 20;
  lb[2] = (config.g_f_stretch_x_min - 1.) * 100;
  lb[3] = (config.g_f_stretch_y_min - 1.) * 100;
  lb[4] = (config.g_f_shear_x_min) * 100;
  lb[5] = (config.g_f_shear_y_min) * 100;

  float ub[dim];
  ub[0] = xstart[0] + 20;
  ub[1] = xstart[1] + 20;
  ub[2] = (config.g_f_stretch_y_max - 1.) * 100;
  ub[3] = (config.g_f_stretch_y_max - 1.) * 100;
  ub[4] = (config.g_f_shear_x_max) * 100;
  ub[5] = (config.g_f_shear_y_max) * 100;

  float stddev[dim];
  stddev[0] = 2;
  stddev[1] = 2;
  stddev[2] = std::min(5., (ub[2] - lb[2]) / 3.);
  stddev[3] = std::min(5., (ub[3] - lb[3]) / 3.);
  stddev[4] = std::min(5., (ub[4] - lb[4]) / 3.);
  stddev[5] = std::min(5., (ub[5] - lb[5]) / 3.);

  Parameters<float> parameters;
  // You can resume a previous run by specifying a file that contains the
  // resume data:
  // parameters.resumefile = "resumeevo2.dat";
  parameters.logWarnings = true; // warnings will be printed on std::cerr
  parameters.stopTolX = 1e-2;
  parameters.stopTolFun = 0.01;
  parameters.updateCmode.maxtime = 1.0;
  parameters.lambda = lambda;
  parameters.stopMaxIter = 30;
  parameters.init(dim, xstart, stddev);

  fitvals = evo.init(parameters); // allocs fitvals
  std::cout << evo.sayHello() << std::endl;
  evo.countevals = countevals; // a hack, effects the output and termination

  while (!evo.testForTermination())
  {
    // Generate population of new candidate solutions
    pop = evo.samplePopulation(); // do not change content of pop

    /* Here optionally handle constraints etc. on pop. You may
     * call evo.reSampleSingle(i) to resample the i-th
     * vector pop[i], see below.  Do not change pop in any other
     * way. You may also copy and modify (repair) pop[i] only
     * for the evaluation of the fitness function and consider
     * adding a penalty depending on the size of the
     * modification.
     */

    // Compute fitness value for each candidate solution
    for (int i = 0; i < evo.get(CMAES<float>::PopSize); ++i)
    {
      /* You may resample the solution i until it lies within the
              feasible domain here, e.g. until it satisfies given
              box constraints (variable boundaries). The function
              is_feasible() needs to be user-defined.
              Assumptions: the feasible domain is convex, the optimum
              is not on (or very close to) the domain boundary,
              initialX is feasible (or in case typicalX +-
         2*initialStandardDeviations is feasible) and initialStandardDeviations
         is (are) sufficiently small to prevent quasi-infinite looping.
        */
      /* while (!is_feasible(pop[i]))
                evo.reSampleSingle(i);
      */
      fitvals[i] = costFunction(source, target, pop[i][0], pop[i][1], 1.0 + pop[i][2] * 0.01,
                                1.0 + pop[i][3] * 0.01, pop[i][4] * 0.01, pop[i][5] * 0.01, config);
    }
    // update search distribution
    evo.updateDistribution(fitvals);
  }

  // keep best ever solution
  // if (irun == 0 || evo.get(CMAES<float>::FBestEver) < fbestever)
  if (evo.get(CMAES<float>::FBestEver) < fbestever)
  {
    fbestever = evo.get(CMAES<float>::FBestEver);
    xbestever = evo.getInto(CMAES<float>::XBestEver,
                            xbestever); // allocates memory if needed
  }

  settings.g_f_translate_x = xbestever[0];
  settings.g_f_translate_y = xbestever[1];
  settings.g_f_stretch_x = xbestever[2] * 0.01 + 1.0;
  settings.g_f_stretch_y = xbestever[3] * 0.01 + 1.0;
  settings.g_f_shear_x = xbestever[4] * 0.01;
  settings.g_f_shear_y = xbestever[5] * 0.01;

  std::cout << "translate: [" << settings.g_f_translate_x << "|" << settings.g_f_translate_y << "]" << std::endl;
  std::cout << "stretch: [" << settings.g_f_stretch_x << "|" << settings.g_f_stretch_y << "]" << std::endl;
  std::cout << "shear: [" << settings.g_f_shear_x << "|" << settings.g_f_shear_y << "]" << std::endl;
}

bool replace(std::string &str, const std::string &from, const std::string &to)
{
  size_t start_pos = str.find(from);
  if (start_pos == std::string::npos)
    return false;
  str.replace(start_pos, from.length(), to);
  return true;
}

int main(int argc, char **argv)
{

  cxxopts::Options options(
      argv[0],
      "Iterative optimizer to find the best match between points before and "
      "after a linear transformation (stretch/shear).");
  try
  {

    options.add_options() // Initialize available options
        ("input_before", "Path of the input file of the before state.",
         cxxopts::value<std::string>()) // Input file before
        ("input_after", "Path of the input file of the after state.",
         cxxopts::value<std::string>()) // Input file before
        ("output_prefix", "Path prefix to put the output files under.",
         cxxopts::value<std::string>()->default_value(
             "./output_")) // Output file path prefix
        ("area_before",
         "Path of the input file of the before area assignments (currently "
         "unused).",
         cxxopts::value<std::string>()) // Input file area before
        ("area_after",
         "Path of the input file of the after area assignments (currently "
         "unused).",
         cxxopts::value<std::string>()) // Input file area after
        ("c,config_file", "Path of the configuration file to be used.",
         cxxopts::value<std::string>()) // Configuration file
        ("v,verbose", "Enable verbose output.",
         cxxopts::value<bool>()->default_value("false")->implicit_value(
             "true")) // Flag to enable verbose output
        ("d,debug", "Enable debugging.",
         cxxopts::value<bool>()->default_value("false")->implicit_value(
             "true"))                        // a bool parameter to enable debugging
        ("h,help", "Print usage.")           // allow help to display
        ("version", "Display version info.") // display version info below
        ;

    // Make input and output positional arguments
    options.parse_positional({"input_before", "input_after", "output_prefix"});
  }
  catch (const cxxopts::exceptions::specification &e)
  {
    // Error handling if something about the definition failed
    std::cerr << "ERROR:\t" << e.what() << std::endl;
    exit(EXIT_FAILURE);
  }

  Configuration config{};

  // Default settings for min, max of directional saling
  config.g_f_stretch_x_min = -std::numeric_limits<float>::max();
  config.g_f_stretch_x_max = std::numeric_limits<float>::max();
  config.g_f_stretch_y_min = -std::numeric_limits<float>::max();
  config.g_f_stretch_y_max = std::numeric_limits<float>::max();

  // default settings for shear
  config.g_f_shear_x_min = -std::numeric_limits<float>::max();
  config.g_f_shear_x_max = std::numeric_limits<float>::max();
  config.g_f_shear_y_min = -std::numeric_limits<float>::max();
  config.g_f_shear_y_max = std::numeric_limits<float>::max();

  // default setting for scaling during input
  config.g_f_loadtime_scaling = 1.0;

  // Debugging options
  config.debugging_enabled = false;
  config.verbose_output = false;

  bool options_error = false;

  try
  {
    auto result = options.parse(argc, argv);

    if (result.count("help"))
    {
      std::cout << options.help() << std::endl;
      exit(EXIT_SUCCESS);
    }

    if (result.count("version"))
    {
      std::cout << "\t" << argv[0] << std::endl;
      std::cout << "Version:" << std::endl;
      std::cout << "OS:" << std::endl;
      std::cout << "Built on:" << std::endl;
      std::cout << "Git branch:" << std::endl;
      std::cout << "Git commit:" << std::endl;

      exit(EXIT_SUCCESS);
    }

    if (result.count("input_before"))
    {
      config.positions_before_path = result["input_before"].as<std::string>();
    }
    else
    {
      options_error = true;
      std::cerr << "ERROR:\t Missing input file of before point positions "
                << std::endl;
    }
    if (result.count("input_after"))
    {
      config.positions_after_path = result["input_after"].as<std::string>();
    }
    else
    {
      options_error = true;
      std::cerr << "ERROR:\t Missing input file of after point positions "
                << std::endl;
    }
    if (result.count("output_prefix"))
    {
      config.output_prefix = result["output_prefix"].as<std::string>();
    }
    else
    {
      options_error = true;
      std::cerr << "ERROR:\t Missing output path prefix" << std::endl;
    }
    if (result.count("config_file"))
    {
      config.config_file_path = result["config_file"].as<std::string>();
      if (config.config_file_path.empty())
      {
        config.config_file_path = "./conf/Config.ini";
      }
    }
    else
    {
      config.config_file_path = "./conf/Config.ini";
    }

    if (result.count("debug"))
      config.debugging_enabled = result["debug"].as<bool>();
    if (result.count("verbose"))
      config.verbose_output = result["verbose"].as<bool>();
  }
  catch (const cxxopts::exceptions::parsing &e)
  {
    std::cerr << "ERROR:\t" << e.what() << std::endl;
    exit(EXIT_FAILURE);
  }
  if (options_error)
  {
    std::cerr << "There have been errors while processing the program's input "
                 "options. Please use option -h or --help to display the help."
              << std::endl;
    exit(EXIT_FAILURE);
  }

  // TODO: Make settings local. It should not be a global variable

  settings.g_bDoCmaes = false;
  settings.g_f_translate_x = 0;
  settings.g_f_translate_y = 0;
  settings.g_f_stretch_x = 1.0;
  settings.g_f_stretch_y = 1.0;
  settings.g_f_shear_x = 0.0;
  settings.g_f_shear_y = 0.0;

  readConfigFile(config.config_file_path, config);

  cv::namedWindow("Merge");
  cv::setMouseCallback("Merge", MouseCallBackFunc, NULL);
  std::ifstream infileSrc(config.positions_before_path);
  std::ifstream infileTar(config.positions_after_path);
  // TODO: Fully remove if actually unused
  // std::ifstream inAreaFileSrc(area_before_path); // unused
  // std::ifstream inAreaFileTar(area_after_path); // unused

  // pointers initialized with actually allocated data to prevent crashes on cleanup
  PClPtr sourcePtr = std::make_shared<PCl2D>();
  PClPtr targetPtr = std::make_shared<PCl2D>();

  float x, y;
  // read and plot the srouce point cloud
  while (infileSrc >> x >> y)
  {
    // x *= 0.2;
    // y *= 0.2;
    x *= config.g_f_loadtime_scaling;
    y *= config.g_f_loadtime_scaling;
    pcl::PointXY pnt;
    pnt.x = x;
    pnt.y = y;

    // Plot the points as rectangles on the screen
    // cv::circle(imgFixed, cv::Point(x,y), 3, cv::Scalar(0,255,0),-1); //1
    cv::rectangle(imgFixed, cv::Point(x - 2, y - 2), cv::Point(x + 2, y + 2),
                  cv::Scalar(255, 100, 0), -1);
    sourcePtr->push_back(pnt);
  }
  cv::Mat imgMerged = imgFixed.clone();

  // read and plot the target point cloud
  while (infileTar >> x >> y)
  {
    // x *= 0.2;
    // y *= 0.2;
    x *= config.g_f_loadtime_scaling;
    y *= config.g_f_loadtime_scaling;
    pcl::PointXY pnt;
    pnt.x = x;
    pnt.y = y;
    targetPtr->push_back(pnt);
    cv::circle(imgMerged, cv::Point(x, y), 3, cv::Scalar(0, 255, 255), -1);
  }

  std::cout << "Press 'ESC' to quit" << std::endl;

  while (true)
  {
    PCl2D target_tmp;
    imgMerged = imgFixed.clone();
    transformPointCloud(*targetPtr, target_tmp, settings.g_f_translate_x, settings.g_f_translate_y, settings.g_f_stretch_x, settings.g_f_stretch_y, settings.g_f_shear_x, settings.g_f_shear_y);
    for (int i = 0; i < target_tmp.size(); ++i)
    {
      cv::circle(imgMerged, cv::Point(target_tmp.at(i).x, target_tmp.at(i).y),
                 3, cv::Scalar(0, 255, 255), -1);
    }

    cv::imshow("Merge", imgMerged);
    char szKey = cv::waitKey(10);
    // stop when escape is pressed
    if (27 == szKey)
      break;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    pcl::registration::CorrespondenceEstimation<pcl::PointXY, pcl::PointXY> est;
    pcl::Correspondences all_correspondences;

    // ... read or fill in source and target
    est.setInputCloud(sourcePtr);
    est.setInputTarget(targetPtr);

    // Determine all reciprocal correspondences
    est.determineReciprocalCorrespondences(all_correspondences);
    float err = 0;
    for (int i = 0; i < all_correspondences.size(); ++i)
    {
      err += all_correspondences.at(i).distance;
    }
    err /= all_correspondences.size() + 1;

    std::cout << all_correspondences.size() << "	" << err << "\n";

    // Optimize the transformation based on current correspondences
    if (settings.g_bDoCmaes == true)
    {
      doCMAES(sourcePtr, targetPtr, settings.g_f_translate_x, settings.g_f_translate_y, (settings.g_f_stretch_x - 1.0) * 100.0, (settings.g_f_stretch_y - 1.0) * 100.0, settings.g_f_shear_x * 100.0, settings.g_f_shear_y * 100.0, config);
      settings.g_bDoCmaes = false;

      // Output the new correspondences
      std::ofstream oo;
      oo.open(config.output_prefix + "_mapping.txt");
      est.setInputCloud(sourcePtr);
      est.setInputTarget(targetPtr);

      // Determine all reciprocal correspondences
      est.determineReciprocalCorrespondences(all_correspondences);

      float err = 0;
      for (int i = 0; i < all_correspondences.size(); ++i)
      {
        oo << all_correspondences.at(i).index_query << "	"
           << all_correspondences.at(i).index_match << "\n";
      }
      oo.flush();
      oo.close();

      // Output all info for the correspondences
      std::string sOutAll = config.output_prefix + "_all.txt";
      std::ofstream ooAll;
      ooAll.open(sOutAll);
      for (int i = 0; i < all_correspondences.size(); ++i)
      {
        pcl::PointXY pntSource;
        pntSource = sourcePtr->points.at(all_correspondences.at(i).index_query);
        pcl::PointXY pntTargetNew;
        pntTargetNew =
            target_tmp.points.at(all_correspondences.at(i).index_match);
        // ooAll<<all_correspondences.at(i).index_query<<" "<< pntSource.x/0.2<<
        // " " << pntSource.y/0.2 << " "  <<
        // all_correspondences.at(i).index_match << " " << pntTargetNew.x/0.2 <<
        // " " << pntTargetNew.y/0.2<<"\n";
        ooAll << all_correspondences.at(i).index_query << " "
              << pntSource.x / config.g_f_loadtime_scaling << " " << pntSource.y / config.g_f_loadtime_scaling
              << " " << all_correspondences.at(i).index_match << " "
              << pntTargetNew.x / config.g_f_loadtime_scaling << " "
              << pntTargetNew.y / config.g_f_loadtime_scaling << "\n";
      }
      ooAll.flush();
      ooAll.close();
      cv::waitKey(-1);
    }
  }

  return (0);
}