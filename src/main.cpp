#include "cmaes.h"
#include "parameters.h"
#include "version.hpp"

#include "match_points.hpp"
#include <cxxopts.hpp>
#include <fstream>
#include <git.h>
#include <iostream>
#include <limits>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/icp.h>

// 2d Point cloud datatype
typedef pcl::PointCloud<pcl::PointXY> PCl2D;
// Shared pointer to a 2d pointcloud
typedef PCl2D::Ptr PClPtr;

const cv::Scalar before_color = cv::Scalar(255, 100, 0);
const cv::Scalar after_color = cv::Scalar(0, 255, 255);
const cv::Scalar lowlight_color = cv::Scalar(100, 100, 100);
const cv::Scalar highlight_color = cv::Scalar(0, 0, 255);
const cv::Scalar selected_before_color = cv::Scalar(255, 255, 0);
const cv::Scalar selected_after_color = cv::Scalar(0, 255, 100);

float max_match_dist = 0.0;

struct KDTreeLabel {
  size_t before_index;
};

// Runtime-modified settings of the program
struct Settings {
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

  // Keep track of potential mapping information
  bool has_matching = false;
  int64_t match_index_before = -1;
  int64_t match_index_after = -1;
  // Reference points for before and after configurations
  struct {
    float x, y;
  } ref_before;
  struct {
    float x, y;
  } ref_after;
};

struct TransformSettings {
  // Current streching settings
  struct {
    float x, y;
  } stretch;

  // Current shear settings
  struct {
    float x, y;
  } shear;

  // Specify two points in the before and after clouds that should correspond
  struct {
    float x, y;
  } target_before;
  struct {
    float x, y;
  } target_after;
};

// Configuration options loaded at start
struct Configuration {
  float g_f_stretch_x_min = -std::numeric_limits<float>::max();
  float g_f_stretch_x_max = std::numeric_limits<float>::max();
  float g_f_stretch_y_min = -std::numeric_limits<float>::max();
  float g_f_stretch_y_max = std::numeric_limits<float>::max();

  float g_f_shear_x_min = -std::numeric_limits<float>::max();
  float g_f_shear_x_max = std::numeric_limits<float>::max();
  float g_f_shear_y_min = -std::numeric_limits<float>::max();
  float g_f_shear_y_max = std::numeric_limits<float>::max();

  float g_f_loadtime_scaling = 1.0;

  float max_match_distance_scale = 1.0;

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

const size_t img_size_x = 1024;
const size_t img_size_y = 1024;

// Rendering targets for the output of the program
cv::Mat imgFixed;
pcl::PointXY min_plot, max_plot;

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

// Mouse control reference points
cv::Vec2f mouse_control_point_beginf;

// Deal with selecting corresponding cells
bool isSelectMode = false;
int currSelectStep = 0;
std::vector<int> indices(2, -1);
std::vector<pcl::PointXY> image_space_pos_before;
std::vector<pcl::PointXY> image_space_pos_after;

// Read configuration file
bool readConfigFile(const std::string config_file_pathName,
                    Configuration &config) {
  std::string sKey;
  std::string sValue;
  std::ifstream f(config_file_pathName);

  if (f.is_open()) {
    while (!f.eof()) {
      f >> sKey >> sValue;
      f.ignore();
      std::cerr << "Debug config: key(" << sKey << ") -> val(" << sValue << ")"
                << std::endl;
      // Read scaling/stretching config
      if (sKey == config_key_xscale_minimum) {
        config.g_f_stretch_x_min = atof(sValue.data());
      }
      if (sKey == config_key_xscale_maximum) {
        config.g_f_stretch_x_max = atof(sValue.data());
      }
      if (sKey == config_key_yscale_minimum) {
        config.g_f_stretch_y_min = atof(sValue.data());
      }
      if (sKey == config_key_yscale_maximum) {
        config.g_f_stretch_y_max = atof(sValue.data());
      }

      // Read shear config
      if (sKey == config_key_xshear_minimum) {
        config.g_f_shear_x_min = atof(sValue.data());
      }
      if (sKey == config_key_xshear_maximum) {
        config.g_f_shear_x_max = atof(sValue.data());
      }
      if (sKey == config_key_yshear_minimum) {
        config.g_f_shear_y_min = atof(sValue.data());
      }
      if (sKey == config_key_yshear_maximum) {
        config.g_f_shear_y_max = atof(sValue.data());
      }

      // Read load time scale config
      if (sKey == config_key_scaling_factor) {
        config.g_f_loadtime_scaling = atof(sValue.data());
      }
    }
    f.close();
    return true;
  } else {
    std::cout << "Can't open file: " << config_file_pathName << std::endl;
    return false;
  }
}

// Mouse control
void MouseCallBackFunc(int event, int x, int y, int flags, void *userdata) {
  // TODO: Fix issue with translation and redrawing. Probably need to rescale
  // the translate offset in mouse callback FIX: Still issue with scale of
  // translation and scaling based on the mouse callback

  // Current and previous mouse pointer positions for distance calculations
  cv::Vec2f pntCur;
  if (event == cv::EVENT_LBUTTONDOWN) {
    settings.g_bEventLButtonDown = true;
    pntCur[0] = x;
    pntCur[1] = y;
    mouse_control_point_beginf[0] = x;
    mouse_control_point_beginf[1] = y;
  } else if (event == cv::EVENT_LBUTTONUP) {
    settings.g_bEventLButtonDown = false;
  } else if (event == cv::EVENT_RBUTTONDOWN) {
    settings.g_bEventRButtonDown = true;
    pntCur[0] = x;
    pntCur[1] = y;
    mouse_control_point_beginf[0] = x;
    mouse_control_point_beginf[1] = y;
  } else if (event == cv::EVENT_RBUTTONUP) {
    settings.g_bEventRButtonDown = false;
  } else if (event == cv::EVENT_MOUSEMOVE) {
    pntCur[0] = x;
    pntCur[1] = y;
    if (settings.g_bEventLButtonDown == true) {
      float plot_range_x = max_plot.x - min_plot.x;
      float plot_range_y = max_plot.y - min_plot.y;
      settings.g_f_translate_x = (pntCur[0] - mouse_control_point_beginf[0]) /
                                 img_size_x * plot_range_x;
      settings.g_f_translate_y = (pntCur[1] - mouse_control_point_beginf[1]) /
                                 img_size_y * plot_range_y;
    }

    if (settings.g_bEventRButtonDown == true) {
      settings.g_f_stretch_x =
          settings.g_f_stretch_x +
          (pntCur[0] - mouse_control_point_beginf[0]) * 0.001;
      settings.g_f_stretch_y =
          settings.g_f_stretch_y +
          (pntCur[1] - mouse_control_point_beginf[1]) * 0.001;
    }
  } else if (event == cv::EVENT_LBUTTONDBLCLK) {
    settings.g_bDoCmaes = true;
  }
}

int find_closest_match(const std::vector<pcl::PointXY> &ref,
                       const pcl::PointXY &chosen) {
  float min_dist_sq = std::numeric_limits<float>::max();
  int chosen_index = -1;
  for (int i = 0; i < ref.size(); i++) {
    float dx = (ref[i].x - chosen.x);
    float dy = (ref[i].y - chosen.y);
    float curr_dist_sq = dx * dx + dy * dy;

    if (curr_dist_sq < min_dist_sq) {
      chosen_index = i;
      min_dist_sq = curr_dist_sq;
    }
  }
  return chosen_index;
}

// Mouse control for mapping selection
void MouseCallbackFuncMapping(int event, int x, int y, int flags,
                              void *userdata) {
  pcl::PointXY pntCur;
  if (event == cv::EVENT_LBUTTONUP) {
    pntCur.x = x;
    pntCur.y = y;

    if (currSelectStep == 0) {
      indices[currSelectStep] =
          find_closest_match(image_space_pos_before, pntCur);
      std::cerr << "Before index:" << indices[currSelectStep] << std::endl;
    } else if (currSelectStep == 1) {
      indices[currSelectStep] =
          find_closest_match(image_space_pos_after, pntCur);
      std::cerr << "After index:" << indices[currSelectStep] << std::endl;
    }
    currSelectStep++;
  }
}

// Transformation calculation
void transformPointCloud(const PCl2D &pcIn, PCl2D &pcOut,
                         const TransformSettings &trans_settings) {
  // Implements the shear and stretch steps
  cv::Mat shear_stretch = cv::Mat::eye(3, 3, CV_32F);

  // Shifts the after image to a common reference frame
  cv::Mat shift_after_to_common = cv::Mat::eye(3, 3, CV_32F);
  // Shifts the common reference frame into the before image frame
  cv::Mat shift_common_to_before = cv::Mat::eye(3, 3, CV_32F);
  // configure shear
  shear_stretch.at<float>(0, 1) += trans_settings.shear.x;
  shear_stretch.at<float>(1, 0) += trans_settings.shear.y;
  shear_stretch.at<float>(0, 0) +=
      trans_settings.shear.y * trans_settings.shear.x;

  // configure stretch
  shear_stretch.at<float>(0, 0) *= trans_settings.stretch.x;
  shear_stretch.at<float>(1, 0) *= trans_settings.stretch.y;

  shear_stretch.at<float>(0, 1) *= trans_settings.stretch.y;
  shear_stretch.at<float>(1, 1) *= trans_settings.stretch.y;

  // configure displacement steps
  shift_after_to_common.at<float>(0, 2) = -trans_settings.target_after.x;
  shift_after_to_common.at<float>(1, 2) = -trans_settings.target_after.y;

  shift_common_to_before.at<float>(0, 2) = trans_settings.target_before.x;
  shift_common_to_before.at<float>(1, 2) = trans_settings.target_before.y;

  cv::Mat combined_transformation =
      shift_common_to_before * shear_stretch * shift_after_to_common;

  cv::Mat pntIn = cv::Mat::ones(3, 1, CV_32F);
  cv::Mat pntOut;

  pcOut.resize(pcIn.size());
  for (int i = 0; i < pcIn.size(); ++i) {
    pntIn.at<float>(0, 0) = pcIn.at(i).x;
    pntIn.at<float>(1, 0) = pcIn.at(i).y;
    pntIn.at<float>(2, 0) = 1.0;

    pntOut = combined_transformation * pntIn;
    pcOut.at(i).x = pntOut.at<float>(0, 0) / pntOut.at<float>(2, 0);
    pcOut.at(i).y = pntOut.at<float>(1, 0) / pntOut.at<float>(2, 0);
  }
}

// Calculation of cost function, tx,ty: translation; sx, sy: scaling
float costFunction(const PClPtr &sourcePtr, const PClPtr &targetPtr,
                   const TransformSettings &trans_settings,
                   const Configuration &config) {
  // Range limitation. Output infinite cost if transformation outside of range
  // of scales
  if (trans_settings.stretch.x < config.g_f_stretch_x_min ||
      trans_settings.stretch.x > config.g_f_stretch_x_max ||
      trans_settings.stretch.y < config.g_f_stretch_y_min ||
      trans_settings.stretch.y > config.g_f_stretch_y_max ||
      trans_settings.shear.x < config.g_f_shear_x_min ||
      trans_settings.shear.x > config.g_f_shear_x_max ||
      trans_settings.shear.y < config.g_f_shear_y_min ||
      trans_settings.shear.y > config.g_f_shear_y_max)
    return FLT_MAX;

  cv::Mat imgMerged = imgFixed.clone();

  float plot_range_x = max_plot.x - min_plot.x;
  float plot_range_y = max_plot.y - min_plot.y;

  PClPtr target_New = std::make_shared<PCl2D>();

  transformPointCloud(*targetPtr, *target_New, trans_settings);
  for (int i = 0; i < target_New->size(); ++i) {
    cv::circle(
        imgMerged,
        cv::Point(
            (target_New->at(i).x - min_plot.x) / plot_range_x * img_size_x,
            (target_New->at(i).y - min_plot.y) / plot_range_y * img_size_y),
        3, after_color, -1);
  }

  pcl::registration::CorrespondenceEstimation<pcl::PointXY, pcl::PointXY> est;
  pcl::Correspondences all_correspondences;

  cv::imshow("Merge", imgMerged);
  char szKey = cv::waitKey(10);

  est.setInputCloud(sourcePtr);
  est.setInputTarget(target_New);

  std::cerr << "Matching with max dist:" << max_match_dist << std::endl;

  // Determine all reciprocal correspondences
  est.determineReciprocalCorrespondences(all_correspondences, max_match_dist);

  float err = 0;
  for (int i = 0; i < all_correspondences.size(); ++i) {
    err += all_correspondences.at(i).distance;
  }
  err /= (all_correspondences.size() + 1) *
         (float(all_correspondences.size()) / float(sourcePtr->size())) *
         (float(all_correspondences.size()) / float(targetPtr->size()));
  // err/=all_correspondences.size()+1; //not work for large cluster
  std::cout << "Cost: Matches=" << all_correspondences.size()
            << "[b:" << sourcePtr->size() << "|a:" << targetPtr->size() << "]"
            << "	Error:" << err << "\n";
  return err;
}

// CMA-ES optimization
void doCMAES(const PClPtr &source, const PClPtr &target,
             const Settings &trafo_settings, const Configuration &config) {
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
  const int dim = 4;
  float xstart[dim];
  xstart[0] = (trafo_settings.g_f_stretch_x - 1.) * 100;
  xstart[1] = (trafo_settings.g_f_stretch_y - 1.) * 100;
  xstart[2] = trafo_settings.g_f_shear_x * 100;
  xstart[3] = trafo_settings.g_f_shear_y * 100;

  float lb[dim];
  lb[0] = (config.g_f_stretch_x_min - 1.) * 100;
  lb[1] = (config.g_f_stretch_y_min - 1.) * 100;
  lb[2] = (config.g_f_shear_x_min) * 100;
  lb[3] = (config.g_f_shear_y_min) * 100;

  float ub[dim];
  ub[0] = (config.g_f_stretch_y_max - 1.) * 100;
  ub[1] = (config.g_f_stretch_y_max - 1.) * 100;
  ub[2] = (config.g_f_shear_x_max) * 100;
  ub[3] = (config.g_f_shear_y_max) * 100;

  float stddev[dim];
  stddev[0] = std::min(5., (ub[0] - lb[0]) / 3.);
  stddev[1] = std::min(5., (ub[1] - lb[1]) / 3.);
  stddev[2] = std::min(5., (ub[2] - lb[2]) / 3.);
  stddev[3] = std::min(5., (ub[3] - lb[3]) / 3.);

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

  while (!evo.testForTermination()) {
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
    for (int i = 0; i < evo.get(CMAES<float>::PopSize); ++i) {
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
      TransformSettings transform_settings{};

      if (trafo_settings.has_matching) {
        transform_settings.target_before.x =
            source->at(trafo_settings.match_index_before).x;
        transform_settings.target_before.y =
            source->at(trafo_settings.match_index_before).y;
        transform_settings.target_after.x =
            target->at(trafo_settings.match_index_after).x;
        transform_settings.target_after.y =
            target->at(trafo_settings.match_index_after).y;
      }
      transform_settings.stretch.x = 1.0 + pop[i][0] * 0.01;
      transform_settings.stretch.y = 1.0 + pop[i][1] * 0.01;
      transform_settings.shear.x = pop[i][2] * 0.01;
      transform_settings.shear.y = pop[i][3] * 0.01;

      fitvals[i] = costFunction(source, target, transform_settings, config);
    }
    // update search distribution
    evo.updateDistribution(fitvals);
  }

  // keep best ever solution
  // if (irun == 0 || evo.get(CMAES<float>::FBestEver) < fbestever)
  if (evo.get(CMAES<float>::FBestEver) < fbestever) {
    fbestever = evo.get(CMAES<float>::FBestEver);
    xbestever = evo.getInto(CMAES<float>::XBestEver,
                            xbestever); // allocates memory if needed
  }

  settings.g_f_stretch_x = xbestever[0] * 0.01 + 1.0;
  settings.g_f_stretch_y = xbestever[1] * 0.01 + 1.0;
  settings.g_f_shear_x = xbestever[2] * 0.01;
  settings.g_f_shear_y = xbestever[3] * 0.01;

  std::cout << "translate: [" << settings.g_f_translate_x << "|"
            << settings.g_f_translate_y << "]" << std::endl;
  std::cout << "stretch: [" << settings.g_f_stretch_x << "|"
            << settings.g_f_stretch_y << "]" << std::endl;
  std::cout << "shear lim x: " << config.g_f_shear_x_min << "<"
            << settings.g_f_shear_x << " < " << config.g_f_shear_x_max
            << std::endl;
  std::cout << "shear lim y: " << config.g_f_shear_y_min << "<"
            << settings.g_f_shear_y << " < " << config.g_f_shear_y_max
            << std::endl;
  std::cout << "shear: [" << settings.g_f_shear_x << "|" << settings.g_f_shear_y
            << "]" << std::endl;
}

bool replace(std::string &str, const std::string &from, const std::string &to) {
  size_t start_pos = str.find(from);
  if (start_pos == std::string::npos)
    return false;
  str.replace(start_pos, from.length(), to);
  return true;
}

void print_version_info(std::ostream &stream,
                        const std::string &line_prefix = "") {
  stream << line_prefix << "VERSION INFORMATION" << std::endl;
  stream << line_prefix << "Program version: " << icp::get_version()
         << std::endl;
  if (git::IsPopulated()) {
    stream << line_prefix << "GIT Branch:" << git::Branch() << std::endl;
    stream << line_prefix << "GIT Commit:" << git::CommitSHA1() << std::endl;
    stream << line_prefix << "GIT Has uncommitted changes:"
           << (git::AnyUncommittedChanges() ? "yes" : "no") << std::endl;
    stream << line_prefix << "GIT Commit date:" << git::CommitDate()
           << std::endl;
  }
}

std::vector<pcl::PointXY> to_image_space(const PClPtr &cloudptr,
                                         const Settings &image_settings) {
  std::vector<pcl::PointXY> res(cloudptr->size());
  float plot_range_x = max_plot.x - min_plot.x;
  float plot_range_y = max_plot.y - min_plot.y;

  for (size_t i = 0; i < cloudptr->size(); i++) {
    // Plot the points as rectangles on the screen
    // cv::circle(imgFixed, cv::Point(x,y), 3, cv::Scalar(0,255,0),-1); //1
    res[i].x = (cloudptr->at(i).x - min_plot.x) / plot_range_x * img_size_x;
    res[i].y = (cloudptr->at(i).y - min_plot.y) / plot_range_y * img_size_y;
  }

  return res;
}

void input_mapping(const PClPtr &source, const PClPtr &target,
                   Settings &map_settings) {
  isSelectMode = true;
  currSelectStep = 0;

  indices.assign(2, -1);

  image_space_pos_before = to_image_space(source, map_settings);
  image_space_pos_after = to_image_space(target, map_settings);

  cv::namedWindow("Mapping");
  cv::setMouseCallback("Mapping", MouseCallbackFuncMapping, NULL);

  cv::Scalar colors[2][2] = {{highlight_color, lowlight_color},
                             {lowlight_color, highlight_color}};

  while (currSelectStep < indices.size()) {
    cv::Mat img_mapping = cv::Mat::zeros(img_size_x, img_size_y, CV_8UC3);

    // Plot before and after images
    for (size_t i = 0; i < image_space_pos_before.size(); i++) {
      cv::Scalar rendercolor =
          (currSelectStep > 0 ? (i == indices[0] ? selected_before_color
                                                 : colors[currSelectStep][0])
                              : colors[currSelectStep][0]);
      cv::rectangle(img_mapping,
                    cv::Point(image_space_pos_before[i].x - 2,
                              image_space_pos_before[i].y - 2),
                    cv::Point(image_space_pos_before[i].x + 2,
                              image_space_pos_before[i].y + 2),
                    rendercolor, -1);
    }
    for (size_t i = 0; i < image_space_pos_before.size(); i++) {
      cv::circle(
          img_mapping,
          cv::Point(image_space_pos_after[i].x, image_space_pos_after[i].y), 3,
          colors[currSelectStep][1], -1);
    }

    cv::imshow("Mapping", img_mapping);
    char szKey = cv::waitKey(100);
    // stop when escape is pressed
    if (27 == szKey) {
      break;
    }
    // Check if the window was closed
    if (!cv::getWindowProperty("Mapping", cv::WND_PROP_VISIBLE)) {
      break;
    }
  }
  isSelectMode = false;
  image_space_pos_before.clear();
  image_space_pos_after.clear();

  if (cv::getWindowProperty("Mapping", cv::WND_PROP_VISIBLE)) {
    cv::destroyWindow("Mapping");
  }

  if (currSelectStep >= 2) {
    settings.has_matching = true;
    settings.match_index_before = indices[0];
    settings.match_index_after = indices[1];
  } else {
    settings.has_matching = false;
    settings.match_index_before = -1;
    settings.match_index_after = -1;
  }
}

int main(int argc, char **argv) {

  cxxopts::Options options(
      argv[0],
      "Iterative optimizer to find the best match between points before and "
      "after a linear transformation (stretch/shear).");
  try {

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
             "true"))              // a bool parameter to enable debugging
        ("l,cutoff_scale", "Scaling factor to scale the maximum distance for points to be considered possible matches. Is multiplied to a scale factor calculated from the median distance between closest neighbors in the before image. Default: 1.0.",
         cxxopts::value<float>()->default_value("1.0")) // Parameter to set the scale for the maximum cutoff
      
        ("h,help", "Print usage.") // allow help to display
        ("version", "Display version info.") // display version info below
        ;

    // Make input and output positional arguments
    options.parse_positional({"input_before", "input_after", "output_prefix"});

    options.positional_help("<input_before> <input_after> <output_prefix>");
    options.show_positional_help();
  } catch (const cxxopts::exceptions::specification &e) {
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

  try {
    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      std::cout << options.help() << std::endl;
      exit(EXIT_SUCCESS);
    }

    if (result.count("version")) {
      print_version_info(std::cout);

      exit(EXIT_SUCCESS);
    }

    if (result.count("input_before")) {
      config.positions_before_path = result["input_before"].as<std::string>();
    } else {
      options_error = true;
      std::cerr << "ERROR:\t Missing input file of before point positions "
                << std::endl;
    }
    if (result.count("input_after")) {
      config.positions_after_path = result["input_after"].as<std::string>();
    } else {
      options_error = true;
      std::cerr << "ERROR:\t Missing input file of after point positions "
                << std::endl;
    }
    if (result.count("output_prefix")) {
      config.output_prefix = result["output_prefix"].as<std::string>();
    } else {
      options_error = true;
      std::cerr << "ERROR:\t Missing output path prefix" << std::endl;
    }
    if (result.count("config_file")) {
      config.config_file_path = result["config_file"].as<std::string>();
      if (config.config_file_path.empty()) {
        config.config_file_path = "./conf/config.txt";
        std::cerr << "No config file provided, defaulting to: "
                  << config.config_file_path << std::endl;
      }
    } else {
      config.config_file_path = "./conf/config.txt";
    }

    if (result.count("debug"))
      config.debugging_enabled = result["debug"].as<bool>();
    if (result.count("verbose"))
      config.verbose_output = result["verbose"].as<bool>();

    if(result.count("cutoff_scale")){
      config.max_match_distance_scale = result["cutoff_scale"].as<float>();
    }
  } catch (const cxxopts::exceptions::parsing &e) {
    std::cerr << "ERROR:\t" << e.what() << std::endl;
    exit(EXIT_FAILURE);
  }
  if (options_error) {
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

  std::ifstream infileSrc(config.positions_before_path);
  std::ifstream infileTar(config.positions_after_path);
  // TODO: Fully remove if actually unused
  // std::ifstream inAreaFileSrc(area_before_path); // unused
  // std::ifstream inAreaFileTar(area_after_path); // unused

  // pointers initialized with actually allocated data to prevent crashes on
  // cleanup
  PClPtr sourcePtr = std::make_shared<PCl2D>();
  PClPtr targetPtr = std::make_shared<PCl2D>();

  size_t before_count = 0;
  pcl::PointXY before_avg{};

  size_t after_count = 0;
  pcl::PointXY after_avg{};

  TransformSettings default_transform{};

  std::vector<pcl::PointXY> before_points;
  std::vector<pcl::PointXY> after_points;

  pcl::PointXY input_point{};
  // read and plot the srouce point cloud
  while (infileSrc >> input_point.x >> input_point.y) {
    before_points.push_back(input_point);
    before_avg.x += input_point.x;
    before_avg.y += input_point.y;
  }
  before_count = before_points.size();
  std::vector<pcl::PointXY> before_points_unnormalized(before_points);

  before_avg.x /= float(before_count);
  before_avg.y /= float(before_count);

  while (infileTar >> input_point.x >> input_point.y) {
    after_points.push_back(input_point);
    after_avg.x += input_point.x;
    after_avg.y += input_point.y;
  }
  after_count = after_points.size();
  std::vector<pcl::PointXY> after_points_unnormalized(after_points);

  after_avg.x /= float(before_count);
  after_avg.y /= float(before_count);

  pcl::PointXY max_pos(-std::numeric_limits<float>::max(),
                       -std::numeric_limits<float>::max()),
      min_pos(std::numeric_limits<float>::max(),
              std::numeric_limits<float>::max());

  // Normalize COM position and find outer bounds of all points
  for (pcl::PointXY &point : before_points) {
    point.x -= before_avg.x;
    point.y -= before_avg.y;

    max_pos.x = std::max(max_pos.x, point.x);
    max_pos.y = std::max(max_pos.y, point.y);

    min_pos.x = std::min(min_pos.x, point.x);
    min_pos.y = std::min(min_pos.y, point.y);
  }

  for (pcl::PointXY &point : after_points) {
    point.x -= after_avg.x;
    point.y -= after_avg.y;

    max_pos.x = std::max(max_pos.x, point.x);
    max_pos.y = std::max(max_pos.y, point.y);

    min_pos.x = std::min(min_pos.x, point.x);
    min_pos.y = std::min(min_pos.y, point.y);
  }

  // The default reference points are the averages normalized to zero
  default_transform.target_after.x = 0.;
  default_transform.target_after.y = 0.;
  default_transform.target_before.x = 0.;
  default_transform.target_before.y = 0.;

  float range_x = max_pos.x - min_pos.x;
  float range_y = max_pos.y - min_pos.y;

  min_plot = pcl::PointXY(min_pos.x - range_x * 0.2, min_pos.y - range_y * 0.2);
  max_plot = pcl::PointXY(max_pos.x + range_x * 0.2, max_pos.y + range_y * 0.2);

  float plot_range_x = max_plot.x - min_plot.x;
  float plot_range_y = max_plot.y - min_plot.y;

  // Plot points and transform into common
  for (int i = 0; i < before_points.size(); i++) {
    pcl::PointXY &point = before_points[i];
    sourcePtr->push_back(point);
  }

  // read and plot the target point cloud
  for (int i = 0; i < after_points.size(); i++) {
    pcl::PointXY &point = after_points[i];
    targetPtr->push_back(point);
  }

  max_match_dist = find_mapping_distance(sourcePtr);

  std::cerr << "max matching distance: " << max_match_dist << std::endl;
  max_match_dist *=  config.max_match_distance_scale;
  std::cerr << "-> Scaled based on config: " << max_match_dist << std::endl;

  std::cout << "Press 'ESC' to quit" << std::endl;

  cv::namedWindow("Merge");
  cv::setMouseCallback("Merge", MouseCallBackFunc, NULL);
  while (true) {
    PCl2D target_tmp;

    imgFixed = cv::Mat::zeros(img_size_x, img_size_y, CV_8UC3);

    // Plot points and transform into common
    for (int i = 0; i < before_points.size(); i++) {
      pcl::PointXY &point = before_points[i];
      cv::Scalar rendercolor =
          (settings.has_matching
               ? (i == indices[0] ? selected_before_color : before_color)
               : before_color);
      // Plot the points as rectangles on the screen
      // cv::circle(imgFixed, cv::Point(x,y), 3, cv::Scalar(0,255,0),-1); //1
      cv::rectangle(
          imgFixed,
          cv::Point((point.x - min_plot.x) / plot_range_x * img_size_x - 2,
                    (point.y - min_plot.y) / plot_range_y * img_size_y - 2),
          cv::Point((point.x - min_plot.x) / plot_range_x * img_size_x + 2,
                    (point.y - min_plot.y) / plot_range_y * img_size_y + 2),
          rendercolor, -1);
    }

    if (settings.has_matching) {
      // Get reference data from array
      default_transform.target_before.x =
          sourcePtr->at(settings.match_index_before).x;
      default_transform.target_before.y =
          sourcePtr->at(settings.match_index_before).y;
      default_transform.target_after.x =
          targetPtr->at(settings.match_index_after).x;
      default_transform.target_after.y =
          targetPtr->at(settings.match_index_after).y;
    } else {
      // The default reference points are the averages normalized to zero
      default_transform.target_after.x = 0.;
      default_transform.target_after.y = 0.;
      default_transform.target_before.x = 0.;
      default_transform.target_before.y = 0.;
    }

    default_transform.shear.x = settings.g_f_shear_x;
    default_transform.shear.y = settings.g_f_shear_y;
    default_transform.stretch.x = settings.g_f_stretch_x;
    default_transform.stretch.y = settings.g_f_stretch_y;

    cv::Mat imgMerged = imgFixed.clone();

    transformPointCloud(*targetPtr, target_tmp, default_transform);
    // read and plot the target point cloud
    for (int i = 0; i < after_points.size(); i++) {
      pcl::PointXY &point = target_tmp.at(i);
      cv::Scalar rendercolor =
          (settings.has_matching
               ? (i == indices[1] ? selected_after_color : after_color)
               : after_color);
      cv::circle(imgMerged,
                 cv::Point((point.x - min_plot.x) / plot_range_x * img_size_x,
                           (point.y - min_plot.y) / plot_range_y * img_size_y),
                 3, rendercolor, -1);
    }

    cv::imshow("Merge", imgMerged);
    char szKey = cv::waitKey(10);
    // stop when escape is pressed
    if (27 == szKey)
      break;
    else if (szKey == 'm') {
      cv::destroyWindow("Merge");
      input_mapping(sourcePtr, targetPtr, settings);
      cv::namedWindow("Merge");
      cv::setMouseCallback("Merge", MouseCallBackFunc, NULL);
    }
    // Check if the window was closed
    else if (!cv::getWindowProperty("Merge", cv::WND_PROP_VISIBLE)) {
      break;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    pcl::registration::CorrespondenceEstimation<pcl::PointXY, pcl::PointXY> est;
    pcl::Correspondences all_correspondences;

    // ... read or fill in source and target
    est.setInputCloud(sourcePtr);
    est.setInputTarget(targetPtr);

    // Determine all reciprocal correspondences
    est.determineReciprocalCorrespondences(all_correspondences, max_match_dist);
    float err = 0;
    for (int i = 0; i < all_correspondences.size(); ++i) {
      err += all_correspondences.at(i).distance;
    }
    err /= all_correspondences.size() + 1;

    // TODO: Disabled for now
    // std::cout << all_correspondences.size() << "	" << err << "\n";

    // Optimize the transformation based on current correspondences
    if (settings.g_bDoCmaes == true) {
      doCMAES(sourcePtr, targetPtr, settings, config);
      settings.g_bDoCmaes = false;

      // Output the new correspondences
      std::ofstream oo;
      std::string oo_path = config.output_prefix + "_mapping.txt";
      oo.open(oo_path);

      if (!oo.is_open() || !oo.good()) {
        std::cerr << "[ERROR]:\tFailed to open output file: " << oo_path
                  << std::endl;
        exit(EXIT_FAILURE);
      }

      print_version_info(oo, "#\t");
      est.setInputCloud(sourcePtr);

      PClPtr target_New = std::make_shared<PCl2D>();
      *target_New = target_tmp;
      est.setInputTarget(target_New);

      // Determine all reciprocal correspondences
      est.determineReciprocalCorrespondences(all_correspondences);

      float err = 0;
      for (int i = 0; i < all_correspondences.size(); ++i) {
        oo << all_correspondences.at(i).index_query << "	"
           << all_correspondences.at(i).index_match << "\n";
      }
      oo.flush();
      oo.close();

      // Output all info for the correspondences
      std::string ooall_path = config.output_prefix + "_all.txt";
      std::ofstream ooAll;

      ooAll.open(ooall_path);

      if (!ooAll.is_open() || !ooAll.good()) {
        std::cerr << "[ERROR]:\tFailed to open output file: " << ooall_path
                  << std::endl;
        exit(EXIT_FAILURE);
      }

      print_version_info(ooAll, "#\t");

      for (int i = 0; i < all_correspondences.size(); ++i) {
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
              << pntSource.x / config.g_f_loadtime_scaling << " "
              << pntSource.y / config.g_f_loadtime_scaling << " "
              << all_correspondences.at(i).index_match << " "
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