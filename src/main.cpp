#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "cmaes.h"
#include "parameters.h"

float g_fTx, g_fTy, g_fSx, g_fSy;
float g_fSxMin, g_fSxMax, g_fSyMin, g_fSyMax, g_fScaling;
bool g_bDoCmaes;
bool g_bEventLButtonDown;
bool g_bEventRButtonDown;
cv::Vec2f pntCur, pntBegin;
cv::Mat imgFixed = cv::Mat::zeros(1024, 1024, CV_8UC3);
cv::Mat imgMerged = cv::Mat::zeros(1024, 1024, CV_8UC3);
pcl::PointCloud<pcl::PointXY> ptSource, ptTarget, ptTargetNew;
pcl::registration::CorrespondenceEstimation<pcl::PointXY, pcl::PointXY> est;
pcl::Correspondences all_correspondences;
pcl::PointCloud<pcl::PointXY>::Ptr sourcePtr(&ptSource);
pcl::PointCloud<pcl::PointXY>::Ptr targetPtr(&ptTargetNew);

const std::string SxMin = "Sx_Min";
const std::string SxMax = "Sx_Max";
const std::string SyMin = "Sy_Min";
const std::string SyMax = "Sy_Max";
const std::string Scaling = "Scaling";

// Read configuration file
void readConfigFile(const std::string sConfigFileName)
{
	std::string sKey;
	std::string sValue;
	std::ifstream f(sConfigFileName);
	if (f.is_open())
	{
		while (!f.eof())
		{
			f >> sKey >> sValue;
			if (sKey == SxMin)
			{
				g_fSxMin = atof(sValue.data());
			}
			if (sKey == SxMax)
			{
				g_fSxMax = atof(sValue.data());
			}
			if (sKey == SyMin)
			{
				g_fSyMin = atof(sValue.data());
			}
			if (sKey == SyMax)
			{
				g_fSyMax = atof(sValue.data());
			}
			if (sKey == Scaling)
			{
				g_fScaling = atof(sValue.data());
			}
		}
		f.close();
	}
	else
	{
		std::cout << "Can't open file" << std::endl;
	}
	f.close();
}

// Mouse control
void MouseCallBackFunc(int event, int x, int y, int flags, void *userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		g_bEventLButtonDown = true;
		pntCur[0] = x;
		pntCur[1] = y;
		pntBegin[0] = x;
		pntBegin[1] = y;
	}
	else if (event == cv::EVENT_LBUTTONUP)
	{
		g_bEventLButtonDown = false;
	}
	else if (event == cv::EVENT_RBUTTONDOWN)
	{
		g_bEventRButtonDown = true;
		pntCur[0] = x;
		pntCur[1] = y;
		pntBegin[0] = x;
		pntBegin[1] = y;
	}
	else if (event == cv::EVENT_RBUTTONUP)
	{
		g_bEventRButtonDown = false;
	}
	else if (event == cv::EVENT_MOUSEMOVE)
	{
		pntCur[0] = x;
		pntCur[1] = y;
		if (g_bEventLButtonDown == true)
		{
			g_fTx = pntCur[0] - pntBegin[0];
			g_fTy = pntCur[1] - pntBegin[1];
		}

		if (g_bEventRButtonDown == true)
		{
			g_fSx = g_fSx + (pntCur[0] - pntBegin[0]) * 0.001;
			g_fSy = g_fSy + (pntCur[1] - pntBegin[1]) * 0.001;
		}
	}
	else if (event == cv::EVENT_LBUTTONDBLCLK)
	{

		g_bDoCmaes = true;
	}
}

// Transformation calculation
void transformPointCloud(pcl::PointCloud<pcl::PointXY> &pcIn, pcl::PointCloud<pcl::PointXY> &pcOut, float tx, float ty, float sx, float sy)
{
	cv::Mat matTransfo = cv::Mat::eye(3, 3, CV_32F);
	matTransfo.at<float>(0, 0) *= sx;
	matTransfo.at<float>(1, 0) *= sx;

	matTransfo.at<float>(0, 1) *= sy;
	matTransfo.at<float>(1, 1) *= sy;

	matTransfo.at<float>(0, 2) = tx;
	matTransfo.at<float>(1, 2) = ty;

	cv::Mat pntIn = cv::Mat::ones(3, 1, CV_32F);
	cv::Mat pntOut = cv::Mat::ones(3, 1, CV_32F);
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
float costFunction(float tx, float ty, float sx, float sy)
{
	// Range limitation
	if (sx < g_fSxMin || sx > g_fSxMax || sy < g_fSyMin || sy > g_fSyMax)
		return FLT_MAX;
	imgMerged = imgFixed.clone();
	transformPointCloud(ptTarget, ptTargetNew, tx, ty, sx, sy);
	for (int i = 0; i < ptTargetNew.size(); ++i)
	{
		cv::circle(imgMerged, cv::Point(ptTargetNew.at(i).x, ptTargetNew.at(i).y), 3, cv::Scalar(0, 255, 255), -1);
	}

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
	err /= (all_correspondences.size() + 1) * (float(all_correspondences.size()) / float(sourcePtr->size())) * (float(all_correspondences.size()) / float(targetPtr->size()));
	// err/=all_correspondences.size()+1; //not work for large cluster
	std::cout << all_correspondences.size() << "	" << err << "\n";
	return err;
}

// CMA-ES optimization
void doCMAES(float tx, float ty, float sx, float sy)
{
	CMAES<float> evo;  // the optimizer
	float *const *pop; // sampled population
	float *fitvals;	   // objective function values of sampled population
	float fbestever = 10000;
	float *xbestever = NULL; // store best solution
	float fmean;
	int irun;
	int lambda = 50;	// offspring population size, 0 invokes default
	int countevals = 0; // used to set for restarts

	// for (irun = 0; irun < nrestarts+1; ++irun)

	/* Parameters can be set in two ways. Here as input parameter to evo.init()
		and as value read from signals.par by calling evo.readSignals
		explicitely.
	  */
	const int dim = 4;
	float xstart[dim];
	xstart[0] = tx;
	xstart[1] = ty;
	xstart[2] = sx;
	xstart[3] = sy;

	float stddev[dim];
	stddev[0] = 2;
	stddev[1] = 2;
	stddev[2] = 5;
	stddev[3] = 5;

	float lb[dim];
	lb[0] = xstart[0] - 20;
	lb[1] = xstart[1] - 20;
	lb[2] = xstart[2] - 20;
	lb[3] = xstart[3] - 20;

	float ub[dim];
	ub[0] = xstart[0] + 20;
	ub[1] = xstart[1] + 20;
	ub[2] = xstart[2] + 20;
	ub[3] = xstart[3] + 20;
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
				initialX is feasible (or in case typicalX +- 2*initialStandardDeviations
				is feasible) and initialStandardDeviations is (are)
				sufficiently small to prevent quasi-infinite looping.
			  */
			/* while (!is_feasible(pop[i]))
				  evo.reSampleSingle(i);
			*/
			fitvals[i] = costFunction(pop[i][0], pop[i][1], 1.0 + pop[i][2] * 0.01, 1.0 + pop[i][3] * 0.01);
		}
		// update search distribution
		evo.updateDistribution(fitvals);
	}

	// keep best ever solution
	// if (irun == 0 || evo.get(CMAES<float>::FBestEver) < fbestever)
	if (evo.get(CMAES<float>::FBestEver) < fbestever)
	{
		fbestever = evo.get(CMAES<float>::FBestEver);
		xbestever = evo.getInto(CMAES<float>::XBestEver, xbestever); // allocates memory if needed
	}

	g_fTx = xbestever[0];
	g_fTy = xbestever[1];
	g_fSx = xbestever[2] * 0.01 + 1.0;
	g_fSy = xbestever[3] * 0.01 + 1.0;
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
	std::string sInfileSrc = argv[1];
	std::string sInfileTar = argv[2];
	std::string sAreaFileSrc = argv[3];
	std::string sAreaFileTar = argv[4];
	std::string sOutFile = argv[5];
	std::string sConfigFile = argv[6];

	g_bDoCmaes = false;
	g_fTx = 0;
	g_fTy = 0;
	g_fSx = 1.2;
	g_fSy = 1.0;

	readConfigFile(sConfigFile);

	cv::namedWindow("Merge");
	cv::setMouseCallback("Merge", MouseCallBackFunc, NULL);
	std::ifstream infileSrc(sInfileSrc);
	std::ifstream infileTar(sInfileTar);
	std::ifstream inAreaFileSrc(sAreaFileSrc);
	std::ifstream inAreaFileTar(sAreaFileTar);

	float x, y;
	// read and plot the srouce point cloud
	while (infileSrc >> x >> y)
	{
		// x *= 0.2;
		// y *= 0.2;
		x *= g_fScaling;
		y *= g_fScaling;
		pcl::PointXY pnt;
		pnt.x = x;
		pnt.y = y;

		// cv::circle(imgFixed, cv::Point(x,y), 3, cv::Scalar(0,255,0),-1); //1
		cv::rectangle(imgFixed, cv::Point(x - 2, y - 2), cv::Point(x + 2, y + 2), cv::Scalar(255, 100, 0), -1);
		ptSource.push_back(pnt);
	}
	imgMerged = imgFixed.clone();

	// read and plot the target point cloud
	while (infileTar >> x >> y)
	{
		// x *= 0.2;
		// y *= 0.2;
		x *= g_fScaling;
		y *= g_fScaling;
		pcl::PointXY pnt;
		pnt.x = x;
		pnt.y = y;
		ptTarget.push_back(pnt);
		cv::circle(imgMerged, cv::Point(x, y), 3, cv::Scalar(0, 255, 255), -1);
	}

	while (true)
	{
		imgMerged = imgFixed.clone();
		transformPointCloud(ptTarget, ptTargetNew, g_fTx, g_fTy, g_fSx, g_fSy);
		for (int i = 0; i < ptTargetNew.size(); ++i)
		{
			cv::circle(imgMerged, cv::Point(ptTargetNew.at(i).x, ptTargetNew.at(i).y), 3, cv::Scalar(0, 255, 255), -1);
		}

		cv::imshow("Merge", imgMerged);
		char szKey = cv::waitKey(10);
		// stop when press space
		if (27 == szKey)
			break;

		///////////////////////////////////////////////////////////////////////////////////////////////////////

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
		if (g_bDoCmaes == true)
		{
			doCMAES(g_fTx, g_fTy, (g_fSx - 1.0) * 100.0, (g_fSy - 1.0) * 100.0);
			g_bDoCmaes = false;

			// Output the new correspondences
			std::ofstream oo;
			oo.open(sOutFile);
			est.setInputCloud(sourcePtr);
			est.setInputTarget(targetPtr);

			// Determine all reciprocal correspondences
			est.determineReciprocalCorrespondences(all_correspondences);

			float err = 0;
			for (int i = 0; i < all_correspondences.size(); ++i)
			{
				oo << all_correspondences.at(i).index_query << "	" << all_correspondences.at(i).index_match << "\n";
			}
			oo.flush();
			oo.close();

			// Output all info for the correspondences
			std::string sOutAll = sOutFile + "_all.txt";
			std::ofstream ooAll;
			ooAll.open(sOutAll);
			for (int i = 0; i < all_correspondences.size(); ++i)
			{
				pcl::PointXY pntSource;
				pntSource = ptSource.points.at(all_correspondences.at(i).index_query);
				pcl::PointXY pntTargetNew;
				pntTargetNew = ptTargetNew.points.at(all_correspondences.at(i).index_match);
				// ooAll<<all_correspondences.at(i).index_query<<" "<< pntSource.x/0.2<< " " << pntSource.y/0.2 << " "  << all_correspondences.at(i).index_match << " " << pntTargetNew.x/0.2 << " " << pntTargetNew.y/0.2<<"\n";
				ooAll << all_correspondences.at(i).index_query << " " << pntSource.x / g_fScaling << " " << pntSource.y / g_fScaling << " " << all_correspondences.at(i).index_match << " " << pntTargetNew.x / g_fScaling << " " << pntTargetNew.y / g_fScaling << "\n";
			}
			ooAll.flush();
			ooAll.close();
			cv::waitKey(-1);
		}
	}

	return (0);
}