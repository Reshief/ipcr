#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <fstream>
#include "cv.h"
#include "highgui.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "cmaes.h"
#include "parameters.h"

//Define the paramenters
float g_Tx, g_Ty, g_Sx, g_Sy;
bool g_do_cmaes;
bool g_event_lbuttondown;
bool g_event_rbuttondown;
cv::Vec2f pntCur, pntBegin;
cv::Mat FixImg = cv::Mat::zeros(512, 512, CV_8UC3);
cv::Mat MergedImg = cv::Mat::zeros(512, 512, CV_8UC3);

// 2D pointclouds
pcl::PointCloud<pcl::PointXY> source, target, targetNew;
// CorrespondenceEstimation represents the base class for determining correspondences between target and query point sets/features.
pcl::registration::CorrespondenceEstimation<pcl::PointXY, pcl::PointXY> est; 

// Determine all reciprocal correspondences
pcl::Correspondences all_correspondences;

// Stretched Points
pcl::PointCloud<pcl::PointXY>::Ptr sourcePtr(&source);
// Unstretched Points
pcl::PointCloud<pcl::PointXY>::Ptr targetPtr(&targetNew);

// OpenCV supports for detecting mouse events. 
// Mouse events include mouse clicks and movements over an attached OpenCV window.
void MouseCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	// "Left button down of the mouse is pressed - position"
	if(event == cv::EVENT_LBUTTONDOWN)
	{
		g_event_lbuttondown = true;
		pntCur[0] = x;
		pntCur[1] = y;
		pntBegin[0] = x;
		pntBegin[1] = y;
	}
	// "indicates that left mouse button is released"
	else if(event == cv::EVENT_LBUTTONUP)
	{
		g_event_lbuttondown = false;
	}
	// "indicates that the right mouse button is pressed - position"	
	else if(event == cv::EVENT_RBUTTONDOWN)
	{
		g_event_rbuttondown = true;
		pntCur[0] = x;
		pntCur[1] = y;
		pntBegin[0] = x;
		pntBegin[1] = y;

	}
	// "indicates that right mouse button is released"		
	else if(event == cv::EVENT_RBUTTONUP)
	{
		g_event_rbuttondown = false;
	}
	//	"indicates that the mouse pointer has moved over the window - position"	
	else if(event == cv::EVENT_MOUSEMOVE)
	{
		pntCur[0] = x;
		pntCur[1] = y;
	// "indicates that the left mouse button is pressed - position"		
		if(g_event_lbuttondown == true)
		{
			g_Tx = pntCur[0] - pntBegin[0];
			g_Ty = pntCur[1] - pntBegin[1];
		}
	// "indicates that the right mouse button is pressed - position"			
		if(g_event_rbuttondown == true)
		{
			g_Sx =g_Sx + (pntCur[0] - pntBegin[0])*0.001;
			g_Sy = g_Sy + (pntCur[1] - pntBegin[1])*0.001;
		}
	}
	// indicates that left mouse button is double clicked.	
	else if(event == cv::EVENT_LBUTTONDBLCLK)
	{
		g_do_cmaes = true;
	}
}
// Cloud_in the input point cloud, Cloud_out the resultant output point cloud. 
void TransformPointCloud(pcl::PointCloud<pcl::PointXY>& pcIn, pcl::PointCloud<pcl::PointXY>& pcOut,float tx, float ty, float sx, float sy)
{
	cv::Mat matTransfo = cv::Mat::eye(3,3,CV_32F);
	// Scale x axis
	matTransfo.at<float>(0,0) *= sx;
	matTransfo.at<float>(1,0) *= sx;
	// Scale y axis
	matTransfo.at<float>(0,1) *= sy;
	matTransfo.at<float>(1,1) *= sy;
	// Translation vector
	matTransfo.at<float>(0,2) = tx;
	matTransfo.at<float>(1,2) = ty;
	// Point Cloud in and resultant output point cloud
	cv::Mat pntIn = cv::Mat::ones(3,1, CV_32F);
	cv::Mat pntOut = cv::Mat::ones(3,1, CV_32F);
	pcOut.resize(pcIn.size());
	
	// For all the members of point cloud in
	for (int i=0; i<pcIn.size(); ++i)
	{
		pntIn.at<float>(0,0) = pcIn.at(i).x;
		pntIn.at<float>(1,0) = pcIn.at(i).y;
		pntIn.at<float>(2,0) = 1.0;

		pntOut = matTransfo * pntIn;
		pcOut.at(i).x = pntOut.at<float>(0,0)/pntOut.at<float>(2,0);
		pcOut.at(i).y = pntOut.at<float>(1,0)/pntOut.at<float>(2,0);
	}
}

// Determine cost function based on the distance of the two point couds
float costFunction(float tx, float ty, float sx, float sy)
{
	MergedImg = FixImg.clone();
	TransformPointCloud(target, targetNew, tx, ty, sx, sy);
	for (int i=0; i<targetNew.size(); ++i)
	{
		cv::circle(MergedImg, cv::Point(targetNew.at(i).x,targetNew.at(i).y), 1, cv::Scalar(0,0,255), -1);
	}

	cv::imshow("Merge", MergedImg);
	char szKey = cv::waitKey(10);

	est.setInputCloud(sourcePtr);
	est.setInputTarget (targetPtr);

	// Determine all reciprocal correspondences
	est.determineReciprocalCorrespondences (all_correspondences);

	float err = 0;
	for (int i=0; i<all_correspondences.size(); ++i)
	{
		err += all_correspondences.at(i).distance;

	}
	err/=all_correspondences.size()+1;
	std::cout<<all_correspondences.size()<<"	"<<err<<"\n";
	return err;
}

// Apply CMA-ES as an optimizer
void DoCMAES(float tx, float ty, float sx, float sy)
{
  CMAES<float> evo; // the optimizer
  float *const*pop; // sampled population
  float *fitvals;   // objective function values of sampled population
  float fbestever=0;
  float *xbestever=NULL; // store best solution
  float fmean; 
  int irun;
  int lambda = 50;      // offspring population size, 0 invokes default
  int countevals = 0;  // used to set for restarts

  //for (irun = 0; irun < nrestarts+1; ++irun)
  
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
	lb[0] =  xstart[0]-20;
	lb[1] =  xstart[1]-20;
	lb[2] =  xstart[2]-20;
	lb[3] =  xstart[3]-20;

	float ub[dim];
	ub[0] =  xstart[0]+20;
	ub[1] =  xstart[1]+20;
	ub[2] =  xstart[2]+20;
	ub[3] =  xstart[3]+20;
    Parameters<float> parameters;
    // You can resume a previous run by specifying a file that contains the
    // resume data:
    //parameters.resumefile = "resumeevo2.dat";
    parameters.logWarnings = true; // warnings will be printed on std::cerr
    parameters.stopTolX = 1e-2;
	parameters.stopTolFun=0.01;
    parameters.updateCmode.maxtime = 1.0;
    parameters.lambda = lambda;
	parameters.stopMaxIter=30;
    parameters.init(dim, xstart, stddev);

    fitvals = evo.init(parameters); // allocs fitvals
    std::cout << evo.sayHello() << std::endl;
    evo.countevals = countevals; // a hack, effects the output and termination

    while(!evo.testForTermination())
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
      for(int i = 0; i < evo.get(CMAES<float>::PopSize); ++i)
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
        fitvals[i] = costFunction(pop[i][0], pop[i][1], 1.0+pop[i][2]*0.01, 1.0+pop[i][3]*0.01); 
      }

      // update search distribution
      evo.updateDistribution(fitvals);
      
    }
    // keep best ever solution
    //if (irun == 0 || evo.get(CMAES<float>::FBestEver) < fbestever)
    {
      fbestever = evo.get(CMAES<float>::FBestEver); 
      xbestever = evo.getInto(CMAES<float>::XBestEver, xbestever); // allocates memory if needed
    }

	g_Tx = xbestever[0];
	g_Ty = xbestever[1];
	g_Sx = xbestever[2]*0.01+1.0;
	g_Sy = xbestever[3]*0.01+1.0;

}

// Main function. Call the methods 
int main (int argc, char** argv)
{
	std::string sInfileSrc = argv[1];
	std::string sInfileTar = argv[2];
	std::string sOutfile = argv[3];
	g_do_cmaes = false;
	g_Tx = 0;
	g_Ty = 0;
	g_Sx = 1.2;
	g_Sy = 1.0;
	cv::namedWindow("Merge");
	cv::setMouseCallback("Merge", MouseCallBackFunc, NULL);
	std::ifstream infileSrc(sInfileSrc); //"Centre.txt"); //Stretched
	std::ifstream infileTar(sInfileTar); //"CentreFused.txt"); //Unstreched
	
	float x,y;
	
	while(infileSrc>>x>>y)
	{
		pcl::PointXY pnt;
		pnt.x = x;
		pnt.y = y;

		cv::circle(FixImg, cv::Point(x,y), 1, cv::Scalar(0,255,0), -1);
		source.push_back(pnt);
	}
	MergedImg = FixImg.clone();

	while(infileTar>>x>>y)
	{
		pcl::PointXY pnt;
		pnt.x = x;
		pnt.y = y;
		target.push_back(pnt);
		cv::circle(MergedImg, cv::Point(x,y), 1, cv::Scalar(0,0,255), -1);
	}
	
	while(true)
	{
		MergedImg = FixImg.clone();
		TransformPointCloud(target, targetNew, g_Tx, g_Ty, g_Sx, g_Sy);
		for (int i=0; i<targetNew.size(); ++i)
		{
			cv::circle(MergedImg, cv::Point(targetNew.at(i).x,targetNew.at(i).y), 1, cv::Scalar(0,0,255), -1);
		}

		cv::imshow("Merge", MergedImg);
		char szKey = cv::waitKey(10);
		if(27==szKey)
			break;	
		///////////////////////////////////////////////////////////////////////////////////////////////////////
		// ... read or fill in source and target
		est.setInputCloud(sourcePtr);
		est.setInputTarget (targetPtr);
		// Determine all reciprocal correspondences
		est.determineReciprocalCorrespondences (all_correspondences);
		float err = 0;
		for (int i=0; i<all_correspondences.size(); ++i)
		{
			err += all_correspondences.at(i).distance;
		}
		err/=all_correspondences.size()+1;
		std::cout<<all_correspondences.size()<<"	"<<err<<"\n";

		if(g_do_cmaes==true)
		{
			DoCMAES(g_Tx, g_Ty, (g_Sx-1.0)*100.0, (g_Sy-1.0)*100.0 );
			g_do_cmaes = false;
			ofstream oo;
			oo.open(sOutfile);//"e:\\corresp.txt");
			est.setInputCloud(sourcePtr);
			est.setInputTarget (targetPtr);
			
			// Determine all reciprocal correspondences
			est.determineReciprocalCorrespondences (all_correspondences);

			float err = 0;
			for (int i=0; i<all_correspondences.size(); ++i)
			{
				oo<<all_correspondences.at(i).index_query<<"	"<<all_correspondences.at(i).index_match<<"\n";
			}
			oo.flush();
			oo.close();

			// Output all info for the correspondences
			std::string sOutAll=sOutfile + "_all.txt";
			ofstream ooAll;
			ooAll.open(sOutAll);
			for (int i=0; i<all_correspondences.size(); ++i)
			{
				pcl::PointXY pntSource;
				pntSource= source.points.at(all_correspondences.at(i).index_query);
				pcl::PointXY pntTargetNew;
				pntTargetNew= targetNew.points.at(all_correspondences.at(i).index_match);
				ooAll<<all_correspondences.at(i).index_query<<" "<< pntSource.x  << " " << pntSource.y << " "  << all_correspondences.at(i).index_match << " " << pntTargetNew.x << " " << pntTargetNew.y<<"\n";
			}
			ooAll.flush();
			ooAll.close();
			cv::waitKey(-1);
		}
	}
	return (0);
}