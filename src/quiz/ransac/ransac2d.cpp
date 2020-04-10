/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#include <math.h> 
#include <iostream>
#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -15; i < 15; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations 
	while(maxIterations --)
	{
		std::unordered_set<int> inliers;
		// Randomly sample subset and fit line
		while(inliers.size()<2)
		{
			inliers.insert(rand()%(cloud->points.size()));
		}
		float x[3],y[3],z[2];
		auto itr = inliers.begin();
		for (auto i = 0; i<2; i++)
		{
			x[i] = cloud->points[*itr].x;
			y[i] = cloud->points[*itr].y;
			z[i] = cloud->points[*itr].z;
			itr++;
		}
		float a,b,c,delta;

			a = y[0] - y[1];
			b = x[1] - x[0];
			c = x[0]*y[1] - x[1]*y[0];
			delta = sqrt(a*a +b*b);

		for( int index = 0; index < cloud->points.size(); index ++ )
		{
			if(inliers.count(index)>0)
				continue;

			pcl::PointXYZ point = cloud->points[index];
			x[2] = cloud->points[index].x;
			y[2] = cloud->points[index].y;
			float d = fabs(a*x[2] + b*y[2] + c );

			if(d <= distanceTol*delta)
			{
				inliers.insert(index);
			}
			if(inliers.size() > inliersResult.size() )
				{
					inliersResult = inliers;
				}
			
		}
		std::cout <<maxIterations<< " "<< inliers.size() << std::endl;
	}


	return inliersResult;

}
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations 
	while(maxIterations --)
	{
		std::unordered_set<int> inliers;
		// Randomly sample subset and fit line
		while(inliers.size()<3)
		{
			inliers.insert(rand()%(cloud->points.size()));
		}
		float x[3],y[3],z[3];
		auto itr = inliers.begin();
		for (auto i = 0; i<3; i++)
		{
			x[i] = cloud->points[*itr].x;
			y[i] = cloud->points[*itr].y;
			z[i] = cloud->points[*itr].z;
			itr++;
		}
		float a,b,c,d,delta;

			a = (y[1] - y[0])*(z[2] - z[0]) - (z[1]-z[0])*(y[2]-y[1]);
			b = (z[1] - z[0])*(x[2] - x[0]) - (x[1]-x[0])*(z[2]-z[1]);
			c = (x[1] - x[0])*(y[2] - y[0]) - (y[1]-y[0])*(x[2]-x[1]);
			d = -1.*(a*x[0] + b*y[0] + c*z[0]);

			delta = sqrt(a*a +b*b + c*c);

		for( int index = 0; index < cloud->points.size(); index ++ )
		{
			if(inliers.count(index)>0)
				continue;

			pcl::PointXYZ point = cloud->points[index];
			float xx = cloud->points[index].x;
			float yy = cloud->points[index].y;
			float zz = cloud->points[index].z;
			float dd = fabs(a*xx + b*yy + c*zz + d);
 
			if(dd <= distanceTol*delta)
			{
				inliers.insert(index);
			}
			if(inliers.size() > inliersResult.size() )
				{
					inliersResult = inliers;
				}
			
		}
	}


	return inliersResult;

}
int main ()
{

	// Create viewer
	
	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 15, 0.2);
	//return 1;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,0,1));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
