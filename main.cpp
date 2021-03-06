
//  638297225
//  2570

#include <iostream>
#include <string.h>

#include "Distance.h"
#include "MultiCameraPnP.h"
#include "SfMUpdateListener.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>


using namespace std;

std::vector<cv::Mat> images;
std::vector<std::string> images_names;

struct REnder : SfMUpdateListener
{
    pcl::visualization::CloudViewer viewer;

    REnder() : viewer("reconstruction") 
    {
        viewer.runOnVisualizationThreadOnce(viewerOneOff);
    }  

    static void viewerOneOff( pcl::visualization::PCLVisualizer& pclviewer )
    {
        //viewer.initCameraParameters ();
        pclviewer.addCoordinateSystem (0.05);
	    pclviewer.setBackgroundColor (0.1, 0.1, 0.35);    
	    pclviewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "reconstruction");
    }


    void update(std::vector<cv::Point3d> points,
				std::vector<cv::Vec3b> rgb, 
				std::vector<cv::Point3d> pcld_alternate,
				std::vector<cv::Vec3b> pcldrgb_alternate, 
				std::vector<cv::Matx34d> cameras) 
    {
        show(points,rgb,50.0);
    }

    void show( std::vector<cv::Point3d> points,	std::vector<cv::Vec3b> rgb, float filter=0.0f )
    {
        if ( points.empty() ) return;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->is_dense = false;

        for (size_t i=0; i<points.size (); ++i)
        {
		    pcl::PointXYZRGB p(0,200,0);
            if ( rgb.size() > i )
            {
                p.r = rgb[i][2];
                p.g = rgb[i][1];
                p.b = rgb[i][0];
            }
            p.x = points[i].x;
            p.y = points[i].y;
            p.z = points[i].z;
            cloud->push_back(p);
        }

        if ( filter > 0 )
        {
            // OutlierRemoval filtering 
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>sor;
            sor.setInputCloud (cloud);
            sor.setMeanK (filter);
            sor.setStddevMulThresh (1.0);
            sor.filter (*cloud_filtered);
            cloud = cloud_filtered;
        }

        pcl::io::savePCDFileASCII ("reco.pcd", *cloud);
        viewer.showCloud (cloud,  "reconstruction" );
    }
    void loop()
    {
	    while (!viewer.wasStopped ())
	    {
            cv::waitKey(10);
	    }
    }
};


//---------------------------- Using command-line ----------------------------
int doSSBA = false;
int main(int argc, char** argv) {
    //string path_to_images = "dataset\\crazyhorse3";
    string path_to_images = "dataset\\cactus3";
    if (argc < 2) {
		cerr << "USAGE: " << argv[0] << " <path_to_images> [use rich features (RICH/OF) = RICH] [use GPU (GPU/CPU) = GPU] [downscale factor = 1.0]" << endl;
		//return 0;
	} else
        path_to_images = string(argv[1]);
	
	double downscale_factor = 1.0;
	//if(argc >= 5)
	//	downscale_factor = atof(argv[4]);

	float outlier_filter = 0.0f;
	if(argc >= 5)
		outlier_filter = atof(argv[4]);

    cv::Ptr<REnder> render = cv::makePtr<REnder>();

    open_imgs_dir(path_to_images.c_str(),images,images_names,downscale_factor);
	if(images.size() == 0) { 
		cerr << "can't get image files" << endl;
		return 1;
	}

    cv::Ptr<MultiCameraPnP> distance = cv::makePtr<MultiCameraPnP>(images,images_names,path_to_images);
	if(argc < 3)
		distance->use_rich_features = true;
	else
		distance->use_rich_features = (stricmp(argv[2], "RICH") == 0);

	if(argc>3)
	    doSSBA = (stricmp(argv[3], "ssba") == 0);

	distance->use_gpu = false;
	
	//cv::Ptr<VisualizerListener> visualizerListener = cv::makePtr<VisualizerListener>(); //with ref-count
	distance->attach(render);
	
    //RunVisualizationThread();
    //render->show(distance->getPointCloud(),distance->getPointCloudRGB(),outlier_filter);

    cerr  << " " << path_to_images 
        << " rich:" << distance->use_rich_features 
        << " sbba:" << doSSBA << endl;

	distance->RecoverDepthFromImages();

    render->loop();
    //int k=0;
    //cin >> k;
	//TODO: save point cloud and cameras to file
}

