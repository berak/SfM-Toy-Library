/*
 *  MultiCameraDistance.cpp
 *  SfMToyExample
 *
 *  Created by Roy Shilkrot on 3/27/12.
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2013 Roy Shilkrot
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 */

#include "MultiCameraDistance.h"
#include "RichFeatureMatcher.h"
#include "OFFeatureMatcher.h"
#include "GPUSURFFeatureMatcher.h"
#include "assert.h"
#include "opencv2/core/utility.hpp"

void MultiCameraDistance::setImages(const std::vector<cv::Mat>& imgs_,
		const std::vector<std::string>& imgs_names_,
		const std::string& imgs_path_)
{
	imgpts.clear();
	fullpts.clear();
	imgpts_good.clear();
	matches_matrix.clear();
	imgs_orig.clear();
	imgs.clear();
	imgs_names.clear();
	Pmats.clear();

	std::cout  <<"=========================== Load Images ===========================\n";
	imgs_names = imgs_names_;
	//ensure images are CV_8UC3
	for (unsigned int i = 0; i < imgs_.size(); i++) {
		imgs_orig.push_back(cv::Mat_<cv::Vec3b>());
		if (!imgs_[i].empty()) {
			if (imgs_[i].type() == CV_8UC1) {
                cvtColor(imgs_[i], imgs_orig[i], cv::COLOR_GRAY2BGR);
			} else if (imgs_[i].type() == CV_32FC3 || imgs_[i].type()
					== CV_64FC3) {
				imgs_[i].convertTo(imgs_orig[i], CV_8UC3, 255.0);
			} else {
				imgs_[i].copyTo(imgs_orig[i]);
			}
		}

		imgs.push_back(cv::Mat());
        cvtColor(imgs_orig[i], imgs[i], cv::COLOR_BGR2GRAY);

		imgpts.push_back(std::vector<cv::KeyPoint>());
		imgpts_good.push_back(std::vector<cv::KeyPoint>());
		std::cout  <<imgs_names[i] << std::endl;
	}
	std::cout  <<std::endl;

	init(imgs_path_);
}

void MultiCameraDistance::init(const std::string& imgs_path_) {
	//load calibration matrix
	cv::FileStorage fs;
	if (fs.open(imgs_path_ + "\\out_camera_data.yml", cv::FileStorage::READ)) {
		fs["camera_matrix"] >> cam_matrix;
		fs["distortion_coefficients"] >> distortion_coeff;
	} else {
		//no calibration matrix file - mockup calibration
		cv::Size imgs_size = imgs[0].size();
		double max_w_h = MAX(imgs_size.height,imgs_size.width);
		cam_matrix = (cv::Mat_<double>(3, 3) << max_w_h, 0, imgs_size.width
				/ 2.0, 0, max_w_h, imgs_size.height / 2.0, 0, 0, 1);
		distortion_coeff = cv::Mat_<double>::zeros(1, 4);
	}

	K = cam_matrix;
	invert(K, Kinv); //get inverse of camera matrix

	distortion_coeff.convertTo(distcoeff_32f, CV_32FC1);
	K.convertTo(K_32f, CV_32FC1);

	bInitialized = true;
}

void MultiCameraDistance::OnlyMatchFeatures()
{
	if(features_matched) return;

	std::cout  <<"Matching features...\n";
	
	if (use_rich_features) {
		if (use_gpu) {
			std::cout  <<"Using GPU\n";
//			feature_matcher = cv::makePtr<GPUSURFFeatureMatcher>(imgs,imgpts);
		} else {
			std::cout  <<"Using CPU\n";
            feature_matcher = cv::Ptr<RichFeatureMatcher>(new RichFeatureMatcher(imgs,imgpts));
		}
	} else {
		std::cout  <<"Using Optical Flow\n";
        feature_matcher = cv::Ptr<OFFeatureMatcher>(new OFFeatureMatcher(use_gpu,imgs,imgpts));
	}	

	int loop1_top = imgs.size() - 1, loop2_top = imgs.size();
	int frame_num_i = 0;
	//#pragma omp parallel for schedule(dynamic)
	
	//if (use_rich_features) {
	//	for (frame_num_i = 0; frame_num_i < loop1_top; frame_num_i++) {
	//		for (int frame_num_j = frame_num_i + 1; frame_num_j < loop2_top; frame_num_j++)
	//		{
	//			std::vector<cv::KeyPoint> fp,fp1;
	//			std::cout  <<"------------ Match " << imgs_names[frame_num_i] << ","<<imgs_names[frame_num_j]<<" ------------\n";
	//			std::vector<cv::DMatch> matches_tmp;
	//			feature_matcher->MatchFeatures(frame_num_i,frame_num_j,&matches_tmp);
	//			
	//			//#pragma omp critical
	//			{
	//				matches_matrix[std::make_pair(frame_num_i,frame_num_j)] = matches_tmp;
	//			}
	//		}
	//	}
	//} else {
    cv::FileStorage fs("matches.yml",cv::FileStorage::READ);
    bool doRead = fs.isOpened();
    if ( ! doRead )
    {
        fs = cv::FileStorage("matches.yml",cv::FileStorage::WRITE);
    }
#pragma omp parallel for
		for (frame_num_i = 0; frame_num_i < loop1_top; frame_num_i++) {
			for (int frame_num_j = frame_num_i + 1; frame_num_j < loop2_top; frame_num_j++)
			{
				std::cout  <<"------------ Match " << imgs_names[frame_num_i] << ","<<imgs_names[frame_num_j]<<" ------------\n";
				std::vector<cv::DMatch> matches_tmp;
                std::string kn = cv::format("match_%02d_%02d",frame_num_i,frame_num_j);
                if ( doRead ) 
                {
                    fs[kn] >> matches_tmp;  
                } 
                else 
                {
				    feature_matcher->MatchFeatures(frame_num_i,frame_num_j,&matches_tmp);
                    fs << kn << matches_tmp;
                }
				matches_matrix[std::make_pair(frame_num_i,frame_num_j)] = matches_tmp;
				std::vector<cv::DMatch> matches_tmp_flip = FlipMatches(matches_tmp);
				matches_matrix[std::make_pair(frame_num_j,frame_num_i)] = matches_tmp_flip;
			}
		}
	//}
    fs.release();
	features_matched = true;
}

void MultiCameraDistance::GetRGBForPointCloud(
	const std::vector<struct CloudPoint>& _pcloud,
	std::vector<cv::Vec3b>& RGBforCloud
	) 
{
	RGBforCloud.resize(_pcloud.size());
	for (unsigned int i=0; i<_pcloud.size(); i++) {
		unsigned int good_view = 0;
		std::vector<cv::Vec3b> point_colors;
		for(; good_view < imgs_orig.size(); good_view++) {
			if(_pcloud[i].imgpt_for_img[good_view] != -1) {
				int pt_idx = _pcloud[i].imgpt_for_img[good_view];
				if(pt_idx >= imgpts[good_view].size()) {
					std::cerr << "BUG: point id:" << pt_idx << " should not exist for img #" << good_view << " which has only " << imgpts[good_view].size() << std::endl;
					continue;
				}
				cv::Point _pt = imgpts[good_view][pt_idx].pt;
				assert(good_view < imgs_orig.size() && _pt.x < imgs_orig[good_view].cols && _pt.y < imgs_orig[good_view].rows);
				
				point_colors.push_back(imgs_orig[good_view].at<cv::Vec3b>(_pt));
				
//				std::stringstream ss; ss << "patch " << good_view;
//				imshow_250x250(ss.str(), imgs_orig[good_view](cv::Range(_pt.y-10,_pt.y+10),cv::Range(_pt.x-10,_pt.x+10)));
			}
		}
//		cv::waitKey(0);
		cv::Scalar res_color = cv::mean(point_colors);
		RGBforCloud[i] = (cv::Vec3b(res_color[0],res_color[1],res_color[2])); //bgr2rgb
		if(good_view == imgs.size()) //nothing found.. put red dot
			RGBforCloud.push_back(cv::Vec3b(255,0,0));
	}
}
