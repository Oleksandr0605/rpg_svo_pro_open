#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
//#include <glog/logging.h>
#include <gflags/gflags.h>
#include <opencv2/opencv.hpp>
#include <vikit/sample.h>
#include <svo/common/conversions.h>
#include <svo/common/frame.h>
#include <svo/common/point.h>
#include <svo/imu_handler.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/direct/feature_detection.h>
#include <svo/direct/feature_detection_utils.h>
#include <svo_ros/csv_dataset_reader.h>
#include <iostream>

// Add declarations for imu_handler_ and set_initial_attitude_from_gravity_
std::shared_ptr<svo::FrameHandlerBase> svo_;
std::shared_ptr<svo::ImuHandler> imu_handler_;
bool set_initial_attitude_from_gravity_ = true; // Initialize appropriately
double img_noise_sigma_;
size_t sleep_us_;
size_t first_frame_id_ = 0u;
size_t last_frame_id_ = 0u;
size_t blackout_start_id_ = 0u;
size_t blackout_end_id_ = 0u;
bool trace_only_kf_;


/*void processImageBundle(
        const std::vector<cv::Mat>& images,
        const int64_t timestamp_nanoseconds)
{
    if (imu_handler_)
    {
        svo_->setBundleAdjuster(ceres_backend_interface_);
        ceres_backend_interface_->setImu(imu_handler_);
        ceres_backend_interface_->makePublisher(pnh_, ceres_backend_publisher_);
    }
    else
    {
        SVO_ERROR_STREAM("Cannot use ceres backend without using imu");
    }
    svo_->addImageBundle(images, timestamp_nanoseconds);
}*/


bool loadNextImages(const std::string& dataset_basedir,
                                                                     std::ifstream& img_fs, double* stamp_sec,
                                                                     size_t* img_id, std::vector<cv::Mat>* images)
{
    CHECK_NOTNULL(images);
    CHECK_NOTNULL(stamp_sec);
    CHECK_NOTNULL(img_id);
    images->clear();

    if (img_fs.peek() == '#' || img_fs.peek() == '\n')    // skip and empty line
    {
        img_fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    // load image
    img_fs >> *img_id >> *stamp_sec;
    if ((*img_id < first_frame_id_) ||
            (last_frame_id_ > 0 && *img_id >= last_frame_id_) || img_fs.eof())
    {
        img_fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return false;
    }

    size_t n_img = svo_->cams_->numCameras();
    for (size_t i = 0; i < n_img; i++)
    {
        std::string img_name;
        img_fs >> img_name;
        std::string img_filename(dataset_basedir + "/data/" + img_name);
        cv::Mat img(cv::imread(img_filename, 0));
        if (img.empty())
        {
            LOG(ERROR) << "Reading image " << img_filename << " failed. ";
        }

        if (*img_id > blackout_start_id_ && *img_id < blackout_end_id_)
        {
            img.setTo(cv::Scalar(0));
        }

        images->push_back(img);
    }
    img_fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    return true;
}

bool setImuPrior(const int64_t timestamp_nanoseconds) {
    if (svo_->getBundleAdjuster()) {
        // if we use backend, this will take care of setting priors
        if (!svo_->hasStarted()) {
            // when starting up, make sure we already have IMU measurements
            if (imu_handler_->getMeasurementsCopy().size() < 10u) {
                return false;
            }
        }
        return true;
    }

    if (imu_handler_ && !svo_->hasStarted() && set_initial_attitude_from_gravity_) {
        // set initial orientation
        aslam::Quaternion R_imu_world; // Specify the correct namespace
        if (imu_handler_->getInitialAttitude(
                timestamp_nanoseconds * svo::common::conversions::kNanoSecondsToSeconds,
                R_imu_world)) {
            VLOG(3) << "Set initial orientation from accelerometer measurements.";
            svo_->setRotationPrior(R_imu_world);
        } else {
            return false;
        }
    } else if (imu_handler_ && svo_->getLastFrames()) {
        // set incremental rotation prior
        aslam::Quaternion R_lastimu_newimu; // Specify the correct namespace
        if (imu_handler_->getRelativeRotationPrior(
                svo_->getLastFrames()->getMinTimestampNanoseconds() *
                svo::common::conversions::kNanoSecondsToSeconds,
                timestamp_nanoseconds * svo::common::conversions::kNanoSecondsToSeconds,
                false, R_lastimu_newimu)) {
            VLOG(3) << "Set incremental rotation prior from IMU.";
            svo_->setRotationIncrementPrior(R_lastimu_newimu);
        }
    }
    return true;
}

int main() {
    std::string dataset_dir = "data";
    std::string imu_filename(dataset_dir + "/imu.csv");
    std::string img_filename(dataset_dir + "/images.csv");
    std::ifstream img_fs(img_filename.c_str());
    while (img_fs.good() && !img_fs.eof()) {
        size_t img_id;
        double stamp_seconds;
        std::vector<cv::Mat> images;
        if (!loadNextImages(dataset_dir, img_fs, &stamp_seconds, &img_id, &images)) {
            continue;
        }
        int64_t stamp_nanoseconds = static_cast<int64_t>(stamp_seconds * 1e9);

//        setImuPrior(stamp_nanoseconds);
//        processImageBundle(images, stamp_nanoseconds);
//        publishResults(images, stamp_nanoseconds);
        break;
    }
}
