#pragma once
#include "cvtask.hpp"

#define BUFFER_SIZE 512

using namespace ilsr;

class CvTrack : public CvTask {

public:
    CvTrack(cv::Mat frame0) {

        std::vector<cv::Rect> rois;

        while (rois.size() < 1) {
            selectROIs("CvTrack-ROI", frame0, rois, true);
        }
        cv::destroyWindow("CvTrack-ROI");

        std::vector<cv::Ptr<cv::Tracker>> algorithms;
        std::vector<cv::Rect2d>           targets;
        for (auto i = 0; i < rois.size(); i++) {
            targets.push_back(rois[i]);
            algorithms.push_back(cv::TrackerMedianFlow::create());
        }

        {
            // Writer lock
            boost::upgrade_lock<boost::shared_mutex>           lock(m_access);
            boost::upgrade_to_unique_lock<boost::shared_mutex> unique(lock);
            trackers = std::make_shared<cv::MultiTracker>();
            trackers->add(algorithms, frame0, targets);
        }
    }

    ~CvTrack() {
        close();
        {
            // Writer lock
            boost::upgrade_lock<boost::shared_mutex>           lock(m_access);
            boost::upgrade_to_unique_lock<boost::shared_mutex> unique(lock);
            trackers.reset();
        }
    }

protected:
    std::string get_name() {
        return "CvTrack";
    };

    void process(cv::Mat &frame) {

        std::vector<cv::Rect2d> objects;

        {
            // Reader lock
            boost::shared_lock<boost::shared_mutex> lock(m_access);
            if (!trackers) { return; }

            bool ok = trackers->update(frame);
            if (!ok) { return; }

            objects = trackers->getObjects();
        }

        cv::Mat to_show = frame.clone();

        for (auto box : objects) {
            char BUFFER[BUFFER_SIZE];
            cv::rectangle(to_show, box, cv::Scalar(255, 0, 0), 2, 1);
            snprintf(BUFFER, BUFFER_SIZE, "(%.1f, %.1f)", box.x, box.y);

            int         baseline  = 0;
            cv::Size    text_size = cv::getTextSize(BUFFER, fontFace, fontScale, thickness, &baseline);
            cv::Point2f origin    = box.tl();
            if (origin.y - text_size.height < 0) { origin.y = text_size.height + 2; }
            if (origin.x + text_size.width >= to_show.cols) { origin.x = to_show.cols - text_size.width - 2; }
            putText(to_show, BUFFER, origin, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("tracker", to_show);
        cv::waitKey(1);
    }

    void on_update(cv::Mat &frame) {}

private:
    std::shared_ptr<cv::MultiTracker> trackers;

    const int    fontFace  = cv::FONT_HERSHEY_SIMPLEX;
    const double fontScale = 0.8;
    const int    thickness = 2;
};
