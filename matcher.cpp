#include <opencv2/opencv.hpp>
#include <glog/logging.h>

#include "localmap.h"

#include "matcher.h"

using namespace cv;
using namespace std;

struct Matcher::Data {
  TrackedPoint* point;
  int frame_idx;
};

Matcher::~Matcher() { }

Matcher::Matcher() : detector_(
        new GoodFeaturesToTrackDetector(200, 0.05, 10, 3)),
    extractor_(DescriptorExtractor::create("FREAK")),
    matcher_(DescriptorMatcher::create("BruteForce-Hamming")),
    descriptors_(new Mat()) {
}

bool Matcher::Track(const Mat& img, int camera, LocalMap* map) {
  LOG(ERROR) << "Track";

  CHECK_NE(img.size().width, 0);
  CHECK_NE(img.size().height, 0);
  CHECK_NOTNULL(map);

  double ransacReprojThreshold = 10;

  Frame* frame = map->AddFrame(map->cameras[camera].get());
  CHECK_NOTNULL(frame);

  // Detect corners.
  CHECK_EQ(CV_8U, img.type());
  vector<KeyPoint> keypoints1;
  detector_->detect(img, keypoints1, Mat());

  // Compute descriptors for each corner.
  Mat descriptors1;
  extractor_->compute(img, keypoints1, descriptors1);

  // Search for matching descriptors.
  vector<bool> matched;  // was the corner successfully matched.
  matched.resize(keypoints1.size());
  vector<DMatch> matches;
  if (descriptors_->size().height > 0 && descriptors1.size().height > 0) {
    LOG(ERROR) << "query " << descriptors1.size().height << " against "
               << descriptors_->size().height;
    CHECK_EQ(descriptors1.size().width, descriptors_->size().width);
    matcher_->match(descriptors1, *(descriptors_.get()), matches, Mat());
  }

  // Find the homography
  vector<Point2d> p1;
  vector<Point2d> p2;
  for (const auto& m : matches) {
    const Data& d = data_[m.trainIdx];

    matched[m.queryIdx] = true;

    Vector2d projected;
    if (!frame->Project(d.point->location(), &projected))
      continue;

    p1.push_back({(projected(0) + 1) / 2 * img.size().width, (projected(1) + 1) /2 * img.size().height });
    p2.push_back(keypoints1[m.queryIdx].pt);
  }
  
  if (p1.size() > 4) {
    Mat homog = findHomography(Mat(p1), Mat(p2), RANSAC, ransacReprojThreshold);

    // Accept points that match the homography.
    Mat p1t;
    perspectiveTransform(Mat(p1), p1t, homog);
    for (unsigned int i = 0; i < matches.size(); ++i) {
      if (norm(p2[i] - p1t.at<Point2d>(i, 0)) > ransacReprojThreshold) // outlier
        continue;

      Vector2d frame_point;
      frame_point << p2[i].x / img.size().width * 2 - 1, p2[i].y / img.size().height * 2 - 1;

      Observation o;
      o.frame_idx = frame->frame_num;
      o.pt = frame_point;

      data_[matches[i].trainIdx].point->AddObservation(o);
    }
  }


  const int size = 30;
  int grid[size][size] = { };

  

  // Add unmatched points as new descriptors.
  for (unsigned int i = 0; i < keypoints1.size(); ++i) {
    if (matched[i])
      continue;

    Vector2d frame_point;
    frame_point << keypoints1[i].pt.x / img.size().width * 2 - 1, keypoints1[i].pt.y / img.size().height * 2 - 1;
    Vector4d location = map->frame(frame->frame_num)->Unproject(frame_point, 500);
    TrackedPoint* point = map->AddPoint(location);

    point->AddObservation({frame_point, frame->frame_num});

    data_.push_back({point, frame->frame_num});

    descriptors_->push_back(descriptors1.row(i));
  }
    
  return true;
}
