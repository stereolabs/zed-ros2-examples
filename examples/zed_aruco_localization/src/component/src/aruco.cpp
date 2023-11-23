/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

License Agreement
For Open Source Computer Vision Library
(3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the names of the copyright holders nor the names of the contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#include "aruco.hpp"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/hal/hal.hpp>

namespace cv
{
namespace aruco
{
using namespace std;

/**
		*
		*/
DetectorParameters::DetectorParameters()
: adaptiveThreshWinSizeMin(3),
  adaptiveThreshWinSizeMax(23),
  adaptiveThreshWinSizeStep(10),
  adaptiveThreshConstant(7),
  minMarkerPerimeterRate(0.03),
  maxMarkerPerimeterRate(4.),
  polygonalApproxAccuracyRate(0.03),
  minCornerDistanceRate(0.05),
  minDistanceToBorder(3),
  minMarkerDistanceRate(0.05),
  doCornerRefinement(false),
  cornerRefinementWinSize(5),
  cornerRefinementMaxIterations(30),
  cornerRefinementMinAccuracy(0.1),
  markerBorderBits(1),
  perspectiveRemovePixelPerCell(4),
  perspectiveRemoveIgnoredMarginPerCell(0.13),
  maxErroneousBitsInBorderRate(0.35),
  minOtsuStdDev(5.0),
  errorCorrectionRate(0.6) {}


/**
		* @brief Convert input image to gray if it is a 3-channels image
		*/
static void _convertToGrey(InputArray _in, OutputArray _out)
{

  CV_Assert(_in.getMat().channels() == 1 || _in.getMat().channels() == 3);

  _out.create(_in.getMat().size(), CV_8UC1);
  if (_in.getMat().type() == CV_8UC3) {
    cvtColor(_in.getMat(), _out.getMat(), COLOR_BGR2GRAY);
  } else {
    _in.getMat().copyTo(_out);
  }
}


/**
		* @brief Threshold input image using adaptive thresholding
		*/
static void _threshold(InputArray _in, OutputArray _out, int winSize, double constant)
{

  CV_Assert(winSize >= 3);
  if (winSize % 2 == 0) {
    winSize++;                                           // win size must be odd
  }
  adaptiveThreshold(_in, _out, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, winSize, constant);
}


/**
		* @brief Given a tresholded image, find the contours, calculate their polygonal approximation
		* and take those that accomplish some conditions
		*/
static void _findMarkerContours(
  InputArray _in, vector<vector<Point2f>> & candidates,
  vector<vector<Point>> & contoursOut, double minPerimeterRate,
  double maxPerimeterRate, double accuracyRate,
  double minCornerDistanceRate, int minDistanceToBorder)
{

  CV_Assert(
    minPerimeterRate > 0 && maxPerimeterRate > 0 && accuracyRate > 0 &&
    minCornerDistanceRate >= 0 && minDistanceToBorder >= 0);

  // calculate maximum and minimum sizes in pixels
  unsigned int minPerimeterPixels =
    (unsigned int)(minPerimeterRate * max(_in.getMat().cols, _in.getMat().rows));
  unsigned int maxPerimeterPixels =
    (unsigned int)(maxPerimeterRate * max(_in.getMat().cols, _in.getMat().rows));

  Mat contoursImg;
  _in.getMat().copyTo(contoursImg);
  vector<vector<Point>> contours;
  findContours(contoursImg, contours, RETR_LIST, CHAIN_APPROX_NONE);
  // now filter list of contours
  for (unsigned int i = 0; i < contours.size(); i++) {
    // check perimeter
    if (contours[i].size() < minPerimeterPixels || contours[i].size() > maxPerimeterPixels) {
      continue;
    }

    // check is square and is convex
    vector<Point> approxCurve;
    approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * accuracyRate, true);
    if (approxCurve.size() != 4 || !isContourConvex(approxCurve)) {continue;}

    // check min distance between corners
    double minDistSq =
      max(contoursImg.cols, contoursImg.rows) * max(contoursImg.cols, contoursImg.rows);
    for (int j = 0; j < 4; j++) {
      double d = (double)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) *
        (double)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) +
        (double)(approxCurve[j].y - approxCurve[(j + 1) % 4].y) *
        (double)(approxCurve[j].y - approxCurve[(j + 1) % 4].y);
      minDistSq = min(minDistSq, d);
    }
    double minCornerDistancePixels = double(contours[i].size()) * minCornerDistanceRate;
    if (minDistSq < minCornerDistancePixels * minCornerDistancePixels) {continue;}

    // check if it is too near to the image border
    bool tooNearBorder = false;
    for (int j = 0; j < 4; j++) {
      if (approxCurve[j].x<minDistanceToBorder || approxCurve[j].y<minDistanceToBorder ||
        approxCurve[j].x> contoursImg.cols - 1 - minDistanceToBorder ||
        approxCurve[j].y> contoursImg.rows - 1 - minDistanceToBorder)
      {
        tooNearBorder = true;
      }
    }
    if (tooNearBorder) {continue;}

    // if it passes all the test, add to candidates vector
    vector<Point2f> currentCandidate;
    currentCandidate.resize(4);
    for (int j = 0; j < 4; j++) {
      currentCandidate[j] = Point2f((float)approxCurve[j].x, (float)approxCurve[j].y);
    }
    candidates.push_back(currentCandidate);
    contoursOut.push_back(contours[i]);
  }
}


/**
		* @brief Assure order of candidate corners is clockwise direction
		*/
static void _reorderCandidatesCorners(vector<vector<Point2f>> & candidates)
{

  for (unsigned int i = 0; i < candidates.size(); i++) {
    double dx1 = candidates[i][1].x - candidates[i][0].x;
    double dy1 = candidates[i][1].y - candidates[i][0].y;
    double dx2 = candidates[i][2].x - candidates[i][0].x;
    double dy2 = candidates[i][2].y - candidates[i][0].y;
    double crossProduct = (dx1 * dy2) - (dy1 * dx2);

    if (crossProduct < 0.0) {                             // not clockwise direction
      swap(candidates[i][1], candidates[i][3]);
    }
  }
}


/**
		* @brief Check candidates that are too close to each other and remove the smaller one
		*/
static void _filterTooCloseCandidates(
  const vector<vector<Point2f>> & candidatesIn,
  vector<vector<Point2f>> & candidatesOut,
  const vector<vector<Point>> & contoursIn,
  vector<vector<Point>> & contoursOut,
  double minMarkerDistanceRate)
{

  CV_Assert(minMarkerDistanceRate >= 0);

  vector<pair<int, int>> nearCandidates;
  for (unsigned int i = 0; i < candidatesIn.size(); i++) {
    for (unsigned int j = i + 1; j < candidatesIn.size(); j++) {

      int minimumPerimeter = min((int)contoursIn[i].size(), (int)contoursIn[j].size());

      // fc is the first corner considered on one of the markers, 4 combinatios are posible
      for (int fc = 0; fc < 4; fc++) {
        double distSq = 0;
        for (int c = 0; c < 4; c++) {
          // modC is the corner considering first corner is fc
          int modC = (c + fc) % 4;
          distSq += (candidatesIn[i][modC].x - candidatesIn[j][c].x) *
            (candidatesIn[i][modC].x - candidatesIn[j][c].x) +
            (candidatesIn[i][modC].y - candidatesIn[j][c].y) *
            (candidatesIn[i][modC].y - candidatesIn[j][c].y);
        }
        distSq /= 4.;

        // if mean square distance is too low, remove the smaller one of the two markers
        double minMarkerDistancePixels = double(minimumPerimeter) * minMarkerDistanceRate;
        if (distSq < minMarkerDistancePixels * minMarkerDistancePixels) {
          nearCandidates.push_back(pair<int, int>(i, j));
          break;
        }
      }
    }
  }

  // mark smaller one in pairs to remove
  vector<bool> toRemove(candidatesIn.size(), false);
  for (unsigned int i = 0; i < nearCandidates.size(); i++) {
    // if one of the marker has been already markerd to removed, dont need to do anything
    if (toRemove[nearCandidates[i].first] || toRemove[nearCandidates[i].second]) {continue;}
    size_t perimeter1 = contoursIn[nearCandidates[i].first].size();
    size_t perimeter2 = contoursIn[nearCandidates[i].second].size();
    if (perimeter1 > perimeter2) {
      toRemove[nearCandidates[i].second] = true;
    } else {
      toRemove[nearCandidates[i].first] = true;
    }
  }

  // remove extra candidates
  candidatesOut.clear();
  int totalRemaining = 0;
  for (unsigned int i = 0; i < toRemove.size(); i++) {
    if (!toRemove[i]) {totalRemaining++;}}
  candidatesOut.resize(totalRemaining);
  contoursOut.resize(totalRemaining);
  for (unsigned int i = 0, currIdx = 0; i < candidatesIn.size(); i++) {
    if (toRemove[i]) {continue;}
    candidatesOut[currIdx] = candidatesIn[i];
    contoursOut[currIdx] = contoursIn[i];
    currIdx++;
  }
}


/**
		* ParallelLoopBody class for the parallelization of the basic candidate detections using
		* different threhold window sizes. Called from function _detectInitialCandidates()
		*/
class DetectInitialCandidatesParallel : public ParallelLoopBody
{
public:
  DetectInitialCandidatesParallel(
    const Mat * _grey,
    vector<vector<vector<Point2f>>> * _candidatesArrays,
    vector<vector<vector<Point>>> * _contoursArrays,
    DetectorParameters * _params)
  : grey(_grey), candidatesArrays(_candidatesArrays), contoursArrays(_contoursArrays),
    params(_params) {}

  void operator()(const Range & range) const
  {
    const int begin = range.start;
    const int end = range.end;

    for (int i = begin; i < end; i++) {
      int currScale =
        params->adaptiveThreshWinSizeMin + i * params->adaptiveThreshWinSizeStep;
      // threshold
      Mat thresh;
      _threshold(*grey, thresh, currScale, params->adaptiveThreshConstant);

      // detect rectangles
      _findMarkerContours(
        thresh, (*candidatesArrays)[i], (*contoursArrays)[i],
        params->minMarkerPerimeterRate, params->maxMarkerPerimeterRate,
        params->polygonalApproxAccuracyRate, params->minCornerDistanceRate,
        params->minDistanceToBorder);
    }
  }

private:
  DetectInitialCandidatesParallel & operator=(const DetectInitialCandidatesParallel &);

  const Mat * grey;
  vector<vector<vector<Point2f>>> * candidatesArrays;
  vector<vector<vector<Point>>> * contoursArrays;
  DetectorParameters * params;
};


/**
		* @brief Initial steps on finding square candidates
		*/
static void _detectInitialCandidates(
  const Mat & grey, vector<vector<Point2f>> & candidates,
  vector<vector<Point>> & contours,
  DetectorParameters params)
{

  CV_Assert(params.adaptiveThreshWinSizeMin >= 3 && params.adaptiveThreshWinSizeMax >= 3);
  CV_Assert(params.adaptiveThreshWinSizeMax >= params.adaptiveThreshWinSizeMin);
  CV_Assert(params.adaptiveThreshWinSizeStep > 0);

  // number of window sizes (scales) to apply adaptive thresholding
  int nScales = (params.adaptiveThreshWinSizeMax - params.adaptiveThreshWinSizeMin) /
    params.adaptiveThreshWinSizeStep + 1;

  vector<vector<vector<Point2f>>> candidatesArrays(nScales);
  vector<vector<vector<Point>>> contoursArrays(nScales);

  ////for each value in the interval of thresholding window sizes
  // for(int i = 0; i < nScales; i++) {
  //    int currScale = params.adaptiveThreshWinSizeMin + i*params.adaptiveThreshWinSizeStep;
  //    // treshold
  //    Mat thresh;
  //    _threshold(grey, thresh, currScale, params.adaptiveThreshConstant);
  //    // detect rectangles
  //    _findMarkerContours(thresh, candidatesArrays[i], contoursArrays[i],
  // params.minMarkerPerimeterRate,
  //                        params.maxMarkerPerimeterRate, params.polygonalApproxAccuracyRate,
  //                        params.minCornerDistance, params.minDistanceToBorder);
  //}

  // this is the parallel call for the previous commented loop (result is equivalent)
  parallel_for_(
    Range(0, nScales), DetectInitialCandidatesParallel(
      &grey, &candidatesArrays,
      &contoursArrays, &params));

  // join candidates
  for (int i = 0; i < nScales; i++) {
    for (unsigned int j = 0; j < candidatesArrays[i].size(); j++) {
      candidates.push_back(candidatesArrays[i][j]);
      contours.push_back(contoursArrays[i][j]);
    }
  }
}


/**
		* @brief Detect square candidates in the input image
		*/
static void _detectCandidates(
  InputArray _image, OutputArrayOfArrays _candidates,
  OutputArrayOfArrays _contours, DetectorParameters params)
{

  Mat image = _image.getMat();
  CV_Assert(image.total() != 0);

  /// 1. CONVERT TO GRAY
  Mat grey;
  _convertToGrey(image, grey);

  vector<vector<Point2f>> candidates;
  vector<vector<Point>> contours;
  /// 2. DETECT FIRST SET OF CANDIDATES
  _detectInitialCandidates(grey, candidates, contours, params);

  /// 3. SORT CORNERS
  _reorderCandidatesCorners(candidates);

  /// 4. FILTER OUT NEAR CANDIDATE PAIRS
  vector<vector<Point2f>> candidatesOut;
  vector<vector<Point>> contoursOut;
  _filterTooCloseCandidates(
    candidates, candidatesOut, contours, contoursOut,
    params.minMarkerDistanceRate);

  // parse output
  _candidates.create((int)candidatesOut.size(), 1, CV_32FC2);
  _contours.create((int)contoursOut.size(), 1, CV_32SC2);
  for (int i = 0; i < (int)candidatesOut.size(); i++) {
    _candidates.create(4, 1, CV_32FC2, i, true);
    Mat m = _candidates.getMat(i);
    for (int j = 0; j < 4; j++) {
      m.ptr<Vec2f>(0)[j] = candidatesOut[i][j];
    }

    _contours.create((int)contoursOut[i].size(), 1, CV_32SC2, i, true);
    Mat c = _contours.getMat(i);
    for (unsigned int j = 0; j < contoursOut[i].size(); j++) {
      c.ptr<Point2i>()[j] = contoursOut[i][j];
    }
  }
}


/**
		* @brief Given an input image and candidate corners, extract the bits of the candidate, including
		* the border bits
		*/
static Mat _extractBits(
  InputArray _image, InputArray _corners, int markerSize,
  int markerBorderBits, int cellSize, double cellMarginRate,
  double minStdDevOtsu)
{

  CV_Assert(_image.getMat().channels() == 1);
  CV_Assert(_corners.total() == 4);
  CV_Assert(markerBorderBits > 0 && cellSize > 0 && cellMarginRate >= 0 && cellMarginRate <= 1);
  CV_Assert(minStdDevOtsu >= 0);

  // number of bits in the marker
  int markerSizeWithBorders = markerSize + 2 * markerBorderBits;
  int cellMarginPixels = int(cellMarginRate * cellSize);

  Mat resultImg;                       // marker image after removing perspective
  int resultImgSize = markerSizeWithBorders * cellSize;
  Mat resultImgCorners(4, 1, CV_32FC2);
  resultImgCorners.ptr<Point2f>(0)[0] = Point2f(0, 0);
  resultImgCorners.ptr<Point2f>(0)[1] = Point2f((float)resultImgSize - 1, 0);
  resultImgCorners.ptr<Point2f>(0)[2] =
    Point2f((float)resultImgSize - 1, (float)resultImgSize - 1);
  resultImgCorners.ptr<Point2f>(0)[3] = Point2f(0, (float)resultImgSize - 1);

  // remove perspective
  Mat transformation = getPerspectiveTransform(_corners, resultImgCorners);
  warpPerspective(
    _image, resultImg, transformation, Size(resultImgSize, resultImgSize),
    INTER_NEAREST);

  // output image containing the bits
  Mat bits(markerSizeWithBorders, markerSizeWithBorders, CV_8UC1, Scalar::all(0));

  // check if standard deviation is enough to apply Otsu
  // if not enough, it probably means all bits are the same color (black or white)
  Mat mean, stddev;
  // Remove some border just to avoid border noise from perspective transformation
  Mat innerRegion = resultImg.colRange(cellSize / 2, resultImg.cols - cellSize / 2)
    .rowRange(cellSize / 2, resultImg.rows - cellSize / 2);
  meanStdDev(innerRegion, mean, stddev);
  if (stddev.ptr<double>(0)[0] < minStdDevOtsu) {
    // all black or all white, depending on mean value
    if (mean.ptr<double>(0)[0] > 127) {
      bits.setTo(1);
    } else {
      bits.setTo(0);
    }
    return bits;
  }

  // now extract code, first threshold using Otsu
  threshold(resultImg, resultImg, 125, 255, THRESH_BINARY | THRESH_OTSU);

  // for each cell
  for (int y = 0; y < markerSizeWithBorders; y++) {
    for (int x = 0; x < markerSizeWithBorders; x++) {
      int Xstart = x * (cellSize) + cellMarginPixels;
      int Ystart = y * (cellSize) + cellMarginPixels;
      Mat square = resultImg(
        Rect(
          Xstart, Ystart, cellSize - 2 * cellMarginPixels,
          cellSize - 2 * cellMarginPixels));
      // count white pixels on each cell to assign its value
      unsigned int nZ = countNonZero(square);
      if (nZ > square.total() / 2) {bits.at<unsigned char>(y, x) = 1;}
    }
  }

  return bits;
}


/**
		* @brief Return number of erroneous bits in border, i.e. number of white bits in border.
		*/
static int _getBorderErrors(const Mat & bits, int markerSize, int borderSize)
{

  int sizeWithBorders = markerSize + 2 * borderSize;

  CV_Assert(markerSize > 0 && bits.cols == sizeWithBorders && bits.rows == sizeWithBorders);

  int totalErrors = 0;
  for (int y = 0; y < sizeWithBorders; y++) {
    for (int k = 0; k < borderSize; k++) {
      if (bits.ptr<unsigned char>(y)[k] != 0) {totalErrors++;}
      if (bits.ptr<unsigned char>(y)[sizeWithBorders - 1 - k] != 0) {totalErrors++;}
    }
  }
  for (int x = borderSize; x < sizeWithBorders - borderSize; x++) {
    for (int k = 0; k < borderSize; k++) {
      if (bits.ptr<unsigned char>(k)[x] != 0) {totalErrors++;}
      if (bits.ptr<unsigned char>(sizeWithBorders - 1 - k)[x] != 0) {totalErrors++;}
    }
  }
  return totalErrors;
}


/**
		* @brief Tries to identify one candidate given the dictionary
		*/
static bool _identifyOneCandidate(
  const Dictionary & dictionary, InputArray _image,
  InputOutputArray _corners, int & idx, DetectorParameters params)
{

  CV_Assert(_corners.total() == 4);
  CV_Assert(_image.getMat().total() != 0);
  CV_Assert(params.markerBorderBits > 0);

  // get bits
  Mat candidateBits =
    _extractBits(
    _image, _corners, dictionary.markerSize, params.markerBorderBits,
    params.perspectiveRemovePixelPerCell,
    params.perspectiveRemoveIgnoredMarginPerCell, params.minOtsuStdDev);

  // analyze border bits
  int maximumErrorsInBorder =
    int(dictionary.markerSize * dictionary.markerSize * params.maxErroneousBitsInBorderRate);
  int borderErrors =
    _getBorderErrors(candidateBits, dictionary.markerSize, params.markerBorderBits);
  if (borderErrors > maximumErrorsInBorder) {
    return false;                                                               // border is wrong

  }
  // take only inner bits
  Mat onlyBits =
    candidateBits.rowRange(
    params.markerBorderBits,
    candidateBits.rows - params.markerBorderBits)
    .colRange(params.markerBorderBits, candidateBits.rows - params.markerBorderBits);

  // try to indentify the marker
  int rotation;
  if (!dictionary.identify(onlyBits, idx, rotation, params.errorCorrectionRate)) {
    return false;
  } else {
    // shift corner positions to the correct rotation
    if (rotation != 0) {
      Mat copyPoints = _corners.getMat().clone();
      for (int j = 0; j < 4; j++) {
        _corners.getMat().ptr<Point2f>(0)[j] =
          copyPoints.ptr<Point2f>(0)[(j + 4 - rotation) % 4];
      }
    }
    return true;
  }
}


/**
		* ParallelLoopBody class for the parallelization of the marker identification step
		* Called from function _identifyCandidates()
		*/
class IdentifyCandidatesParallel : public ParallelLoopBody
{
public:
  IdentifyCandidatesParallel(
    const Mat * _grey, InputArrayOfArrays _candidates,
    InputArrayOfArrays _contours, const Dictionary * _dictionary,
    vector<int> * _idsTmp, vector<char> * _validCandidates,
    DetectorParameters * _params)
  : grey(_grey), candidates(_candidates), contours(_contours), dictionary(_dictionary),
    idsTmp(_idsTmp), validCandidates(_validCandidates), params(_params) {}

  void operator()(const Range & range) const
  {
    const int begin = range.start;
    const int end = range.end;

    for (int i = begin; i < end; i++) {
      int currId;
      Mat currentCandidate = candidates.getMat(i);
      if (_identifyOneCandidate(*dictionary, *grey, currentCandidate, currId, *params)) {
        (*validCandidates)[i] = 1;
        (*idsTmp)[i] = currId;
      }
    }
  }

private:
  IdentifyCandidatesParallel & operator=(const IdentifyCandidatesParallel &);                      // to quiet MSVC

  const Mat * grey;
  InputArrayOfArrays candidates, contours;
  const Dictionary * dictionary;
  vector<int> * idsTmp;
  vector<char> * validCandidates;
  DetectorParameters * params;
};


/**
		* @brief Identify square candidates according to a marker dictionary
		*/
static void _identifyCandidates(
  InputArray _image, InputArrayOfArrays _candidates,
  InputArrayOfArrays _contours, const Dictionary & dictionary,
  OutputArrayOfArrays _accepted, OutputArray _ids,
  DetectorParameters params,
  OutputArrayOfArrays _rejected = noArray())
{

  int ncandidates = (int)_candidates.total();

  vector<Mat> accepted;
  vector<Mat> rejected;
  vector<int> ids;

  CV_Assert(_image.getMat().total() != 0);

  Mat grey;
  _convertToGrey(_image.getMat(), grey);

  vector<int> idsTmp(ncandidates, -1);
  vector<char> validCandidates(ncandidates, 0);

  //// Analyze each of the candidates
  // for (int i = 0; i < ncandidates; i++) {
  //    int currId = i;
  //    Mat currentCandidate = _candidates.getMat(i);
  //    if (_identifyOneCandidate(dictionary, grey, currentCandidate, currId, params)) {
  //        validCandidates[i] = 1;
  //        idsTmp[i] = currId;
  //    }
  //}

  // this is the parallel call for the previous commented loop (result is equivalent)
  parallel_for_(
    Range(0, ncandidates),
    IdentifyCandidatesParallel(
      &grey, _candidates, _contours, &dictionary, &idsTmp,
      &validCandidates, &params));

  for (int i = 0; i < ncandidates; i++) {
    if (validCandidates[i] == 1) {
      accepted.push_back(_candidates.getMat(i));
      ids.push_back(idsTmp[i]);
    } else {
      rejected.push_back(_candidates.getMat(i));
    }
  }

  // parse output
  _accepted.create((int)accepted.size(), 1, CV_32FC2);
  for (unsigned int i = 0; i < accepted.size(); i++) {
    _accepted.create(4, 1, CV_32FC2, i, true);
    Mat m = _accepted.getMat(i);
    accepted[i].copyTo(m);
  }

  _ids.create((int)ids.size(), 1, CV_32SC1);
  for (unsigned int i = 0; i < ids.size(); i++) {
    _ids.getMat().ptr<int>(0)[i] = ids[i];
  }

  if (_rejected.needed()) {
    _rejected.create((int)rejected.size(), 1, CV_32FC2);
    for (unsigned int i = 0; i < rejected.size(); i++) {
      _rejected.create(4, 1, CV_32FC2, i, true);
      Mat m = _rejected.getMat(i);
      rejected[i].copyTo(m);
    }
  }
}


/**
		* @brief Final filter of markers after its identification
		*/
static void _filterDetectedMarkers(
  InputArrayOfArrays _inCorners, InputArray _inIds,
  OutputArrayOfArrays _outCorners, OutputArray _outIds)
{

  CV_Assert(_inCorners.total() == _inIds.total());
  if (_inCorners.total() == 0) {return;}

  // mark markers that will be removed
  vector<bool> toRemove(_inCorners.total(), false);
  bool atLeastOneRemove = false;

  // remove repeated markers with same id, if one contains the other (doble border bug)
  for (unsigned int i = 0; i < _inCorners.total() - 1; i++) {
    for (unsigned int j = i + 1; j < _inCorners.total(); j++) {
      if (_inIds.getMat().ptr<int>(0)[i] != _inIds.getMat().ptr<int>(0)[j]) {continue;}

      // check if first marker is inside second
      bool inside = true;
      for (unsigned int p = 0; p < 4; p++) {
        Point2f point = _inCorners.getMat(j).ptr<Point2f>(0)[p];
        if (pointPolygonTest(_inCorners.getMat(i), point, false) < 0) {
          inside = false;
          break;
        }
      }
      if (inside) {
        toRemove[j] = true;
        atLeastOneRemove = true;
        continue;
      }

      // check the second marker
      inside = true;
      for (unsigned int p = 0; p < 4; p++) {
        Point2f point = _inCorners.getMat(i).ptr<Point2f>(0)[p];
        if (pointPolygonTest(_inCorners.getMat(j), point, false) < 0) {
          inside = false;
          break;
        }
      }
      if (inside) {
        toRemove[i] = true;
        atLeastOneRemove = true;
        continue;
      }
    }
  }

  // parse output
  if (atLeastOneRemove) {
    vector<Mat> filteredCorners;
    vector<int> filteredIds;

    for (unsigned int i = 0; i < toRemove.size(); i++) {
      if (!toRemove[i]) {
        filteredCorners.push_back(_inCorners.getMat(i).clone());
        filteredIds.push_back(_inIds.getMat().ptr<int>(0)[i]);
      }
    }

    _outIds.create((int)filteredIds.size(), 1, CV_32SC1);
    for (unsigned int i = 0; i < filteredIds.size(); i++) {
      _outIds.getMat().ptr<int>(0)[i] = filteredIds[i];
    }

    _outCorners.create((int)filteredCorners.size(), 1, CV_32FC2);
    for (unsigned int i = 0; i < filteredCorners.size(); i++) {
      _outCorners.create(4, 1, CV_32FC2, i, true);
      filteredCorners[i].copyTo(_outCorners.getMat(i));
    }
  }
}


/**
		* @brief Return object points for the system centered in a single marker, given the marker length
		*/
static void _getSingleMarkerObjectPoints(float markerLength, OutputArray _objPoints)
{

  CV_Assert(markerLength > 0);

  _objPoints.create(4, 1, CV_32FC3);
  Mat objPoints = _objPoints.getMat();
  // set coordinate system in the middle of the marker, with Z pointing out
  objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
  objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
  objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
  objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
}


/**
		* ParallelLoopBody class for the parallelization of the marker corner subpixel refinement
		* Called from function detectMarkers()
		*/
class MarkerSubpixelParallel : public ParallelLoopBody
{
public:
  MarkerSubpixelParallel(
    const Mat * _grey, OutputArrayOfArrays _corners,
    DetectorParameters * _params)
  : grey(_grey), corners(_corners), params(_params) {}

  void operator()(const Range & range) const
  {
    const int begin = range.start;
    const int end = range.end;

    for (int i = begin; i < end; i++) {
      cornerSubPix(
        *grey, corners.getMat(i),
        Size(params->cornerRefinementWinSize, params->cornerRefinementWinSize),
        Size(-1, -1), TermCriteria(
          TermCriteria::MAX_ITER | TermCriteria::EPS,
          params->cornerRefinementMaxIterations,
          params->cornerRefinementMinAccuracy));
    }
  }

private:
  MarkerSubpixelParallel & operator=(const MarkerSubpixelParallel &);                      // to quiet MSVC

  const Mat * grey;
  OutputArrayOfArrays corners;
  DetectorParameters * params;
};


/**
		*/
void detectMarkers(
  InputArray _image, Dictionary dictionary, OutputArrayOfArrays _corners,
  OutputArray _ids, DetectorParameters params,
  OutputArrayOfArrays _rejectedImgPoints)
{

  CV_Assert(_image.getMat().total() != 0);

  Mat grey;
  _convertToGrey(_image.getMat(), grey);

  /// STEP 1: Detect marker candidates
  vector<vector<Point2f>> candidates;
  vector<vector<Point>> contours;
  _detectCandidates(grey, candidates, contours, params);

  /// STEP 2: Check candidate codification (identify markers)
  _identifyCandidates(
    grey, candidates, contours, dictionary, _corners, _ids, params,
    _rejectedImgPoints);

  /// STEP 3: Filter detected markers;
  _filterDetectedMarkers(_corners, _ids, _corners, _ids);

  /// STEP 4: Corner refinement
  if (params.doCornerRefinement) {
    CV_Assert(
      params.cornerRefinementWinSize > 0 && params.cornerRefinementMaxIterations > 0 &&
      params.cornerRefinementMinAccuracy > 0);

    //// do corner refinement for each of the detected markers
    // for (unsigned int i = 0; i < _corners.total(); i++) {
    //    cornerSubPix(grey, _corners.getMat(i),
    //                 Size(params.cornerRefinementWinSize, params.cornerRefinementWinSize),
    //                 Size(-1, -1), TermCriteria(TermCriteria::MAX_ITER | TermCriteria::EPS,
    //                                            params.cornerRefinementMaxIterations,
    //                                            params.cornerRefinementMinAccuracy));
    //}

    // this is the parallel call for the previous commented loop (result is equivalent)
    parallel_for_(
      Range(0, (int)_corners.total()),
      MarkerSubpixelParallel(&grey, _corners, &params));
  }
}


/**
		* ParallelLoopBody class for the parallelization of the single markers pose estimation
		* Called from function estimatePoseSingleMarkers()
		*/
class SinglePoseEstimationParallel : public ParallelLoopBody
{
public:
  SinglePoseEstimationParallel(
    Mat & _markerObjPoints, InputArrayOfArrays _corners,
    InputArray _cameraMatrix, InputArray _distCoeffs,
    Mat & _rvecs, Mat & _tvecs)
  : markerObjPoints(_markerObjPoints), corners(_corners), cameraMatrix(_cameraMatrix),
    distCoeffs(_distCoeffs), rvecs(_rvecs), tvecs(_tvecs) {}

  void operator()(const Range & range) const
  {
    const int begin = range.start;
    const int end = range.end;

    for (int i = begin; i < end; i++) {
      solvePnP(
        markerObjPoints, corners.getMat(i), cameraMatrix, distCoeffs,
        rvecs.at<Vec3d>(0, i), tvecs.at<Vec3d>(0, i));
    }
  }

private:
  SinglePoseEstimationParallel & operator=(const SinglePoseEstimationParallel &);                      // to quiet MSVC

  Mat & markerObjPoints;
  InputArrayOfArrays corners;
  InputArray cameraMatrix, distCoeffs;
  Mat & rvecs, tvecs;
};


/**
		*/
void estimatePoseSingleMarkers(
  InputArrayOfArrays _corners, float markerLength,
  InputArray _cameraMatrix, InputArray _distCoeffs,
  OutputArrayOfArrays _rvecs, OutputArrayOfArrays _tvecs)
{

  CV_Assert(markerLength > 0);

  Mat markerObjPoints;
  _getSingleMarkerObjectPoints(markerLength, markerObjPoints);
  int nMarkers = (int)_corners.total();
  _rvecs.create(nMarkers, 1, CV_64FC3);
  _tvecs.create(nMarkers, 1, CV_64FC3);

  Mat rvecs = _rvecs.getMat(), tvecs = _tvecs.getMat();

  //// for each marker, calculate its pose
  // for (int i = 0; i < nMarkers; i++) {
  //    solvePnP(markerObjPoints, _corners.getMat(i), _cameraMatrix, _distCoeffs,
  //             _rvecs.getMat(i), _tvecs.getMat(i));
  //}

  // this is the parallel call for the previous commented loop (result is equivalent)
  parallel_for_(
    Range(0, nMarkers),
    SinglePoseEstimationParallel(
      markerObjPoints, _corners, _cameraMatrix,
      _distCoeffs, rvecs, tvecs));
}


/**
		* @brief Given a board configuration and a set of detected markers, returns the corresponding
		* image points and object points to call solvePnP
		*/
static void _getBoardObjectAndImagePoints(
  const Board & board, InputArray _detectedIds,
  InputArrayOfArrays _detectedCorners,
  OutputArray _imgPoints, OutputArray _objPoints)
{

  CV_Assert(board.ids.size() == board.objPoints.size());
  CV_Assert(_detectedIds.total() == _detectedCorners.total());

  int nDetectedMarkers = (int)_detectedIds.total();

  vector<Point3f> objPnts;
  objPnts.reserve(nDetectedMarkers);

  vector<Point2f> imgPnts;
  imgPnts.reserve(nDetectedMarkers);

  // look for detected markers that belong to the board and get their information
  for (int i = 0; i < nDetectedMarkers; i++) {
    int currentId = _detectedIds.getMat().ptr<int>(0)[i];
    for (unsigned int j = 0; j < board.ids.size(); j++) {
      if (currentId == board.ids[j]) {
        for (int p = 0; p < 4; p++) {
          objPnts.push_back(board.objPoints[j][p]);
          imgPnts.push_back(_detectedCorners.getMat(i).ptr<Point2f>(0)[p]);
        }
      }
    }
  }

  // create output
  _objPoints.create((int)objPnts.size(), 1, CV_32FC3);
  for (unsigned int i = 0; i < objPnts.size(); i++) {
    _objPoints.getMat().ptr<Point3f>(0)[i] = objPnts[i];
  }

  _imgPoints.create((int)objPnts.size(), 1, CV_32FC2);
  for (unsigned int i = 0; i < imgPnts.size(); i++) {
    _imgPoints.getMat().ptr<Point2f>(0)[i] = imgPnts[i];
  }
}


/**
		* Project board markers that are not included in the list of detected markers
		*/
static void _projectUndetectedMarkers(
  const Board & board, InputOutputArrayOfArrays _detectedCorners,
  InputOutputArray _detectedIds, InputArray _cameraMatrix,
  InputArray _distCoeffs,
  OutputArrayOfArrays _undetectedMarkersProjectedCorners,
  OutputArray _undetectedMarkersIds)
{

  // first estimate board pose with the current avaible markers
  Mat rvec, tvec;
  int boardDetectedMarkers;
  boardDetectedMarkers = aruco::estimatePoseBoard(
    _detectedCorners, _detectedIds, board,
    _cameraMatrix, _distCoeffs, rvec, tvec);

  // at least one marker from board so rvec and tvec are valid
  if (boardDetectedMarkers == 0) {return;}

  // search undetected markers and project them using the previous pose
  vector<vector<Point2f>> undetectedCorners;
  vector<int> undetectedIds;
  for (unsigned int i = 0; i < board.ids.size(); i++) {
    int foundIdx = -1;
    for (unsigned int j = 0; j < _detectedIds.total(); j++) {
      if (board.ids[i] == _detectedIds.getMat().ptr<int>()[j]) {
        foundIdx = j;
        break;
      }
    }

    // not detected
    if (foundIdx == -1) {
      undetectedCorners.push_back(vector<Point2f>());
      undetectedIds.push_back(board.ids[i]);
      projectPoints(
        board.objPoints[i], rvec, tvec, _cameraMatrix, _distCoeffs,
        undetectedCorners.back());
    }
  }


  // parse output
  _undetectedMarkersIds.create((int)undetectedIds.size(), 1, CV_32SC1);
  for (unsigned int i = 0; i < undetectedIds.size(); i++) {
    _undetectedMarkersIds.getMat().ptr<int>(0)[i] = undetectedIds[i];
  }

  _undetectedMarkersProjectedCorners.create((int)undetectedCorners.size(), 1, CV_32FC2);
  for (unsigned int i = 0; i < undetectedCorners.size(); i++) {
    _undetectedMarkersProjectedCorners.create(4, 1, CV_32FC2, i, true);
    for (int j = 0; j < 4; j++) {
      _undetectedMarkersProjectedCorners.getMat(i).ptr<Point2f>()[j] =
        undetectedCorners[i][j];
    }
  }
}


/**
		* Interpolate board markers that are not included in the list of detected markers using
		* global homography
		*/
static void _projectUndetectedMarkers(
  const Board & board, InputOutputArrayOfArrays _detectedCorners,
  InputOutputArray _detectedIds,
  OutputArrayOfArrays _undetectedMarkersProjectedCorners,
  OutputArray _undetectedMarkersIds)
{


  // check board points are in the same plane, if not, global homography cannot be applied
  CV_Assert(board.objPoints.size() > 0);
  CV_Assert(board.objPoints[0].size() > 0);
  float boardZ = board.objPoints[0][0].z;
  for (unsigned int i = 0; i < board.objPoints.size(); i++) {
    for (unsigned int j = 0; j < board.objPoints[i].size(); j++) {
      CV_Assert(boardZ == board.objPoints[i][j].z);
    }
  }

  vector<Point2f> detectedMarkersObj2DAll;                         // Object coordinates (without Z) of all the detected
  // marker corners in a single vector
  vector<Point2f> imageCornersAll;                         // Image corners of all detected markers in a single vector
  vector<vector<Point2f>> undetectedMarkersObj2D;                           // Object coordinates (without Z) of all
  // missing markers in different vectors
  vector<int> undetectedMarkersIds;                         // ids of missing markers
  // find markers included in board, and missing markers from board. Fill the previous vectors
  for (unsigned int j = 0; j < board.ids.size(); j++) {
    bool found = false;
    for (unsigned int i = 0; i < _detectedIds.total(); i++) {
      if (_detectedIds.getMat().ptr<int>()[i] == board.ids[j]) {
        for (int c = 0; c < 4; c++) {
          imageCornersAll.push_back(_detectedCorners.getMat(i).ptr<Point2f>()[c]);
          detectedMarkersObj2DAll.push_back(
            Point2f(board.objPoints[j][c].x, board.objPoints[j][c].y));
        }
        found = true;
        break;
      }
    }
    if (!found) {
      undetectedMarkersObj2D.push_back(vector<Point2f>());
      for (int c = 0; c < 4; c++) {
        undetectedMarkersObj2D.back().push_back(
          Point2f(board.objPoints[j][c].x, board.objPoints[j][c].y));
      }
      undetectedMarkersIds.push_back(board.ids[j]);
    }
  }
  if (imageCornersAll.size() == 0) {return;}

  // get homography from detected markers
  Mat transformation = findHomography(detectedMarkersObj2DAll, imageCornersAll);

  _undetectedMarkersProjectedCorners.create((int)undetectedMarkersIds.size(), 1, CV_32FC2);

  // for each undetected marker, apply transformation
  for (unsigned int i = 0; i < undetectedMarkersObj2D.size(); i++) {
    Mat projectedMarker;
    perspectiveTransform(undetectedMarkersObj2D[i], projectedMarker, transformation);

    _undetectedMarkersProjectedCorners.create(4, 1, CV_32FC2, i, true);
    projectedMarker.copyTo(_undetectedMarkersProjectedCorners.getMat(i));
  }

  _undetectedMarkersIds.create((int)undetectedMarkersIds.size(), 1, CV_32SC1);
  for (unsigned int i = 0; i < undetectedMarkersIds.size(); i++) {
    _undetectedMarkersIds.getMat().ptr<int>(0)[i] = undetectedMarkersIds[i];
  }
}


/**
		*/
void refineDetectedMarkers(
  InputArray _image, const Board & board,
  InputOutputArrayOfArrays _detectedCorners, InputOutputArray _detectedIds,
  InputOutputArray _rejectedCorners, InputArray _cameraMatrix,
  InputArray _distCoeffs, float minRepDistance, float errorCorrectionRate,
  bool checkAllOrders, OutputArray _recoveredIdxs,
  DetectorParameters params)
{

  CV_Assert(minRepDistance > 0);

  if (_detectedIds.total() == 0 || _rejectedCorners.total() == 0) {return;}

  // get projections of missing markers in the board
  vector<vector<Point2f>> undetectedMarkersCorners;
  vector<int> undetectedMarkersIds;
  if (_cameraMatrix.total() != 0) {
    // reproject based on camera projection model
    _projectUndetectedMarkers(
      board, _detectedCorners, _detectedIds, _cameraMatrix, _distCoeffs,
      undetectedMarkersCorners, undetectedMarkersIds);

  } else {
    // reproject based on global homography
    _projectUndetectedMarkers(
      board, _detectedCorners, _detectedIds, undetectedMarkersCorners,
      undetectedMarkersIds);
  }

  // list of missing markers indicating if they have been assigned to a candidate
  vector<bool> alreadyIdentified(_rejectedCorners.total(), false);

  // maximum bits that can be corrected
  int maxCorrectionRecalculated =
    int(double(board.dictionary.maxCorrectionBits) * errorCorrectionRate);

  Mat grey;
  _convertToGrey(_image, grey);

  // vector of final detected marker corners and ids
  vector<Mat> finalAcceptedCorners;
  vector<int> finalAcceptedIds;
  // fill with the current markers
  finalAcceptedCorners.resize(_detectedCorners.total());
  finalAcceptedIds.resize(_detectedIds.total());
  for (unsigned int i = 0; i < _detectedIds.total(); i++) {
    finalAcceptedCorners[i] = _detectedCorners.getMat(i).clone();
    finalAcceptedIds[i] = _detectedIds.getMat().ptr<int>()[i];
  }
  vector<int> recoveredIdxs;                         // original indexes of accepted markers in _rejectedCorners

  // for each missing marker, try to find a correspondence
  for (unsigned int i = 0; i < undetectedMarkersIds.size(); i++) {

    // best match at the moment
    int closestCandidateIdx = -1;
    double closestCandidateDistance = minRepDistance * minRepDistance + 1;
    Mat closestRotatedMarker;

    for (unsigned int j = 0; j < _rejectedCorners.total(); j++) {
      if (alreadyIdentified[j]) {continue;}

      // check distance
      double minDistance = closestCandidateDistance + 1;
      bool valid = false;
      int validRot = 0;
      for (int c = 0; c < 4; c++) {                                   // first corner in rejected candidate
        double currentMaxDistance = 0;
        for (int k = 0; k < 4; k++) {
          Point2f rejCorner = _rejectedCorners.getMat(j).ptr<Point2f>()[(c + k) % 4];
          Point2f distVector = undetectedMarkersCorners[i][k] - rejCorner;
          double cornerDist = distVector.x * distVector.x + distVector.y * distVector.y;
          currentMaxDistance = max(currentMaxDistance, cornerDist);
        }
        // if distance is better than current best distance
        if (currentMaxDistance < closestCandidateDistance) {
          valid = true;
          validRot = c;
          minDistance = currentMaxDistance;
        }
        if (!checkAllOrders) {break;}
      }

      if (!valid) {continue;}

      // apply rotation
      Mat rotatedMarker;
      if (checkAllOrders) {
        rotatedMarker = Mat(4, 1, CV_32FC2);
        for (int c = 0; c < 4; c++) {
          rotatedMarker.ptr<Point2f>()[c] =
            _rejectedCorners.getMat(j).ptr<Point2f>()[(c + 4 + validRot) % 4];
        }
      } else {rotatedMarker = _rejectedCorners.getMat(j);}

      // last filter, check if inner code is close enough to the assigned marker code
      int codeDistance = 0;
      // if errorCorrectionRate, dont check code
      if (errorCorrectionRate >= 0) {

        // extract bits
        Mat bits = _extractBits(
          grey, rotatedMarker, board.dictionary.markerSize, params.markerBorderBits,
          params.perspectiveRemovePixelPerCell,
          params.perspectiveRemoveIgnoredMarginPerCell, params.minOtsuStdDev);

        Mat onlyBits =
          bits.rowRange(params.markerBorderBits, bits.rows - params.markerBorderBits)
          .colRange(params.markerBorderBits, bits.rows - params.markerBorderBits);

        codeDistance =
          board.dictionary.getDistanceToId(onlyBits, undetectedMarkersIds[i], false);
      }

      // if everythin is ok, assign values to current best match
      if (errorCorrectionRate < 0 || codeDistance < maxCorrectionRecalculated) {
        closestCandidateIdx = j;
        closestCandidateDistance = minDistance;
        closestRotatedMarker = rotatedMarker;
      }
    }

    // if at least one good match, we have rescue the missing marker
    if (closestCandidateIdx >= 0) {

      // subpixel refinement
      if (params.doCornerRefinement) {
        CV_Assert(
          params.cornerRefinementWinSize > 0 &&
          params.cornerRefinementMaxIterations > 0 &&
          params.cornerRefinementMinAccuracy > 0);
        cornerSubPix(
          grey, closestRotatedMarker,
          Size(params.cornerRefinementWinSize, params.cornerRefinementWinSize),
          Size(-1, -1), TermCriteria(
            TermCriteria::MAX_ITER | TermCriteria::EPS,
            params.cornerRefinementMaxIterations,
            params.cornerRefinementMinAccuracy));
      }

      // remove from rejected
      alreadyIdentified[closestCandidateIdx] = true;

      // add to detected
      finalAcceptedCorners.push_back(closestRotatedMarker);
      finalAcceptedIds.push_back(undetectedMarkersIds[i]);

      // add the original index of the candidate
      recoveredIdxs.push_back(closestCandidateIdx);
    }
  }

  // parse output
  if (finalAcceptedIds.size() != _detectedIds.total()) {
    _detectedCorners.clear();
    _detectedIds.clear();

    // parse output
    _detectedIds.create((int)finalAcceptedIds.size(), 1, CV_32SC1);
    for (unsigned int i = 0; i < finalAcceptedIds.size(); i++) {
      _detectedIds.getMat().ptr<int>(0)[i] = finalAcceptedIds[i];
    }

    _detectedCorners.create((int)finalAcceptedCorners.size(), 1, CV_32FC2);
    for (unsigned int i = 0; i < finalAcceptedCorners.size(); i++) {
      _detectedCorners.create(4, 1, CV_32FC2, i, true);
      for (int j = 0; j < 4; j++) {
        _detectedCorners.getMat(i).ptr<Point2f>()[j] =
          finalAcceptedCorners[i].ptr<Point2f>()[j];
      }
    }

    // recalculate _rejectedCorners based on alreadyIdentified
    vector<Mat> finalRejected;
    for (unsigned int i = 0; i < alreadyIdentified.size(); i++) {
      if (!alreadyIdentified[i]) {
        finalRejected.push_back(_rejectedCorners.getMat(i).clone());
      }
    }

    _rejectedCorners.clear();
    _rejectedCorners.create((int)finalRejected.size(), 1, CV_32FC2);
    for (unsigned int i = 0; i < finalRejected.size(); i++) {
      _rejectedCorners.create(4, 1, CV_32FC2, i, true);
      for (int j = 0; j < 4; j++) {
        _rejectedCorners.getMat(i).ptr<Point2f>()[j] =
          finalRejected[i].ptr<Point2f>()[j];
      }
    }

    if (_recoveredIdxs.needed()) {
      _recoveredIdxs.create((int)recoveredIdxs.size(), 1, CV_32SC1);
      for (unsigned int i = 0; i < recoveredIdxs.size(); i++) {
        _recoveredIdxs.getMat().ptr<int>()[i] = recoveredIdxs[i];
      }
    }
  }
}


/**
		*/
int estimatePoseBoard(
  InputArrayOfArrays _corners, InputArray _ids, const Board & board,
  InputArray _cameraMatrix, InputArray _distCoeffs, OutputArray _rvec,
  OutputArray _tvec)
{

  CV_Assert(_corners.total() == _ids.total());

  // get object and image points for the solvePnP function
  Mat objPoints, imgPoints;
  _getBoardObjectAndImagePoints(board, _ids, _corners, imgPoints, objPoints);

  CV_Assert(imgPoints.total() == objPoints.total());

  if (objPoints.total() == 0) {                     // 0 of the detected markers in board
    return 0;
  }

  _rvec.create(3, 1, CV_64FC1);
  _tvec.create(3, 1, CV_64FC1);
  solvePnP(objPoints, imgPoints, _cameraMatrix, _distCoeffs, _rvec, _tvec);

  // divide by four since all the four corners are concatenated in the array for each marker
  return (int)objPoints.total() / 4;
}


/**
		*/
void GridBoard::draw(Size outSize, OutputArray _img, int marginSize, int borderBits)
{
  aruco::drawPlanarBoard((*this), outSize, _img, marginSize, borderBits);
}


/**
		*/
GridBoard GridBoard::create(
  int markersX, int markersY, float markerLength, float markerSeparation,
  Dictionary _dictionary)
{

  GridBoard res;

  CV_Assert(markersX > 0 && markersY > 0 && markerLength > 0 && markerSeparation > 0);

  res._markersX = markersX;
  res._markersY = markersY;
  res._markerLength = markerLength;
  res._markerSeparation = markerSeparation;
  res.dictionary = _dictionary;

  int totalMarkers = markersX * markersY;
  res.ids.resize(totalMarkers);
  res.objPoints.reserve(totalMarkers);

  // fill ids with first identifiers
  for (int i = 0; i < totalMarkers; i++) {
    res.ids[i] = i;
  }

  // calculate Board objPoints
  float maxY = (float)markersY * markerLength + (markersY - 1) * markerSeparation;
  for (int y = 0; y < markersY; y++) {
    for (int x = 0; x < markersX; x++) {
      vector<Point3f> corners;
      corners.resize(4);
      corners[0] = Point3f(
        x * (markerLength + markerSeparation),
        maxY - y * (markerLength + markerSeparation), 0);
      corners[1] = corners[0] + Point3f(markerLength, 0, 0);
      corners[2] = corners[0] + Point3f(markerLength, -markerLength, 0);
      corners[3] = corners[0] + Point3f(0, -markerLength, 0);
      res.objPoints.push_back(corners);
    }
  }

  return res;
}


/**
		*/
void drawDetectedMarkers(
  InputOutputArray _image, InputArrayOfArrays _corners,
  InputArray _ids, Scalar borderColor)
{


  CV_Assert(
    _image.getMat().total() != 0 &&
    (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
  CV_Assert((_corners.total() == _ids.total()) || _ids.total() == 0);

  // calculate colors
  Scalar textColor, cornerColor;
  textColor = cornerColor = borderColor;
  swap(textColor.val[0], textColor.val[1]);                           // text color just sawp G and R
  swap(cornerColor.val[1], cornerColor.val[2]);                       // corner color just sawp G and B

  int nMarkers = (int)_corners.total();
  for (int i = 0; i < nMarkers; i++) {
    Mat currentMarker = _corners.getMat(i);
    CV_Assert(currentMarker.total() == 4 && currentMarker.type() == CV_32FC2);

    // draw marker sides
    for (int j = 0; j < 4; j++) {
      Point2f p0, p1;
      p0 = currentMarker.ptr<Point2f>(0)[j];
      p1 = currentMarker.ptr<Point2f>(0)[(j + 1) % 4];
      line(_image, p0, p1, borderColor, 1);
    }
    // draw first corner mark
    rectangle(
      _image, currentMarker.ptr<Point2f>(0)[0] - Point2f(3, 3),
      currentMarker.ptr<Point2f>(0)[0] + Point2f(3, 3), cornerColor, 1, LINE_AA);

    // draw ID
    if (_ids.total() != 0) {
      Point2f cent(0, 0);
      for (int p = 0; p < 4; p++) {
        cent += currentMarker.ptr<Point2f>(0)[p];
      }
      cent = cent / 4.;
      stringstream s;
      s << "id=" << _ids.getMat().ptr<int>(0)[i];
      putText(_image, s.str(), cent, FONT_HERSHEY_SIMPLEX, 0.5, textColor, 2);
    }
  }
}


/**
		*/
void drawAxis(
  InputOutputArray _image, InputArray _cameraMatrix, InputArray _distCoeffs,
  InputArray _rvec, InputArray _tvec, float length)
{

  CV_Assert(
    _image.getMat().total() != 0 &&
    (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
  CV_Assert(length > 0);

  // project axis points
  vector<Point3f> axisPoints;
  axisPoints.push_back(Point3f(0, 0, 0));
  axisPoints.push_back(Point3f(length, 0, 0));
  axisPoints.push_back(Point3f(0, length, 0));
  axisPoints.push_back(Point3f(0, 0, length));
  vector<Point2f> imagePoints;
  projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

  // draw axis lines
  line(_image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
  line(_image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
  line(_image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);
}


/**
		*/
void drawMarker(Dictionary dictionary, int id, int sidePixels, OutputArray _img, int borderBits)
{
  dictionary.drawMarker(id, sidePixels, _img, borderBits);
}


/**
		*/
void drawPlanarBoard(
  const Board & board, Size outSize, OutputArray _img, int marginSize,
  int borderBits)
{

  CV_Assert(outSize.area() > 0);
  CV_Assert(marginSize >= 0);

  _img.create(outSize, CV_8UC1);
  Mat out = _img.getMat();
  out.setTo(Scalar::all(255));
  Mat outNoMargins =
    out.colRange(marginSize, out.cols - marginSize).rowRange(marginSize, out.rows - marginSize);

  // calculate max and min values in XY plane
  CV_Assert(board.objPoints.size() > 0);
  float minX, maxX, minY, maxY;
  minX = maxX = board.objPoints[0][0].x;
  minY = maxY = board.objPoints[0][0].y;

  for (unsigned int i = 0; i < board.objPoints.size(); i++) {
    for (int j = 0; j < 4; j++) {
      minX = min(minX, board.objPoints[i][j].x);
      maxX = max(maxX, board.objPoints[i][j].x);
      minY = min(minY, board.objPoints[i][j].y);
      maxY = max(maxY, board.objPoints[i][j].y);
    }
  }

  float sizeX, sizeY;
  sizeX = maxX - minX;
  sizeY = maxY - minY;

  // proportion transformations
  float xReduction = sizeX / float(outNoMargins.cols);
  float yReduction = sizeY / float(outNoMargins.rows);

  // determine the zone where the markers are placed
  Mat markerZone;
  if (xReduction > yReduction) {
    int nRows = int(sizeY / xReduction);
    int rowsMargins = (outNoMargins.rows - nRows) / 2;
    markerZone = outNoMargins.rowRange(rowsMargins, outNoMargins.rows - rowsMargins);
  } else {
    int nCols = int(sizeX / yReduction);
    int colsMargins = (outNoMargins.cols - nCols) / 2;
    markerZone = outNoMargins.colRange(colsMargins, outNoMargins.cols - colsMargins);
  }

  // now paint each marker
  for (unsigned int m = 0; m < board.objPoints.size(); m++) {

    // transform corners to markerZone coordinates
    vector<Point2f> outCorners;
    outCorners.resize(4);
    for (int j = 0; j < 4; j++) {
      Point2f p0, p1, pf;
      p0 = Point2f(board.objPoints[m][j].x, board.objPoints[m][j].y);
      // remove negativity
      p1.x = p0.x - minX;
      p1.y = p0.y - minY;
      pf.x = p1.x * float(markerZone.cols - 1) / sizeX;
      pf.y = float(markerZone.rows - 1) - p1.y * float(markerZone.rows - 1) / sizeY;
      outCorners[j] = pf;
    }

    // get tiny marker
    int tinyMarkerSize = 10 * board.dictionary.markerSize + 2;
    Mat tinyMarker;
    board.dictionary.drawMarker(board.ids[m], tinyMarkerSize, tinyMarker, borderBits);

    // interpolate tiny marker to marker position in markerZone
    Mat inCorners(4, 1, CV_32FC2);
    inCorners.ptr<Point2f>(0)[0] = Point2f(0, 0);
    inCorners.ptr<Point2f>(0)[1] = Point2f((float)tinyMarker.cols, 0);
    inCorners.ptr<Point2f>(0)[2] = Point2f((float)tinyMarker.cols, (float)tinyMarker.rows);
    inCorners.ptr<Point2f>(0)[3] = Point2f(0, (float)tinyMarker.rows);

    // remove perspective
    Mat transformation = getPerspectiveTransform(inCorners, outCorners);
    Mat aux;
    const char borderValue = 127;
    warpPerspective(
      tinyMarker, aux, transformation, markerZone.size(), INTER_NEAREST,
      BORDER_CONSTANT, Scalar::all(borderValue));

    // copy only not-border pixels
    for (int y = 0; y < aux.rows; y++) {
      for (int x = 0; x < aux.cols; x++) {
        if (aux.at<unsigned char>(y, x) == borderValue) {continue;}
        markerZone.at<unsigned char>(y, x) = aux.at<unsigned char>(y, x);
      }
    }
  }
}


/**
		*/
double calibrateCameraAruco(
  InputArrayOfArrays _corners, InputArray _ids, InputArray _counter,
  const Board & board, Size imageSize, InputOutputArray _cameraMatrix,
  InputOutputArray _distCoeffs, OutputArrayOfArrays _rvecs,
  OutputArrayOfArrays _tvecs, int flags, TermCriteria criteria)
{

  // for each frame, get properly processed imagePoints and objectPoints for the calibrateCamera
  // function
  vector<Mat> processedObjectPoints, processedImagePoints;
  int nFrames = (int)_counter.total();
  int markerCounter = 0;
  for (int frame = 0; frame < nFrames; frame++) {
    int nMarkersInThisFrame = _counter.getMat().ptr<int>()[frame];
    vector<Mat> thisFrameCorners;
    vector<int> thisFrameIds;
    thisFrameCorners.reserve(nMarkersInThisFrame);
    thisFrameIds.reserve(nMarkersInThisFrame);
    for (int j = markerCounter; j < markerCounter + nMarkersInThisFrame; j++) {
      thisFrameCorners.push_back(_corners.getMat(j));
      thisFrameIds.push_back(_ids.getMat().ptr<int>()[j]);
    }
    markerCounter += nMarkersInThisFrame;
    Mat currentImgPoints, currentObjPoints;
    _getBoardObjectAndImagePoints(
      board, thisFrameIds, thisFrameCorners, currentImgPoints,
      currentObjPoints);
    if (currentImgPoints.total() > 0 && currentObjPoints.total() > 0) {
      processedImagePoints.push_back(currentImgPoints);
      processedObjectPoints.push_back(currentObjPoints);
    }
  }

  return calibrateCamera(
    processedObjectPoints, processedImagePoints, imageSize, _cameraMatrix,
    _distCoeffs, _rvecs, _tvecs, flags, criteria);
}

/**
		*/
void CharucoBoard::draw(Size outSize, OutputArray _img, int marginSize, int borderBits)
{

  CV_Assert(outSize.area() > 0);
  CV_Assert(marginSize >= 0);

  _img.create(outSize, CV_8UC1);
  _img.setTo(255);
  Mat out = _img.getMat();
  Mat noMarginsImg =
    out.colRange(marginSize, out.cols - marginSize).rowRange(marginSize, out.rows - marginSize);

  double totalLengthX, totalLengthY;
  totalLengthX = _squareLength * _squaresX;
  totalLengthY = _squareLength * _squaresY;

  // proportional transformation
  double xReduction = totalLengthX / double(noMarginsImg.cols);
  double yReduction = totalLengthY / double(noMarginsImg.rows);

  // determine the zone where the chessboard is placed
  Mat chessboardZoneImg;
  if (xReduction > yReduction) {
    int nRows = int(totalLengthY / xReduction);
    int rowsMargins = (noMarginsImg.rows - nRows) / 2;
    chessboardZoneImg = noMarginsImg.rowRange(rowsMargins, noMarginsImg.rows - rowsMargins);
  } else {
    int nCols = int(totalLengthX / yReduction);
    int colsMargins = (noMarginsImg.cols - nCols) / 2;
    chessboardZoneImg = noMarginsImg.colRange(colsMargins, noMarginsImg.cols - colsMargins);
  }

  // determine the margins to draw only the markers
  // take the minimum just to be sure
  double squareSizePixels = min(
    double(chessboardZoneImg.cols) / double(_squaresX),
    double(chessboardZoneImg.rows) / double(_squaresY));

  double diffSquareMarkerLength = (_squareLength - _markerLength) / 2;
  int diffSquareMarkerLengthPixels =
    int(diffSquareMarkerLength * squareSizePixels / _squareLength);

  // draw markers
  Mat markersImg;
  aruco::drawPlanarBoard(
    (*this), chessboardZoneImg.size(), markersImg,
    diffSquareMarkerLengthPixels, borderBits);

  markersImg.copyTo(chessboardZoneImg);

  // now draw black squares
  for (int y = 0; y < _squaresY; y++) {
    for (int x = 0; x < _squaresX; x++) {

      if (y % 2 != x % 2) {
        continue;                                                     // white corner, dont do anything

      }
      double startX, startY;
      startX = squareSizePixels * double(x);
      startY = double(chessboardZoneImg.rows) - squareSizePixels * double(y + 1);

      Mat squareZone = chessboardZoneImg.rowRange(int(startY), int(startY + squareSizePixels))
        .colRange(int(startX), int(startX + squareSizePixels));

      squareZone.setTo(0);
    }
  }
}


/**
		*/
CharucoBoard CharucoBoard::create(
  int squaresX, int squaresY, float squareLength,
  float markerLength, Dictionary dictionary)
{

  CV_Assert(squaresX > 1 && squaresY > 1 && markerLength > 0 && squareLength > markerLength);
  CharucoBoard res;

  res._squaresX = squaresX;
  res._squaresY = squaresY;
  res._squareLength = squareLength;
  res._markerLength = markerLength;
  res.dictionary = dictionary;

  float diffSquareMarkerLength = (squareLength - markerLength) / 2;

  // calculate Board objPoints
  for (int y = squaresY - 1; y >= 0; y--) {
    for (int x = 0; x < squaresX; x++) {

      if (y % 2 == x % 2) {
        continue;                                                     // black corner, no marker here

      }
      vector<Point3f> corners;
      corners.resize(4);
      corners[0] = Point3f(
        x * squareLength + diffSquareMarkerLength,
        y * squareLength + diffSquareMarkerLength + markerLength, 0);
      corners[1] = corners[0] + Point3f(markerLength, 0, 0);
      corners[2] = corners[0] + Point3f(markerLength, -markerLength, 0);
      corners[3] = corners[0] + Point3f(0, -markerLength, 0);
      res.objPoints.push_back(corners);
      // first ids in dictionary
      int nextId = (int)res.ids.size();
      res.ids.push_back(nextId);
    }
  }

  // now fill chessboardCorners
  for (int y = 0; y < squaresY - 1; y++) {
    for (int x = 0; x < squaresX - 1; x++) {
      Point3f corner;
      corner.x = (x + 1) * squareLength;
      corner.y = (y + 1) * squareLength;
      corner.z = 0;
      res.chessboardCorners.push_back(corner);
    }
  }

  res._getNearestMarkerCorners();

  return res;
}


/**
		* Fill nearestMarkerIdx and nearestMarkerCorners arrays
		*/
void CharucoBoard::_getNearestMarkerCorners()
{

  nearestMarkerIdx.resize(chessboardCorners.size());
  nearestMarkerCorners.resize(chessboardCorners.size());

  unsigned int nMarkers = (unsigned int)ids.size();
  unsigned int nCharucoCorners = (unsigned int)chessboardCorners.size();
  for (unsigned int i = 0; i < nCharucoCorners; i++) {
    double minDist = -1;                             // distance of closest markers
    Point3f charucoCorner = chessboardCorners[i];
    for (unsigned int j = 0; j < nMarkers; j++) {
      // calculate distance from marker center to charuco corner
      Point3f center = Point3f(0, 0, 0);
      for (unsigned int k = 0; k < 4; k++) {
        center += objPoints[j][k];
      }
      center /= 4.;
      double sqDistance;
      Point3f distVector = charucoCorner - center;
      sqDistance = distVector.x * distVector.x + distVector.y * distVector.y;
      if (j == 0 || fabs(sqDistance - minDist) < 0.0001) {
        // if same minimum distance (or first iteration), add to nearestMarkerIdx vector
        nearestMarkerIdx[i].push_back(j);
        minDist = sqDistance;
      } else if (sqDistance < minDist) {
        // if finding a closest marker to the charuco corner
        nearestMarkerIdx[i].clear();                                         // remove any previous added marker
        nearestMarkerIdx[i].push_back(j);                                         // add the new closest marker index
        minDist = sqDistance;
      }
    }

    // for each of the closest markers, search the marker corner index closer
    // to the charuco corner
    for (unsigned int j = 0; j < nearestMarkerIdx[i].size(); j++) {
      nearestMarkerCorners[i].resize(nearestMarkerIdx[i].size());
      double minDistCorner = -1;
      for (unsigned int k = 0; k < 4; k++) {
        double sqDistance;
        Point3f distVector = charucoCorner - objPoints[nearestMarkerIdx[i][j]][k];
        sqDistance = distVector.x * distVector.x + distVector.y * distVector.y;
        if (k == 0 || sqDistance < minDistCorner) {
          // if this corner is closer to the charuco corner, assing its index
          // to nearestMarkerCorners
          minDistCorner = sqDistance;
          nearestMarkerCorners[i][j] = k;
        }
      }
    }
  }
}


/**
		* Remove charuco corners if any of their minMarkers closest markers has not been detected
		*/
static unsigned int _filterCornersWithoutMinMarkers(
  const CharucoBoard & board,
  InputArray _allCharucoCorners,
  InputArray _allCharucoIds,
  InputArray _allArucoIds, int minMarkers,
  OutputArray _filteredCharucoCorners,
  OutputArray _filteredCharucoIds)
{

  CV_Assert(minMarkers >= 0 && minMarkers <= 2);

  vector<Point2f> filteredCharucoCorners;
  vector<int> filteredCharucoIds;
  // for each charuco corner
  for (unsigned int i = 0; i < _allCharucoIds.getMat().total(); i++) {
    int currentCharucoId = _allCharucoIds.getMat().ptr<int>(0)[i];
    int totalMarkers = 0;                             // nomber of closest marker detected
    // look for closest markers
    for (unsigned int m = 0; m < board.nearestMarkerIdx[currentCharucoId].size(); m++) {
      int markerId = board.ids[board.nearestMarkerIdx[currentCharucoId][m]];
      bool found = false;
      for (unsigned int k = 0; k < _allArucoIds.getMat().total(); k++) {
        if (_allArucoIds.getMat().ptr<int>(0)[k] == markerId) {
          found = true;
          break;
        }
      }
      if (found) {totalMarkers++;}
    }
    // if enough markers detected, add the charuco corner to the final list
    if (totalMarkers >= minMarkers) {
      filteredCharucoIds.push_back(currentCharucoId);
      filteredCharucoCorners.push_back(_allCharucoCorners.getMat().ptr<Point2f>(0)[i]);
    }
  }

  // parse output
  _filteredCharucoCorners.create((int)filteredCharucoCorners.size(), 1, CV_32FC2);
  for (unsigned int i = 0; i < filteredCharucoCorners.size(); i++) {
    _filteredCharucoCorners.getMat().ptr<Point2f>(0)[i] = filteredCharucoCorners[i];
  }

  _filteredCharucoIds.create((int)filteredCharucoIds.size(), 1, CV_32SC1);
  for (unsigned int i = 0; i < filteredCharucoIds.size(); i++) {
    _filteredCharucoIds.getMat().ptr<int>(0)[i] = filteredCharucoIds[i];
  }

  return (unsigned int)filteredCharucoCorners.size();
}


/**
		* ParallelLoopBody class for the parallelization of the charuco corners subpixel refinement
		* Called from function _selectAndRefineChessboardCorners()
		*/
class CharucoSubpixelParallel : public ParallelLoopBody
{
public:
  CharucoSubpixelParallel(
    const Mat * _grey, vector<Point2f> * _filteredChessboardImgPoints,
    vector<Size> * _filteredWinSizes, DetectorParameters * _params)
  : grey(_grey), filteredChessboardImgPoints(_filteredChessboardImgPoints),
    filteredWinSizes(_filteredWinSizes), params(_params) {}

  void operator()(const Range & range) const
  {
    const int begin = range.start;
    const int end = range.end;

    for (int i = begin; i < end; i++) {
      vector<Point2f> in;
      in.push_back((*filteredChessboardImgPoints)[i]);
      Size winSize = (*filteredWinSizes)[i];
      if (winSize.height == -1 || winSize.width == -1) {
        winSize = Size(params->cornerRefinementWinSize, params->cornerRefinementWinSize);
      }

      cornerSubPix(
        *grey, in, winSize, Size(),
        TermCriteria(
          TermCriteria::MAX_ITER | TermCriteria::EPS,
          params->cornerRefinementMaxIterations,
          params->cornerRefinementMinAccuracy));

      (*filteredChessboardImgPoints)[i] = in[0];
    }
  }

private:
  CharucoSubpixelParallel & operator=(const CharucoSubpixelParallel &);                      // to quiet MSVC

  const Mat * grey;
  vector<Point2f> * filteredChessboardImgPoints;
  vector<Size> * filteredWinSizes;
  DetectorParameters * params;
};


/**
		* @brief From all projected chessboard corners, select those inside the image and apply subpixel
		* refinement. Returns number of valid corners.
		*/
static unsigned int _selectAndRefineChessboardCorners(
  InputArray _allCorners, InputArray _image,
  OutputArray _selectedCorners,
  OutputArray _selectedIds,
  const vector<Size> & winSizes)
{

  const int minDistToBorder = 2;                       // minimum distance of the corner to the image border
  // remaining corners, ids and window refinement sizes after removing corners outside the image
  vector<Point2f> filteredChessboardImgPoints;
  vector<Size> filteredWinSizes;
  vector<int> filteredIds;

  // filter corners outside the image
  Rect innerRect(minDistToBorder, minDistToBorder, _image.getMat().cols - 2 * minDistToBorder,
    _image.getMat().rows - 2 * minDistToBorder);
  for (unsigned int i = 0; i < _allCorners.getMat().total(); i++) {
    if (innerRect.contains(_allCorners.getMat().ptr<Point2f>(0)[i])) {
      filteredChessboardImgPoints.push_back(_allCorners.getMat().ptr<Point2f>(0)[i]);
      filteredIds.push_back(i);
      filteredWinSizes.push_back(winSizes[i]);
    }
  }

  // if none valid, return 0
  if (filteredChessboardImgPoints.size() == 0) {return 0;}

  // corner refinement, first convert input image to grey
  Mat grey;
  if (_image.getMat().type() == CV_8UC3) {
    cvtColor(_image.getMat(), grey, COLOR_BGR2GRAY);
  } else {
    _image.getMat().copyTo(grey);
  }

  DetectorParameters params;                       // use default params for corner refinement

  //// For each of the charuco corners, apply subpixel refinement using its correspondind winSize
  // for(unsigned int i=0; i<filteredChessboardImgPoints.size(); i++) {
  //    vector<Point2f> in;
  //    in.push_back(filteredChessboardImgPoints[i]);
  //    Size winSize = filteredWinSizes[i];
  //    if(winSize.height == -1 || winSize.width == -1)
  //        winSize = Size(params.cornerRefinementWinSize, params.cornerRefinementWinSize);
  //    cornerSubPix(grey, in, winSize, Size(),
  //                 TermCriteria(TermCriteria::MAX_ITER | TermCriteria::EPS,
  //                              params->cornerRefinementMaxIterations,
  //                              params->cornerRefinementMinAccuracy));
  //    filteredChessboardImgPoints[i] = in[0];
  //}

  // this is the parallel call for the previous commented loop (result is equivalent)
  parallel_for_(
    Range(0, (int)filteredChessboardImgPoints.size()),
    CharucoSubpixelParallel(&grey, &filteredChessboardImgPoints, &filteredWinSizes, &params));

  // parse output
  _selectedCorners.create((int)filteredChessboardImgPoints.size(), 1, CV_32FC2);
  for (unsigned int i = 0; i < filteredChessboardImgPoints.size(); i++) {
    _selectedCorners.getMat().ptr<Point2f>(0)[i] = filteredChessboardImgPoints[i];
  }

  _selectedIds.create((int)filteredIds.size(), 1, CV_32SC1);
  for (unsigned int i = 0; i < filteredIds.size(); i++) {
    _selectedIds.getMat().ptr<int>(0)[i] = filteredIds[i];
  }

  return (unsigned int)filteredChessboardImgPoints.size();
}


/**
		* Calculate the maximum window sizes for corner refinement for each charuco corner based on the
		* distance to their closest markers
		*/
static void _getMaximumSubPixWindowSizes(
  InputArrayOfArrays markerCorners, InputArray markerIds,
  InputArray charucoCorners, const CharucoBoard & board,
  vector<Size> & sizes)
{

  unsigned int nCharucoCorners = (unsigned int)charucoCorners.getMat().total();
  sizes.resize(nCharucoCorners, Size(-1, -1));

  for (unsigned int i = 0; i < nCharucoCorners; i++) {
    if (charucoCorners.getMat().ptr<Point2f>(0)[i] == Point2f(-1, -1)) {continue;}
    if (board.nearestMarkerIdx[i].size() == 0) {continue;}

    double minDist = -1;
    int counter = 0;

    // calculate the distance to each of the closest corner of each closest marker
    for (unsigned int j = 0; j < board.nearestMarkerIdx[i].size(); j++) {
      // find marker
      int markerId = board.ids[board.nearestMarkerIdx[i][j]];
      int markerIdx = -1;
      for (unsigned int k = 0; k < markerIds.getMat().total(); k++) {
        if (markerIds.getMat().ptr<int>(0)[k] == markerId) {
          markerIdx = k;
          break;
        }
      }
      if (markerIdx == -1) {continue;}
      Point2f markerCorner =
        markerCorners.getMat(markerIdx).ptr<Point2f>(0)[board.nearestMarkerCorners[i][j]];
      Point2f charucoCorner = charucoCorners.getMat().ptr<Point2f>(0)[i];
      double dist = norm(markerCorner - charucoCorner);
      if (minDist == -1) {
        minDist = dist;                                                    // if first distance, just assign it
      }
      minDist = min(dist, minDist);
      counter++;
    }

    // if this is the first closest marker, dont do anything
    if (counter == 0) {
      continue;
    } else {
      // else, calculate the maximum window size
      int winSizeInt = int(minDist - 2);                                   // remove 2 pixels for safety
      if (winSizeInt < 1) {
        winSizeInt = 1;                                                     // minimum size is 1
      }
      if (winSizeInt > 10) {
        winSizeInt = 10;                                                      // maximum size is 10
      }
      sizes[i] = Size(winSizeInt, winSizeInt);
    }
  }
}


/**
		* Interpolate charuco corners using approximated pose estimation
		*/
static int _interpolateCornersCharucoApproxCalib(
  InputArrayOfArrays _markerCorners,
  InputArray _markerIds, InputArray _image,
  const CharucoBoard & board,
  InputArray _cameraMatrix, InputArray _distCoeffs,
  OutputArray _charucoCorners,
  OutputArray _charucoIds)
{

  CV_Assert(_image.getMat().channels() == 1 || _image.getMat().channels() == 3);
  CV_Assert(
    _markerCorners.total() == _markerIds.getMat().total() &&
    _markerIds.getMat().total() > 0);

  // approximated pose estimation using marker corners
  Mat approximatedRvec, approximatedTvec;
  int detectedBoardMarkers;
  detectedBoardMarkers =
    aruco::estimatePoseBoard(
    _markerCorners, _markerIds, board, _cameraMatrix, _distCoeffs,
    approximatedRvec, approximatedTvec);

  if (detectedBoardMarkers == 0) {return 0;}

  // project chessboard corners
  vector<Point2f> allChessboardImgPoints;
  projectPoints(
    board.chessboardCorners, approximatedRvec, approximatedTvec, _cameraMatrix,
    _distCoeffs, allChessboardImgPoints);


  // calculate maximum window sizes for subpixel refinement. The size is limited by the distance
  // to the closes marker corner to avoid erroneous displacements to marker corners
  vector<Size> subPixWinSizes;
  _getMaximumSubPixWindowSizes(
    _markerCorners, _markerIds, allChessboardImgPoints, board,
    subPixWinSizes);

  // filter corners outside the image and subpixel-refine charuco corners
  unsigned int nRefinedCorners;
  nRefinedCorners = _selectAndRefineChessboardCorners(
    allChessboardImgPoints, _image, _charucoCorners, _charucoIds, subPixWinSizes);

  // to return a charuco corner, its two closes aruco markers should have been detected
  nRefinedCorners = _filterCornersWithoutMinMarkers(
    board, _charucoCorners, _charucoIds,
    _markerIds, 2, _charucoCorners, _charucoIds);

  return nRefinedCorners;
}


/**
		* Interpolate charuco corners using local homography
		*/
static int _interpolateCornersCharucoLocalHom(
  InputArrayOfArrays _markerCorners,
  InputArray _markerIds, InputArray _image,
  const CharucoBoard & board,
  OutputArray _charucoCorners,
  OutputArray _charucoIds)
{

  CV_Assert(_image.getMat().channels() == 1 || _image.getMat().channels() == 3);
  CV_Assert(
    _markerCorners.total() == _markerIds.getMat().total() &&
    _markerIds.getMat().total() > 0);

  unsigned int nMarkers = (unsigned int)_markerIds.getMat().total();

  // calculate local homographies for each marker
  vector<Mat> transformations;
  transformations.resize(nMarkers);
  for (unsigned int i = 0; i < nMarkers; i++) {
    vector<Point2f> markerObjPoints2D;
    int markerId = _markerIds.getMat().ptr<int>(0)[i];
    vector<int>::const_iterator it = find(board.ids.begin(), board.ids.end(), markerId);
    if (it == board.ids.end()) {continue;}
    int boardIdx = (int)std::distance(board.ids.begin(), it);
    markerObjPoints2D.resize(4);
    for (unsigned int j = 0; j < 4; j++) {
      markerObjPoints2D[j] =
        Point2f(board.objPoints[boardIdx][j].x, board.objPoints[boardIdx][j].y);
    }

    transformations[i] = getPerspectiveTransform(markerObjPoints2D, _markerCorners.getMat(i));
  }

  unsigned int nCharucoCorners = (unsigned int)board.chessboardCorners.size();
  vector<Point2f> allChessboardImgPoints(nCharucoCorners, Point2f(-1, -1));

  // for each charuco corner, calculate its interpolation position based on the closest markers
  // homographies
  for (unsigned int i = 0; i < nCharucoCorners; i++) {
    Point2f objPoint2D = Point2f(board.chessboardCorners[i].x, board.chessboardCorners[i].y);

    vector<Point2f> interpolatedPositions;
    for (unsigned int j = 0; j < board.nearestMarkerIdx[i].size(); j++) {
      int markerId = board.ids[board.nearestMarkerIdx[i][j]];
      int markerIdx = -1;
      for (unsigned int k = 0; k < _markerIds.getMat().total(); k++) {
        if (_markerIds.getMat().ptr<int>(0)[k] == markerId) {
          markerIdx = k;
          break;
        }
      }
      if (markerIdx != -1) {
        vector<Point2f> in, out;
        in.push_back(objPoint2D);
        perspectiveTransform(in, out, transformations[markerIdx]);
        interpolatedPositions.push_back(out[0]);
      }
    }

    // none of the closest markers detected
    if (interpolatedPositions.size() == 0) {continue;}

    // more than one closest marker detected, take middle point
    if (interpolatedPositions.size() > 1) {
      allChessboardImgPoints[i] = (interpolatedPositions[0] + interpolatedPositions[1]) / 2.;
    }
    // a single closest marker detected
    else {allChessboardImgPoints[i] = interpolatedPositions[0];}
  }

  // calculate maximum window sizes for subpixel refinement. The size is limited by the distance
  // to the closes marker corner to avoid erroneous displacements to marker corners
  vector<Size> subPixWinSizes;
  _getMaximumSubPixWindowSizes(
    _markerCorners, _markerIds, allChessboardImgPoints, board,
    subPixWinSizes);


  // filter corners outside the image and subpixel-refine charuco corners
  unsigned int nRefinedCorners;
  nRefinedCorners = _selectAndRefineChessboardCorners(
    allChessboardImgPoints, _image, _charucoCorners, _charucoIds, subPixWinSizes);

  // to return a charuco corner, its two closes aruco markers should have been detected
  nRefinedCorners = _filterCornersWithoutMinMarkers(
    board, _charucoCorners, _charucoIds,
    _markerIds, 2, _charucoCorners, _charucoIds);

  return nRefinedCorners;
}


/**
		*/
int interpolateCornersCharuco(
  InputArrayOfArrays _markerCorners, InputArray _markerIds,
  InputArray _image, const CharucoBoard & board,
  OutputArray _charucoCorners, OutputArray _charucoIds,
  InputArray _cameraMatrix, InputArray _distCoeffs)
{

  // if camera parameters are avaible, use approximated calibration
  if (_cameraMatrix.total() != 0) {
    return _interpolateCornersCharucoApproxCalib(
      _markerCorners, _markerIds, _image, board,
      _cameraMatrix, _distCoeffs, _charucoCorners,
      _charucoIds);
  }
  // else use local homography
  else {
    return _interpolateCornersCharucoLocalHom(
      _markerCorners, _markerIds, _image, board,
      _charucoCorners, _charucoIds);
  }
}


/**
		*/
void drawDetectedCornersCharuco(
  InputOutputArray _image, InputArray _charucoCorners,
  InputArray _charucoIds, Scalar cornerColor)
{

  CV_Assert(
    _image.getMat().total() != 0 &&
    (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
  CV_Assert(
    (_charucoCorners.getMat().total() == _charucoIds.getMat().total()) ||
    _charucoIds.getMat().total() == 0);

  unsigned int nCorners = (unsigned int)_charucoCorners.getMat().total();
  for (unsigned int i = 0; i < nCorners; i++) {
    Point2f corner = _charucoCorners.getMat().ptr<Point2f>(0)[i];

    // draw first corner mark
    rectangle(_image, corner - Point2f(3, 3), corner + Point2f(3, 3), cornerColor, 1, LINE_AA);

    // draw ID
    if (_charucoIds.total() != 0) {
      int id = _charucoIds.getMat().ptr<int>(0)[i];
      stringstream s;
      s << "id=" << id;
      putText(
        _image, s.str(), corner + Point2f(5, -5), FONT_HERSHEY_SIMPLEX, 0.5,
        cornerColor, 2);
    }
  }
}


/**
		* Check if a set of 3d points are enough for calibration. Z coordinate is ignored.
		* Only axis paralel lines are considered
		*/
static bool _arePointsEnoughForPoseEstimation(const vector<Point3f> & points)
{

  if (points.size() < 4) {return false;}

  vector<double> sameXValue;                         // different x values in points
  vector<int> sameXCounter;                          // number of points with the x value in sameXValue
  for (unsigned int i = 0; i < points.size(); i++) {
    bool found = false;
    for (unsigned int j = 0; j < sameXValue.size(); j++) {
      if (sameXValue[j] == points[i].x) {
        found = true;
        sameXCounter[j]++;
      }
    }
    if (!found) {
      sameXValue.push_back(points[i].x);
      sameXCounter.push_back(1);
    }
  }

  // count how many x values has more than 2 points
  int moreThan2 = 0;
  for (unsigned int i = 0; i < sameXCounter.size(); i++) {
    if (sameXCounter[i] >= 2) {moreThan2++;}
  }

  // if we have more than 1 two xvalues with more than 2 points, calibration is ok
  if (moreThan2 > 1) {
    return true;
  } else {
    return false;
  }
}


/**
		*/
bool estimatePoseCharucoBoard(
  InputArray _charucoCorners, InputArray _charucoIds,
  CharucoBoard & board, InputArray _cameraMatrix, InputArray _distCoeffs,
  OutputArray _rvec, OutputArray _tvec)
{

  CV_Assert((_charucoCorners.getMat().total() == _charucoIds.getMat().total()));

  // need, at least, 4 corners
  if (_charucoIds.getMat().total() < 4) {return false;}

  vector<Point3f> objPoints;
  objPoints.reserve(_charucoIds.getMat().total());
  for (unsigned int i = 0; i < _charucoIds.getMat().total(); i++) {
    int currId = _charucoIds.getMat().ptr<int>(0)[i];
    CV_Assert(currId >= 0 && currId < (int)board.chessboardCorners.size());
    objPoints.push_back(board.chessboardCorners[currId]);
  }

  // points need to be in different lines, check if detected points are enough
  if (!_arePointsEnoughForPoseEstimation(objPoints)) {return false;}

  solvePnP(objPoints, _charucoCorners, _cameraMatrix, _distCoeffs, _rvec, _tvec);

  return true;
}


/**
		*/
double calibrateCameraCharuco(
  InputArrayOfArrays _charucoCorners, InputArrayOfArrays _charucoIds,
  const CharucoBoard & board, Size imageSize,
  InputOutputArray _cameraMatrix, InputOutputArray _distCoeffs,
  OutputArrayOfArrays _rvecs, OutputArrayOfArrays _tvecs, int flags,
  TermCriteria criteria)
{


  CV_Assert(_charucoIds.total() > 0 && (_charucoIds.total() == _charucoCorners.total()));

  // Join object points of charuco corners in a single vector for calibrateCamera() function
  vector<vector<Point3f>> allObjPoints;
  allObjPoints.resize(_charucoIds.total());
  for (unsigned int i = 0; i < _charucoIds.total(); i++) {
    unsigned int nCorners = (unsigned int)_charucoIds.getMat(i).total();
    CV_Assert(nCorners > 0 && nCorners == _charucoCorners.getMat(i).total());
    allObjPoints[i].reserve(nCorners);

    for (unsigned int j = 0; j < nCorners; j++) {
      int pointId = _charucoIds.getMat(i).ptr<int>(0)[j];
      CV_Assert(pointId >= 0 && pointId < (int)board.chessboardCorners.size());
      allObjPoints[i].push_back(board.chessboardCorners[pointId]);
    }
  }

  return calibrateCamera(
    allObjPoints, _charucoCorners, imageSize, _cameraMatrix, _distCoeffs,
    _rvecs, _tvecs, flags, criteria);
}


/**
		*/
void detectCharucoDiamond(
  InputArray _image, InputArrayOfArrays _markerCorners,
  InputArray _markerIds, float squareMarkerLengthRate,
  OutputArrayOfArrays _diamondCorners, OutputArray _diamondIds,
  InputArray _cameraMatrix, InputArray _distCoeffs)
{

  CV_Assert(_markerIds.total() > 0 && _markerIds.total() == _markerCorners.total());

  const float minRepDistanceRate = 0.12f;

  // create Charuco board layout for diamond (3x3 layout)
  CharucoBoard charucoDiamondLayout;
  Dictionary dict = getPredefinedDictionary(PREDEFINED_DICTIONARY_NAME(0));
  charucoDiamondLayout = CharucoBoard::create(3, 3, squareMarkerLengthRate, 1., dict);

  vector<vector<Point2f>> diamondCorners;
  vector<Vec4i> diamondIds;

  // stores if the detected markers have been assigned or not to a diamond
  vector<bool> assigned(_markerIds.total(), false);
  if (_markerIds.total() < 4) {
    return;                                                 // a diamond need at least 4 markers

  }
  // convert input image to grey
  Mat grey;
  if (_image.getMat().type() == CV_8UC3) {
    cvtColor(_image.getMat(), grey, COLOR_BGR2GRAY);
  } else {
    _image.getMat().copyTo(grey);
  }

  // for each of the detected markers, try to find a diamond
  for (unsigned int i = 0; i < _markerIds.total(); i++) {
    if (assigned[i]) {continue;}

    // calculate marker perimeter
    float perimeterSq = 0;
    Mat corners = _markerCorners.getMat(i);
    for (int c = 0; c < 4; c++) {
      perimeterSq +=
        (corners.ptr<Point2f>()[c].x - corners.ptr<Point2f>()[(c + 1) % 4].x) *
        (corners.ptr<Point2f>()[c].x - corners.ptr<Point2f>()[(c + 1) % 4].x) +
        (corners.ptr<Point2f>()[c].y - corners.ptr<Point2f>()[(c + 1) % 4].y) *
        (corners.ptr<Point2f>()[c].y - corners.ptr<Point2f>()[(c + 1) % 4].y);
    }
    // maximum reprojection error relative to perimeter
    float minRepDistance = perimeterSq * minRepDistanceRate * minRepDistanceRate;

    int currentId = _markerIds.getMat().ptr<int>()[i];

    // prepare data to call refineDetectedMarkers()
    // detected markers (only the current one)
    vector<Mat> currentMarker;
    vector<int> currentMarkerId;
    currentMarker.push_back(_markerCorners.getMat(i));
    currentMarkerId.push_back(currentId);

    // marker candidates (the rest of markers if they have not been assigned)
    vector<Mat> candidates;
    vector<int> candidatesIdxs;
    for (unsigned int k = 0; k < assigned.size(); k++) {
      if (k == i) {continue;}
      if (!assigned[k]) {
        candidates.push_back(_markerCorners.getMat(k));
        candidatesIdxs.push_back(k);
      }
    }
    if (candidates.size() < 3) {
      break;                                                      // we need at least 3 free markers

    }
    // modify charuco layout id to make sure all the ids are different than current id
    for (int k = 1; k < 4; k++) {
      charucoDiamondLayout.ids[k] = currentId + 1 + k;
    }
    // current id is assigned to [0], so it is the marker on the top
    charucoDiamondLayout.ids[0] = currentId;

    // try to find the rest of markers in the diamond
    vector<int> acceptedIdxs;
    aruco::refineDetectedMarkers(
      grey, charucoDiamondLayout, currentMarker, currentMarkerId,
      candidates, noArray(), noArray(), minRepDistance, -1, false,
      acceptedIdxs);

    // if found, we have a diamond
    if (currentMarker.size() == 4) {

      assigned[i] = true;

      // calculate diamond id, acceptedIdxs array indicates the markers taken from candidates
      // array
      Vec4i markerId;
      markerId[0] = currentId;
      for (int k = 1; k < 4; k++) {
        int currentMarkerIdx = candidatesIdxs[acceptedIdxs[k - 1]];
        markerId[k] = _markerIds.getMat().ptr<int>()[currentMarkerIdx];
        assigned[currentMarkerIdx] = true;
      }

      // interpolate the charuco corners of the diamond
      vector<Point2f> currentMarkerCorners;
      Mat aux;
      interpolateCornersCharuco(
        currentMarker, currentMarkerId, grey, charucoDiamondLayout,
        currentMarkerCorners, aux, _cameraMatrix, _distCoeffs);

      // if everything is ok, save the diamond
      if (currentMarkerCorners.size() > 0) {
        // reorder corners
        vector<Point2f> currentMarkerCornersReorder;
        currentMarkerCornersReorder.resize(4);
        currentMarkerCornersReorder[0] = currentMarkerCorners[2];
        currentMarkerCornersReorder[1] = currentMarkerCorners[3];
        currentMarkerCornersReorder[2] = currentMarkerCorners[1];
        currentMarkerCornersReorder[3] = currentMarkerCorners[0];

        diamondCorners.push_back(currentMarkerCornersReorder);
        diamondIds.push_back(markerId);
      }
    }
  }


  if (diamondIds.size() > 0) {

    // parse output
    _diamondIds.create((int)diamondIds.size(), 1, CV_32SC4);
    for (unsigned int i = 0; i < diamondIds.size(); i++) {
      _diamondIds.getMat().ptr<Vec4i>(0)[i] = diamondIds[i];
    }

    _diamondCorners.create((int)diamondCorners.size(), 1, CV_32FC2);
    for (unsigned int i = 0; i < diamondCorners.size(); i++) {
      _diamondCorners.create(4, 1, CV_32FC2, i, true);
      for (int j = 0; j < 4; j++) {
        _diamondCorners.getMat(i).ptr<Point2f>()[j] = diamondCorners[i][j];
      }
    }
  }
}


/**
		*/
void drawCharucoDiamond(
  Dictionary dictionary, Vec4i ids, int squareLength, int markerLength,
  OutputArray _img, int marginSize, int borderBits)
{

  CV_Assert(squareLength > 0 && markerLength > 0 && squareLength > markerLength);
  CV_Assert(marginSize >= 0 && borderBits > 0);

  // create a charuco board similar to a charuco marker and print it
  CharucoBoard board =
    CharucoBoard::create(3, 3, (float)squareLength, (float)markerLength, dictionary);

  // assign the charuco marker ids
  for (int i = 0; i < 4; i++) {
    board.ids[i] = ids[i];
  }

  Size outSize(3 * squareLength + 2 * marginSize, 3 * squareLength + 2 * marginSize);
  board.draw(outSize, _img, marginSize, borderBits);
}


/**
		*/
void drawDetectedDiamonds(
  InputOutputArray _image, InputArrayOfArrays _corners,
  InputArray _ids, Scalar borderColor)
{


  CV_Assert(
    _image.getMat().total() != 0 &&
    (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
  CV_Assert((_corners.total() == _ids.total()) || _ids.total() == 0);

  // calculate colors
  Scalar textColor, cornerColor;
  textColor = cornerColor = borderColor;
  swap(textColor.val[0], textColor.val[1]);                           // text color just sawp G and R
  swap(cornerColor.val[1], cornerColor.val[2]);                       // corner color just sawp G and B

  int nMarkers = (int)_corners.total();
  for (int i = 0; i < nMarkers; i++) {
    Mat currentMarker = _corners.getMat(i);
    CV_Assert(currentMarker.total() == 4 && currentMarker.type() == CV_32FC2);

    // draw marker sides
    for (int j = 0; j < 4; j++) {
      Point2f p0, p1;
      p0 = currentMarker.ptr<Point2f>(0)[j];
      p1 = currentMarker.ptr<Point2f>(0)[(j + 1) % 4];
      line(_image, p0, p1, borderColor, 1);
    }

    // draw first corner mark
    rectangle(
      _image, currentMarker.ptr<Point2f>(0)[0] - Point2f(3, 3),
      currentMarker.ptr<Point2f>(0)[0] + Point2f(3, 3), cornerColor, 1, LINE_AA);

    // draw id composed by four numbers
    if (_ids.total() != 0) {
      Point2f cent(0, 0);
      for (int p = 0; p < 4; p++) {
        cent += currentMarker.ptr<Point2f>(0)[p];
      }
      cent = cent / 4.;
      stringstream s;
      s << "id=" << _ids.getMat().ptr<Vec4i>(0)[i];
      putText(_image, s.str(), cent, FONT_HERSHEY_SIMPLEX, 0.5, textColor, 2);
    }
  }
}


/**
		*/
Dictionary::Dictionary(const Mat & _bytesList, int _markerSize, int _maxcorr)
{
  markerSize = _markerSize;
  maxCorrectionBits = _maxcorr;
  bytesList = _bytesList;
}


/**
		*/
bool Dictionary::identify(
  const Mat & onlyBits, int & idx, int & rotation,
  double maxCorrectionRate) const
{

  CV_Assert(onlyBits.rows == markerSize && onlyBits.cols == markerSize);

  int maxCorrectionRecalculed = int(double(maxCorrectionBits) * maxCorrectionRate);

  // get as a byte list
  Mat candidateBytes = getByteListFromBits(onlyBits);

  idx = -1;                       // by default, not found

  // search closest marker in dict
  for (int m = 0; m < bytesList.rows; m++) {
    int currentMinDistance = markerSize * markerSize + 1;
    int currentRotation = -1;
    for (unsigned int r = 0; r < 4; r++) {
      int currentHamming = cv::hal::normHamming(
        bytesList.ptr(m) + r * candidateBytes.cols,
        candidateBytes.ptr(),
        candidateBytes.cols);

      if (currentHamming < currentMinDistance) {
        currentMinDistance = currentHamming;
        currentRotation = r;
      }
    }

    // if maxCorrection is fullfilled, return this one
    if (currentMinDistance <= maxCorrectionRecalculed) {
      idx = m;
      rotation = currentRotation;
      break;
    }
  }

  return idx != -1;
}


/**
		*/
int Dictionary::getDistanceToId(InputArray bits, int id, bool allRotations) const
{

  CV_Assert(id >= 0 && id < bytesList.rows);

  unsigned int nRotations = 4;
  if (!allRotations) {nRotations = 1;}

  Mat candidateBytes = getByteListFromBits(bits.getMat());
  int currentMinDistance = int(bits.total() * bits.total());
  for (unsigned int r = 0; r < nRotations; r++) {
    int currentHamming = cv::hal::normHamming(
      bytesList.ptr(id) + r * candidateBytes.cols,
      candidateBytes.ptr(),
      candidateBytes.cols);

    if (currentHamming < currentMinDistance) {
      currentMinDistance = currentHamming;
    }
  }
  return currentMinDistance;
}


/**
		* @brief Draw a canonical marker image
		*/
void Dictionary::drawMarker(int id, int sidePixels, OutputArray _img, int borderBits) const
{

  CV_Assert(sidePixels > markerSize);
  CV_Assert(id < bytesList.rows);
  CV_Assert(borderBits > 0);

  _img.create(sidePixels, sidePixels, CV_8UC1);

  // create small marker with 1 pixel per bin
  Mat tinyMarker(markerSize + 2 * borderBits, markerSize + 2 * borderBits, CV_8UC1,
    Scalar::all(0));
  Mat innerRegion = tinyMarker.rowRange(borderBits, tinyMarker.rows - borderBits)
    .colRange(borderBits, tinyMarker.cols - borderBits);
  // put inner bits
  Mat bits = 255 * getBitsFromByteList(bytesList.rowRange(id, id + 1), markerSize);
  CV_Assert(innerRegion.total() == bits.total());
  bits.copyTo(innerRegion);

  // resize tiny marker to output size
  cv::resize(tinyMarker, _img.getMat(), _img.getMat().size(), 0, 0, INTER_NEAREST);
}


/**
		* @brief Transform matrix of bits to list of bytes in the 4 rotations
		*/
Mat Dictionary::getByteListFromBits(const Mat & bits)
{
  // integer ceil
  int nbytes = (bits.cols * bits.rows + 8 - 1) / 8;

  Mat candidateByteList(1, nbytes, CV_8UC4, Scalar::all(0));
  unsigned char currentBit = 0;
  int currentByte = 0;

  // the 4 rotations
  uchar * rot0 = candidateByteList.ptr();
  uchar * rot1 = candidateByteList.ptr() + 1 * nbytes;
  uchar * rot2 = candidateByteList.ptr() + 2 * nbytes;
  uchar * rot3 = candidateByteList.ptr() + 3 * nbytes;

  for (int row = 0; row < bits.rows; row++) {
    for (int col = 0; col < bits.cols; col++) {
      // circular shift
      rot0[currentByte] <<= 1;
      rot1[currentByte] <<= 1;
      rot2[currentByte] <<= 1;
      rot3[currentByte] <<= 1;
      // set bit
      rot0[currentByte] |= bits.at<uchar>(row, col);
      rot1[currentByte] |= bits.at<uchar>(col, bits.cols - 1 - row);
      rot2[currentByte] |= bits.at<uchar>(bits.rows - 1 - row, bits.cols - 1 - col);
      rot3[currentByte] |= bits.at<uchar>(bits.rows - 1 - col, row);
      currentBit++;
      if (currentBit == 8) {
        // next byte
        currentBit = 0;
        currentByte++;
      }
    }
  }
  return candidateByteList;
}


/**
		* @brief Transform list of bytes to matrix of bits
		*/
Mat Dictionary::getBitsFromByteList(const Mat & byteList, int markerSize)
{
  CV_Assert(
    byteList.total() > 0 &&
    byteList.total() >= (unsigned int)markerSize * markerSize / 8 &&
    byteList.total() <= (unsigned int)markerSize * markerSize / 8 + 1);
  Mat bits(markerSize, markerSize, CV_8UC1, Scalar::all(0));

  unsigned char base2List[] = {128, 64, 32, 16, 8, 4, 2, 1};
  int currentByteIdx = 0;
  // we only need the bytes in normal rotation
  unsigned char currentByte = byteList.ptr()[0];
  int currentBit = 0;
  for (int row = 0; row < bits.rows; row++) {
    for (int col = 0; col < bits.cols; col++) {
      if (currentByte >= base2List[currentBit]) {
        bits.at<unsigned char>(row, col) = 1;
        currentByte -= base2List[currentBit];
      }
      currentBit++;
      if (currentBit == 8) {
        currentByteIdx++;
        currentByte = byteList.ptr()[currentByteIdx];
        // if not enough bits for one more byte, we are in the end
        // update bit position accordingly
        if (8 * (currentByteIdx + 1) > (int)bits.total()) {
          currentBit = 8 * (currentByteIdx + 1) - (int)bits.total();
        } else {
          currentBit = 0;                                               // ok, bits enough for next byte
        }
      }
    }
  }
  return bits;
}

// DictionaryData constructors calls
const Dictionary DICT_ARUCO_DATA =
  Dictionary(Mat(1024, (5 * 5 + 7) / 8, CV_8UC4, (uchar *)DICT_ARUCO_BYTES), 5, 0);

const Dictionary DICT_4X4_50_DATA =
  Dictionary(Mat(50, (4 * 4 + 7) / 8, CV_8UC4, (uchar *)DICT_4X4_1000_BYTES), 4, 1);
const Dictionary DICT_4X4_100_DATA =
  Dictionary(Mat(100, (4 * 4 + 7) / 8, CV_8UC4, (uchar *)DICT_4X4_1000_BYTES), 4, 1);
const Dictionary DICT_4X4_250_DATA =
  Dictionary(Mat(250, (4 * 4 + 7) / 8, CV_8UC4, (uchar *)DICT_4X4_1000_BYTES), 4, 1);
const Dictionary DICT_4X4_1000_DATA =
  Dictionary(Mat(1000, (4 * 4 + 7) / 8, CV_8UC4, (uchar *)DICT_4X4_1000_BYTES), 4, 0);

const Dictionary DICT_5X5_50_DATA =
  Dictionary(Mat(50, (5 * 5 + 7) / 8, CV_8UC4, (uchar *)DICT_5X5_1000_BYTES), 5, 3);
const Dictionary DICT_5X5_100_DATA =
  Dictionary(Mat(100, (5 * 5 + 7) / 8, CV_8UC4, (uchar *)DICT_5X5_1000_BYTES), 5, 3);
const Dictionary DICT_5X5_250_DATA =
  Dictionary(Mat(250, (5 * 5 + 7) / 8, CV_8UC4, (uchar *)DICT_5X5_1000_BYTES), 5, 2);
const Dictionary DICT_5X5_1000_DATA =
  Dictionary(Mat(1000, (5 * 5 + 7) / 8, CV_8UC4, (uchar *)DICT_5X5_1000_BYTES), 5, 2);

const Dictionary DICT_6X6_50_DATA =
  Dictionary(Mat(50, (6 * 6 + 7) / 8, CV_8UC4, (uchar *)DICT_6X6_1000_BYTES), 6, 6);
const Dictionary DICT_6X6_100_DATA =
  Dictionary(Mat(100, (6 * 6 + 7) / 8, CV_8UC4, (uchar *)DICT_6X6_1000_BYTES), 6, 5);
const Dictionary DICT_6X6_250_DATA =
  Dictionary(Mat(250, (6 * 6 + 7) / 8, CV_8UC4, (uchar *)DICT_6X6_1000_BYTES), 6, 5);
const Dictionary DICT_6X6_1000_DATA =
  Dictionary(Mat(1000, (6 * 6 + 7) / 8, CV_8UC4, (uchar *)DICT_6X6_1000_BYTES), 6, 4);

const Dictionary DICT_7X7_50_DATA =
  Dictionary(Mat(50, (7 * 7 + 7) / 8, CV_8UC4, (uchar *)DICT_7X7_1000_BYTES), 7, 9);
const Dictionary DICT_7X7_100_DATA =
  Dictionary(Mat(100, (7 * 7 + 7) / 8, CV_8UC4, (uchar *)DICT_7X7_1000_BYTES), 7, 8);
const Dictionary DICT_7X7_250_DATA =
  Dictionary(Mat(250, (7 * 7 + 7) / 8, CV_8UC4, (uchar *)DICT_7X7_1000_BYTES), 7, 8);
const Dictionary DICT_7X7_1000_DATA =
  Dictionary(Mat(1000, (7 * 7 + 7) / 8, CV_8UC4, (uchar *)DICT_7X7_1000_BYTES), 7, 6);

const Dictionary & getPredefinedDictionary(PREDEFINED_DICTIONARY_NAME name)
{
  switch (name) {

    case DICT_ARUCO_ORIGINAL:
      return DICT_ARUCO_DATA;

    case DICT_4X4_50:
      return DICT_4X4_50_DATA;
    case DICT_4X4_100:
      return DICT_4X4_100_DATA;
    case DICT_4X4_250:
      return DICT_4X4_250_DATA;
    case DICT_4X4_1000:
      return DICT_4X4_1000_DATA;

    case DICT_5X5_50:
      return DICT_5X5_50_DATA;
    case DICT_5X5_100:
      return DICT_5X5_100_DATA;
    case DICT_5X5_250:
      return DICT_5X5_250_DATA;
    case DICT_5X5_1000:
      return DICT_5X5_1000_DATA;

    case DICT_6X6_50:
      return DICT_6X6_50_DATA;
    case DICT_6X6_100:
      return DICT_6X6_100_DATA;
    case DICT_6X6_250:
      return DICT_6X6_250_DATA;
    case DICT_6X6_1000:
      return DICT_6X6_1000_DATA;

    case DICT_7X7_50:
      return DICT_7X7_50_DATA;
    case DICT_7X7_100:
      return DICT_7X7_100_DATA;
    case DICT_7X7_250:
      return DICT_7X7_250_DATA;
    case DICT_7X7_1000:
      return DICT_7X7_1000_DATA;

  }
  return DICT_4X4_50_DATA;
}


/**
		* @brief Generates a random marker Mat of size markerSize x markerSize
		*/
static Mat _generateRandomMarker(int markerSize)
{
  Mat marker(markerSize, markerSize, CV_8UC1, Scalar::all(0));
  for (int i = 0; i < markerSize; i++) {
    for (int j = 0; j < markerSize; j++) {
      unsigned char bit = rand() % 2;
      marker.at<unsigned char>(i, j) = bit;
    }
  }
  return marker;
}

/**
		* @brief Calculate selfDistance of the codification of a marker Mat. Self distance is the Hamming
		* distance of the marker to itself in the other rotations.
		* See S. Garrido-Jurado, R. Muñoz-Salinas, F. J. Madrid-Cuevas, and M. J. Marín-Jiménez. 2014.
		* "Automatic generation and detection of highly reliable fiducial markers under occlusion".
		* Pattern Recogn. 47, 6 (June 2014), 2280-2292. DOI=10.1016/j.patcog.2014.01.005
		*/
static int _getSelfDistance(const Mat & marker)
{
  Mat bytes = Dictionary::getByteListFromBits(marker);
  int minHamming = (int)marker.total() + 1;
  for (int r = 1; r < 4; r++) {
    int currentHamming =
      cv::hal::normHamming(bytes.ptr(), bytes.ptr() + bytes.cols * r, bytes.cols);
    if (currentHamming < minHamming) {minHamming = currentHamming;}
  }
  return minHamming;
}

/**
		*/
Dictionary generateCustomDictionary(
  int nMarkers, int markerSize,
  const Dictionary & baseDictionary)
{

  Dictionary out;
  out.markerSize = markerSize;

  // theoretical maximum intermarker distance
  // See S. Garrido-Jurado, R. Muñoz-Salinas, F. J. Madrid-Cuevas, and M. J. Marín-Jiménez. 2014.
  // "Automatic generation and detection of highly reliable fiducial markers under occlusion".
  // Pattern Recogn. 47, 6 (June 2014), 2280-2292. DOI=10.1016/j.patcog.2014.01.005
  int C = (int)std::floor(float(markerSize * markerSize) / 4.f);
  int tau = 2 * (int)std::floor(float(C) * 4.f / 3.f);

  // if baseDictionary is provided, calculate its intermarker distance
  if (baseDictionary.bytesList.rows > 0) {
    CV_Assert(baseDictionary.markerSize == markerSize);
    out.bytesList = baseDictionary.bytesList.clone();

    int minDistance = markerSize * markerSize + 1;
    for (int i = 0; i < out.bytesList.rows; i++) {
      Mat markerBytes = out.bytesList.rowRange(i, i + 1);
      Mat markerBits = Dictionary::getBitsFromByteList(markerBytes, markerSize);
      minDistance = min(minDistance, _getSelfDistance(markerBits));
      for (int j = i + 1; j < out.bytesList.rows; j++) {
        minDistance = min(minDistance, out.getDistanceToId(markerBits, j));
      }
    }
    tau = minDistance;
  }

  // current best option
  int bestTau = 0;
  Mat bestMarker;

  // after these number of unproductive iterations, the best option is accepted
  const int maxUnproductiveIterations = 5000;
  int unproductiveIterations = 0;

  while (out.bytesList.rows < nMarkers) {
    Mat currentMarker = _generateRandomMarker(markerSize);

    int selfDistance = _getSelfDistance(currentMarker);
    int minDistance = selfDistance;

    // if self distance is better or equal than current best option, calculate distance
    // to previous accepted markers
    if (selfDistance >= bestTau) {
      for (int i = 0; i < out.bytesList.rows; i++) {
        int currentDistance = out.getDistanceToId(currentMarker, i);
        minDistance = min(currentDistance, minDistance);
        if (minDistance <= bestTau) {
          break;
        }
      }
    }

    // if distance is high enough, accept the marker
    if (minDistance >= tau) {
      unproductiveIterations = 0;
      bestTau = 0;
      Mat bytes = Dictionary::getByteListFromBits(currentMarker);
      out.bytesList.push_back(bytes);
    } else {
      unproductiveIterations++;

      // if distance is not enough, but is better than the current best option
      if (minDistance > bestTau) {
        bestTau = minDistance;
        bestMarker = currentMarker;
      }

      // if number of unproductive iterarions has been reached, accept the current best option
      if (unproductiveIterations == maxUnproductiveIterations) {
        unproductiveIterations = 0;
        tau = bestTau;
        bestTau = 0;
        Mat bytes = Dictionary::getByteListFromBits(bestMarker);
        out.bytesList.push_back(bytes);
      }
    }
  }

  // update the maximum number of correction bits for the generated dictionary
  out.maxCorrectionBits = (tau - 1) / 2;

  return out;
}

}
}
