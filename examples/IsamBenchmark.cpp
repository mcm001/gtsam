#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PlanarProjectionFactor.h>

#include <random>

using namespace std;
using namespace gtsam;
using symbol_shorthand::C;
using symbol_shorthand::K;
using symbol_shorthand::L;
using symbol_shorthand::X;

int main() {
  // Example localization
  SharedNoiseModel pxModel = noiseModel::Diagonal::Sigmas(Vector2(1, 1));
  // pose model is wide, so the solver finds the right answer.
  SharedNoiseModel xNoise = noiseModel::Diagonal::Sigmas(Vector3(10, 10, 10));

  // landmarks
  Point3 l0(1, 0.1, 1);
  Point3 l1(1, -0.1, 1);

  // camera pixels
  Point2 p0(180, 0);
  Point2 p1(220, 0);

  // body
  Pose2 x0(0, 0, 0);

  // camera z looking at +x with (xy) antiparallel to (yz)
  Pose3 c0(Rot3(0, 0, 1,    //
                -1, 0, 0,   //
                0, -1, 0),  //
           Vector3(0, 0, 0));
  Cal3DS2 calib(200, 200, 0, 200, 200, 0, 0);

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  auto p = ISAM2DoglegParams();
  p.setVerbose(true);
  parameters.optimizationParams = p;
  ISAM2 isam(parameters);

  {
    NonlinearFactorGraph graph;
    graph.add(PlanarProjectionFactor1(X(0), l0, p0, c0, calib, pxModel));
    graph.add(PlanarProjectionFactor1(X(0), l1, p1, c0, calib, pxModel));
    graph.add(PriorFactor<Pose2>(X(0), x0, xNoise));

    Values initialEstimate;
    initialEstimate.insert(X(0), x0);

    isam.update(graph, initialEstimate);
    isam.calculateEstimate(X(0)).print("X(0)=");
  }
}