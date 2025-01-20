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
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PlanarProjectionFactor.h>

#include <random>

using namespace std;
using namespace gtsam;
using symbol_shorthand::C;
using symbol_shorthand::K;
using symbol_shorthand::L;
using symbol_shorthand::X;

int main() {
  // Pixel noise, in pixels, uv
  SharedNoiseModel pxModel = noiseModel::Diagonal::Sigmas(Vector2(1, 1));
  // Pose between factor noise -- x, y, theta
  noiseModel::Diagonal::shared_ptr model =
      noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  // landmarks
  std::vector<Point3> tagPoints{
      Point3{2.5, 0 - 0.08255, 0.5 - 0.08255},
      Point3{2.5, 0 - 0.08255, 0.5 + 0.08255},
      Point3{2.5, 0 + 0.08255, 0.5 + 0.08255},
      Point3{2.5, 0 + 0.08255, 0.5 - 0.08255},
  };

  // camera pixels
  std::vector<Point2> observations{
      Point2{333, -17},
      Point2{333, -83},
      Point2{267, -83},
      Point2{267, -17},
  };

  // Initial guess for world2body
  Pose2 x0(0, 0, 0);

  // camera z looking at +x with (xy) antiparallel to (yz)
  Pose3 c0(Rot3(0, 0, 1,    //
                -1, 0, 0,   //
                0, -1, 0),  //
           Vector3(0, 0, 0));
  Cal3DS2 calib(600, 600, 0, 300, 150, 0, 0);

  ISAM2Params parameters;
  // parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 1;

  // auto p = ISAM2DoglegParams();
  // p.setVerbose(false);
  // parameters.optimizationParams = p;

  ISAM2 isam(parameters);

  for (int i = 0; i < 20; i++) {
    cout << "========================" << endl << "Iteration " << i << endl;

    NonlinearFactorGraph graph;
    Values initialEstimate;

    for (int j = 0; j < 4; j++) {
      graph.add(PlanarProjectionFactor1(X(i), tagPoints[j], observations[j], c0,
                                        calib, pxModel));
    }

    if (i != 0) {
      graph.emplace_shared<BetweenFactor<Pose2>>(X(i - 1), X(i), Pose2(0, 0, 0),
                                                 model);

      initialEstimate.insert(X(i), isam.calculateEstimate(X(i-1)).cast<Pose2>());
    } else {
      initialEstimate.insert(X(i), x0);
    }
    

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    isam.update(graph, initialEstimate);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    
    isam.calculateEstimate().print("");

    chrono::duration<double, std::micro> timeUsed1 = t2 - t1;
    cout << "Number of factors: " << isam.size() << " time used (us) "
         << timeUsed1.count() << endl;
  }
}