/**
*  @file testTimer.cpp
*  @author Jing Dong
*  @date May 27, 2016
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gpmp2/utils/Timer.h>

#include <iostream>
#include <cmath>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


/**
 * 2D posegraph-like graph optimization problem
 * the graph is 10 x 10 square, each side has 10 states
 * initialize as a circle
 *
 * p
 * |
 * x - x - x - x
 * |           |
 * x           x
 * |           |
 * x           x
 * |           |
 * x - x - x - x
 *
 */
noiseModel::Isotropic::shared_ptr model_prior = noiseModel::Isotropic::Sigma(2, 0.01);
noiseModel::Isotropic::shared_ptr model_between = noiseModel::Isotropic::Sigma(2, 0.01);
const double interval = 1.0;
const size_t side_size = 10;
const size_t nr_iter = 10;


/* ************************************************************************** */
TEST(Timer, FixedSize) {

  // initial the graph
  NonlinearFactorGraph graph;

  // prior
  graph.add(PriorFactor<Vector2>(Symbol('x', 0), Vector2(0, 0), model_prior));

  // between
  for (size_t i = 0; i < side_size; i++)
    graph.add(BetweenFactor<Vector2>(Symbol('x', i), Symbol('x', i+1),
        Vector2(interval, 0), model_between));
  for (size_t i = 0; i < side_size; i++)
    graph.add(BetweenFactor<Vector2>(Symbol('x', i+1*side_size),
        Symbol('x', i+1+1*side_size), Vector2(0, -interval), model_between));
  for (size_t i = 0; i < side_size; i++)
    graph.add(BetweenFactor<Vector2>(Symbol('x', i+2*side_size),
        Symbol('x', i+1+2*side_size), Vector2(-interval, 0), model_between));
  for (size_t i = 0; i < side_size-1; i++)
    graph.add(BetweenFactor<Vector2>(Symbol('x', i+3*side_size),
        Symbol('x', i+1+3*side_size), Vector2(0, interval), model_between));
  // last one: close loop
  graph.add(BetweenFactor<Vector2>(Symbol('x', 4*side_size-1),
      Symbol('x', 0), Vector2(0, interval), model_between));

  // initial values: circle
  Values init_values;
  for (size_t i = 0; i < 4*side_size; i++) {
    double r = interval * static_cast<double>(side_size) / 2.0;
    double theta = M_PI/2.0 + M_PI * static_cast<double>(i) / (2*side_size);
    init_values.insert(Symbol('x', i), Vector2(r + cos(theta)*r, -r + sin(theta)*r));
  }
  //init_values.print();

  Timer timer_fixed("Fixed Size");
  timer_fixed.tic();

  GaussNewtonParams parameters;
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);

  for (size_t i = 0; i < nr_iter; i++)
    optimizer.iterate();

  Values values = optimizer.values();

  timer_fixed.toc();
  //values.print();
  //EXPECT(false);
}

/* ************************************************************************** */
TEST(Timer, DynamicSize) {

  // initial the graph
  NonlinearFactorGraph graph;

  // prior
  graph.add(PriorFactor<Vector>(Symbol('x', 0), (Vector(2) << 0, 0).finished(), model_prior));

  // between
  for (size_t i = 0; i < side_size; i++)
    graph.add(BetweenFactor<Vector>(Symbol('x', i), Symbol('x', i+1),
        (Vector(2) << interval, 0).finished(), model_between));
  for (size_t i = 0; i < side_size; i++)
    graph.add(BetweenFactor<Vector>(Symbol('x', i+1*side_size),
        Symbol('x', i+1+1*side_size), (Vector(2) << 0, -interval).finished(), model_between));
  for (size_t i = 0; i < side_size; i++)
    graph.add(BetweenFactor<Vector>(Symbol('x', i+2*side_size),
        Symbol('x', i+1+2*side_size), (Vector(2) << -interval, 0).finished(), model_between));
  for (size_t i = 0; i < side_size-1; i++)
    graph.add(BetweenFactor<Vector>(Symbol('x', i+3*side_size),
        Symbol('x', i+1+3*side_size), (Vector(2) << 0, interval).finished(), model_between));
  // last one: close loop
  graph.add(BetweenFactor<Vector>(Symbol('x', 4*side_size-1),
      Symbol('x', 0), (Vector(2) << 0, interval).finished(), model_between));

  // initial values: circle
  Values init_values;
  for (size_t i = 0; i < 4*side_size; i++) {
    double r = interval * static_cast<double>(side_size) / 2.0;
    double theta = M_PI/2.0 + M_PI * static_cast<double>(i) / (2*side_size);
    init_values.insert(Symbol('x', i), (Vector(2) << r + cos(theta)*r, -r + sin(theta)*r).finished());
  }
  //init_values.print();

  Timer timer_dynamic("Dynamic Size");
  timer_dynamic.tic();

  GaussNewtonParams parameters;
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);

  for (size_t i = 0; i < nr_iter; i++)
    optimizer.iterate();

  Values values = optimizer.values();

  timer_dynamic.toc();
  //values.print();
  //EXPECT(false);
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
