#include "dynamic-graph/python/module.hh"
#include "device.hh"

namespace dg = dynamicgraph;

BOOST_PYTHON_MODULE(wrap)
{
  bp::import("dynamic_graph.sot.core.wrap");

  dynamicgraph::python::exposeEntity<SoTUniversalRobotDevice,
    bp::bases<dg::sot::Device> >();
}
