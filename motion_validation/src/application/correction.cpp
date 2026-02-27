#include "application/correction.hpp"

using namespace net::wavem::drive;

Correction::Correction(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter, const Vertification::SharedPtr &vertification)
    : node_(node),
      parameter_(parameter),
      vertification_(vertification)
{
}

Correction::~Correction()
{
}