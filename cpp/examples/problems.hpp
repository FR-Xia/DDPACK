#pragma once
#include <ddpack/ddpack.hpp>
namespace ddpack::examples {
struct Case { Problem problem; Options options; };
std::vector<std::string> names();
Case make(const std::string& name);
}
