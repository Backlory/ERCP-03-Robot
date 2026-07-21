#include <yaml-cpp/fptostring.h>

#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>

namespace YAML {
namespace {
template <typename T>
std::string FormatFloatingPoint(T value, std::size_t precision) {
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(
      precision == 0 ? std::numeric_limits<T>::max_digits10
                     : static_cast<int>(precision));
  stream << value;
  return stream.str();
}
}  // namespace

std::string FpToString(float value, std::size_t precision) {
  return FormatFloatingPoint(value, precision);
}

std::string FpToString(double value, std::size_t precision) {
  return FormatFloatingPoint(value, precision);
}

std::string FpToString(long double value, std::size_t precision) {
  return FormatFloatingPoint(value, precision);
}
}  // namespace YAML
