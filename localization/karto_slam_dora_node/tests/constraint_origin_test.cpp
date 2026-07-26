#include <karto_sdk/Mapper.h>

#include <cstdlib>
#include <iostream>

namespace {

void Require(bool condition, const char *message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

}  // namespace

int main()
{
  karto::Matrix3 covariance;
  covariance.SetToIdentity();

  const karto::LinkInfo sequential(
      karto::Pose2(0.0, 0.0, 0.0), karto::Pose2(1.0, 0.0, 0.0), covariance,
      karto::ConstraintSource::Sequential);
  const karto::LinkInfo loop(
      karto::Pose2(0.0, 0.0, 0.0), karto::Pose2(0.1, 0.0, 0.0), covariance,
      karto::ConstraintSource::LoopClosure);

  Require(sequential.GetConstraintSource() == karto::ConstraintSource::Sequential,
          "sequential source was not retained");
  Require(loop.GetConstraintSource() == karto::ConstraintSource::LoopClosure,
          "loop source was not retained");
  std::cout << "constraint_origin_test PASS\n";
}
