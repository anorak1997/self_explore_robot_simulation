// Copyright 2026 Siddhartha Dubey
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BOPT_BOX_FILTER__TRANSFORM2D_HPP_
#define BOPT_BOX_FILTER__TRANSFORM2D_HPP_

namespace bopt_box_filter
{

// XY slice of a 3D rigid transform. Applied to a ray endpoint (x_l, y_l, 0)
// in the source (laser) frame to yield (x_t, y_t) in the target frame:
//   x_t = r00 * x_l + r01 * y_l + tx
//   y_t = r10 * x_l + r11 * y_l + ty
// Z of the transform is dropped because rays are 2D and the box is XY-only.
struct Transform2D
{
  double r00 {1.0};
  double r01 {0.0};
  double r10 {0.0};
  double r11 {1.0};
  double tx {0.0};
  double ty {0.0};
};

}  // namespace bopt_box_filter

#endif  // BOPT_BOX_FILTER__TRANSFORM2D_HPP_
