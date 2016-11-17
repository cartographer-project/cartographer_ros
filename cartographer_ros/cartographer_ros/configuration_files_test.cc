/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer_ros/node_options.h"
#include "gtest/gtest.h"
#include "ros/package.h"

namespace cartographer_ros {
namespace {

class ConfigurationFilesTest : public ::testing::TestWithParam<const char*> {};

TEST_P(ConfigurationFilesTest, ValidateNodeOptions) {
  EXPECT_NO_FATAL_FAILURE({
    auto file_resolver = ::cartographer::common::make_unique<
        ::cartographer::common::ConfigurationFileResolver>(std::vector<string>{
        ::ros::package::getPath("cartographer_ros") + "/configuration_files"});
    const string code = file_resolver->GetFileContentOrDie(GetParam());
    ::cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
        code, std::move(file_resolver));
    ::cartographer_ros::CreateNodeOptions(&lua_parameter_dictionary);
  });
}

INSTANTIATE_TEST_CASE_P(ValidateAllNodeOptions, ConfigurationFilesTest,
                        ::testing::Values("backpack_2d.lua", "backpack_3d.lua",
                                          "pr2.lua", "revo_lds.lua"));

}  // namespace
}  // namespace cartographer_ros
