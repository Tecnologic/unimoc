/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file node_identity_test.cpp
 *    @brief ztest coverage for the Cyphal node identity record.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "cyphal/node_identity.hpp"
#include <zephyr/ztest.h>
#include <cstring>
#include <string>
#include <string_view>
#include "unimoc_test.hpp"

namespace unimoc::cyphal::test {

// --- Default construction
ZTEST(node_identity, test_default_name) {
  NodeIdentity identity;
  zexpect_str_equal(&identity.name[0], "unimoc");
}

ZTEST(node_identity, test_default_versions) {
  NodeIdentity identity;
  zexpect_equal(identity.hw_version_major, 1U);
  zexpect_equal(identity.hw_version_minor, 0U);
  zexpect_equal(identity.sw_version_major, 1U);
  zexpect_equal(identity.sw_version_minor, 0U);
}

// --- set_name / get_name
ZTEST(node_identity, test_set_and_get_name) {
  NodeIdentity identity;
  identity.set_name("my.drive.node");
  zexpect_equal(identity.get_name(), std::string_view("my.drive.node"));
}

ZTEST(node_identity, test_set_name_truncates_at_max_len) {
  NodeIdentity identity;
  // Build a 60-char string — should be truncated to NODE_NAME_MAX_LEN (50)
  const std::string kLongName(60, 'x');
  identity.set_name(kLongName);
  zexpect_equal(identity.get_name().size(), static_cast<std::size_t>(NODE_NAME_MAX_LEN));
  zexpect_equal(identity.name[NODE_NAME_MAX_LEN], '\0');
}

ZTEST(node_identity, test_set_name_empty) {
  NodeIdentity identity;
  identity.set_name("");
  zexpect_equal(identity.get_name().size(), 0U);
  zexpect_equal(identity.name[0], '\0');
}

ZTEST(node_identity, test_set_name_exactly_max_len) {
  NodeIdentity identity;
  const std::string kExact(NODE_NAME_MAX_LEN, 'a');
  identity.set_name(kExact);
  zexpect_equal(identity.get_name().size(), static_cast<std::size_t>(NODE_NAME_MAX_LEN));
  zexpect_equal(identity.name[NODE_NAME_MAX_LEN], '\0');
}

// --- Equality
ZTEST(node_identity, test_equality_default) {
  NodeIdentity a;
  NodeIdentity b;
  zexpect_equal(a, b);
}

ZTEST(node_identity, test_inequality_name) {
  NodeIdentity a;
  NodeIdentity b;
  b.set_name("other");
  zexpect_not_equal(a, b);
}

ZTEST(node_identity, test_inequality_hw_version) {
  NodeIdentity a;
  NodeIdentity b;
  b.hw_version_major = 2U;
  zexpect_not_equal(a, b);
}

}  // namespace unimoc::cyphal::test

ZTEST_SUITE(node_identity, nullptr, nullptr, nullptr, nullptr, nullptr);
