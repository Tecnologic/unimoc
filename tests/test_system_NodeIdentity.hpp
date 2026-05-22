#pragma once

#ifndef UNIMOC_TEST_NODE_IDENTITY_H_
#define UNIMOC_TEST_NODE_IDENTITY_H_

#include <gtest/gtest.h>
#include <cstring>
#include <string_view>
#include "NodeIdentity.hpp"

namespace unimoc
{
namespace system
{
namespace test
{

class NodeIdentityTest : public ::testing::Test {};

// --- Default construction
TEST_F(NodeIdentityTest, DefaultName)
{
    NodeIdentity id;
    EXPECT_STREQ(id.name, "unimoc");
}

TEST_F(NodeIdentityTest, DefaultVersions)
{
    NodeIdentity id;
    EXPECT_EQ(id.hw_version_major, 1u);
    EXPECT_EQ(id.hw_version_minor, 0u);
    EXPECT_EQ(id.sw_version_major, 1u);
    EXPECT_EQ(id.sw_version_minor, 0u);
}

TEST_F(NodeIdentityTest, DefaultUniqueIdZero)
{
    NodeIdentity id;
    for (auto b : id.unique_id)
        EXPECT_EQ(b, 0u);
}

// --- set_name / get_name
TEST_F(NodeIdentityTest, SetAndGetName)
{
    NodeIdentity id;
    id.set_name("my.drive.node");
    EXPECT_EQ(id.get_name(), std::string_view("my.drive.node"));
}

TEST_F(NodeIdentityTest, SetNameTruncatesAtMaxLen)
{
    NodeIdentity id;
    // Build a 60-char string — should be truncated to NODE_NAME_MAX_LEN (50)
    const std::string long_name(60, 'x');
    id.set_name(long_name);
    EXPECT_EQ(id.get_name().size(), static_cast<std::size_t>(NODE_NAME_MAX_LEN));
    EXPECT_EQ(id.name[NODE_NAME_MAX_LEN], '\0');
}

TEST_F(NodeIdentityTest, SetNameEmpty)
{
    NodeIdentity id;
    id.set_name("");
    EXPECT_EQ(id.get_name().size(), 0u);
    EXPECT_EQ(id.name[0], '\0');
}

TEST_F(NodeIdentityTest, SetNameExactlyMaxLen)
{
    NodeIdentity id;
    const std::string exact(NODE_NAME_MAX_LEN, 'a');
    id.set_name(exact);
    EXPECT_EQ(id.get_name().size(), static_cast<std::size_t>(NODE_NAME_MAX_LEN));
    EXPECT_EQ(id.name[NODE_NAME_MAX_LEN], '\0');
}

// --- Equality
TEST_F(NodeIdentityTest, EqualityDefault)
{
    NodeIdentity a, b;
    EXPECT_EQ(a, b);
}

TEST_F(NodeIdentityTest, InequalityName)
{
    NodeIdentity a, b;
    b.set_name("other");
    EXPECT_NE(a, b);
}

TEST_F(NodeIdentityTest, InequalityHwVersion)
{
    NodeIdentity a, b;
    b.hw_version_major = 2u;
    EXPECT_NE(a, b);
}

TEST_F(NodeIdentityTest, UniqueIdNotCompared)
{
    // unique_id differences should NOT affect operator==
    NodeIdentity a, b;
    b.unique_id[0] = 0xFFu;
    EXPECT_EQ(a, b);
}

}  // namespace test
}  // namespace system
}  // namespace unimoc

#endif /* UNIMOC_TEST_NODE_IDENTITY_H_ */
