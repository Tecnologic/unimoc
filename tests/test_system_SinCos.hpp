#pragma once

#ifndef UNIMOC_SYSTEM_TEST_SIN_COS_GTEST_H_
#define UNIMOC_SYSTEM_TEST_SIN_COS_GTEST_H_

#include <gtest/gtest.h>
#include "SinCos.hpp"

namespace unimoc
{
    namespace system
    {
        namespace test
        {
            class SinCosTest : public ::testing::Test
            {
            protected:
                using SinCosF = SinCos<float>;
            };

            TEST_F(SinCosTest, DefaultConstructor)
            {
                SinCosF defaultSinCos;
                EXPECT_FLOAT_EQ(defaultSinCos.sin, 0.0f);
                EXPECT_FLOAT_EQ(defaultSinCos.cos, 0.0f);
            }

            TEST_F(SinCosTest, ConstructorWithValues)
            {
                SinCosF sinCos(0.5f, 0.866f);
                EXPECT_NEAR(sinCos.sin, 0.5f, 1e-6f);
                EXPECT_NEAR(sinCos.cos, 0.866f, 1e-6f);
            }

            TEST_F(SinCosTest, CopyConstructor)
            {
                SinCosF sinCos1(0.5f, 0.866f);
                SinCosF sinCos2(sinCos1);
                EXPECT_EQ(sinCos1, sinCos2);
            }

            TEST_F(SinCosTest, MoveConstructor)
            {
                SinCosF sinCos1(0.5f, 0.866f);
                SinCosF sinCos2(std::move(sinCos1));
                EXPECT_FLOAT_EQ(sinCos2.sin, 0.5f);
                EXPECT_FLOAT_EQ(sinCos2.cos, 0.866f);
            }

            TEST_F(SinCosTest, CopyAssignment)
            {
                SinCosF sinCos1(0.5f, 0.866f);
                SinCosF sinCos2;
                sinCos2 = sinCos1;
                EXPECT_EQ(sinCos1, sinCos2);
            }

            TEST_F(SinCosTest, MoveAssignment)
            {
                SinCosF sinCos1(0.5f, 0.866f);
                SinCosF sinCos2;
                sinCos2 = std::move(sinCos1);
                EXPECT_FLOAT_EQ(sinCos2.sin, 0.5f);
                EXPECT_FLOAT_EQ(sinCos2.cos, 0.866f);
            }

            TEST_F(SinCosTest, EqualityAndInequality)
            {
                SinCosF sinCos1(0.5f, 0.866f);
                SinCosF sinCos2(0.5f, 0.866f);
                SinCosF sinCos3(0.707f, 0.707f);
                EXPECT_EQ(sinCos1, sinCos2);
                EXPECT_NE(sinCos1, sinCos3);
            }

            TEST_F(SinCosTest, LengthCalculation)
            {
                SinCosF sinCos(3.0f, 4.0f);
                EXPECT_NEAR(sinCos.length(), 5.0f, 1e-6f);
            }

            TEST_F(SinCosTest, Normalization)
            {
                SinCosF sinCos(3.0f, 4.0f);
                auto normalized = sinCos.normToOne();
                EXPECT_NEAR(normalized.length(), 1.0f, 1e-6f);
            }

            TEST_F(SinCosTest, ToArray)
            {
                SinCosF sinCos(0.5f, 0.866f);
                auto array = sinCos.to_array();
                EXPECT_NEAR(array[0], 0.5f, 1e-6f);
                EXPECT_NEAR(array[1], 0.866f, 1e-6f);
            }

            TEST_F(SinCosTest, NegationOperator)
            {
                SinCosF sinCos(0.5f, 0.866f);
                auto negated = -sinCos;
                EXPECT_NEAR(negated.sin, -0.5f, 1e-6f);
                EXPECT_NEAR(negated.cos, -0.866f, 1e-6f);
            }

            TEST_F(SinCosTest, SineCosineDifference)
            {
                SinCosF sinCos1(0.70711f, 0.70711f); // 45 degrees
                SinCosF sinCos2(0.5f, 0.86603f);     // 30 degrees
                auto diff = sinCos1 - sinCos2;
                EXPECT_NEAR(diff.sin, 0.2588f, 1e-4f); // sin(45 - 30) = sin(15)
                EXPECT_NEAR(diff.cos, 0.9659f, 1e-4f); // cos(45 - 30) = cos(15)
            }

            TEST_F(SinCosTest, ZeroLengthNormalization)
            {
                SinCosF sinCos(0.0f, 0.0f);
                // normToOne is non-throwing; returns the unchanged zero vector
                auto result = sinCos.normToOne();
                EXPECT_FLOAT_EQ(result.sin, 0.0f);
                EXPECT_FLOAT_EQ(result.cos, 0.0f);
            }
        } // namespace test
    }     // namespace system
} // namespace unimoc

#endif /* UNIMOC_SYSTEM_TEST_SIN_COS_GTEST_H_ */