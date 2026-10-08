#include <gtest/gtest.h>
#include <cmath>
#include "current_controller.hpp"
#include "dead_time_compensation.hpp"
#include "field_weakening.hpp"
#include "mtpa.hpp"

using DimensionlessStator = unimoc::system::Stator<unimoc::unit::Ratio>;
using CurrentStator = unimoc::system::Stator<unimoc::unit::Current>;
using CurrentRotor = unimoc::system::Rotor<unimoc::unit::Current>;
using VoltageRotor = unimoc::system::Rotor<unimoc::unit::Voltage>;
using namespace unimoc::unit;

TEST(DeadTimeCompensationTest, ReconstructsAndTransformsPhaseSigns) {
  unimoc::settings::NvmSettings settings;
  settings.dtc_dead_time = 2.0_us;
  settings.dtc_f_pwm = 10.0_kHz;
  settings.dtc_i_threshold = 1.0_A;

  unimoc::control::DeadTimeCompensation<float> compensation;
  compensation.init(settings);

  const auto result = compensation.calculate(CurrentStator{1.0F, 0.0F});

  EXPECT_NEAR(result.alpha.Value(), 0.02F, 1.0e-6F);
  EXPECT_NEAR(result.beta.Value(), 0.0F, 1.0e-6F);
}

TEST(DeadTimeCompensationTest, SoftSignInterpolatesAndSaturates) {
  unimoc::control::DeadTimeCompensation<float> compensation;
  compensation.dead_time = 2.0_us;
  compensation.f_pwm = 10.0_kHz;
  compensation.i_threshold = 1.0_A;

  const auto interpolated = compensation.calculate(CurrentStator{0.5F, 0.0F});
  const auto saturated = compensation.calculate(CurrentStator{2.0F, 0.0F});

  EXPECT_NEAR(interpolated.alpha.Value(), 0.01F, 1.0e-6F);
  EXPECT_NEAR(saturated.alpha.Value(), 0.0266667F, 1.0e-6F);
}

TEST(MtpaTest, ZeroSaliencyReturnsZero) {
  unimoc::control::Mtpa<float> mtpa;
  mtpa.flux_pm = 0.1_Wb;
  mtpa.L_d = 1.0_mH;
  mtpa.L_q = 1.0_mH;

  EXPECT_FLOAT_EQ(mtpa.calculate(2.0_A).Value(), 0.0F);
}

TEST(MtpaTest, CalculatesAndLimitsDaxisCurrent) {
  unimoc::control::Mtpa<float> mtpa;
  mtpa.flux_pm = 0.0_Wb;
  mtpa.L_d = 2.0_mH;
  mtpa.L_q = 1.0_mH;
  EXPECT_FLOAT_EQ(mtpa.calculate(2.0_A).Value(), -1.0F);

  mtpa.flux_pm = 0.01_Wb;
  mtpa.L_d = 1.0_mH;
  mtpa.L_q = 2.0_mH;
  EXPECT_FLOAT_EQ(mtpa.calculate(1.0_A).Value(), -1.0F);
}

TEST(FieldWeakeningTest, IntegratesOnlyNegativeVoltageHeadroom) {
  unimoc::control::FieldWeakening<float> field_weakening;
  field_weakening.v_max = 0.9_V;
  field_weakening.ki = 10.0_A_per_V_s;
  field_weakening.i_d_min = -0.5_A;

  EXPECT_FLOAT_EQ(field_weakening.update(unimoc::system::Stator<unimoc::unit::Voltage>{0.5_V, 0.0_V}, 0.1_s)
                      .Value(),
                  0.0F);
  EXPECT_NEAR(field_weakening.update(unimoc::system::Stator<unimoc::unit::Voltage>{1.0_V, 0.0_V}, 0.1_s)
                  .Value(),
              -0.1F,
              1.0e-6F);
  EXPECT_FLOAT_EQ(field_weakening.update(unimoc::system::Stator<unimoc::unit::Voltage>{10.0_V, 0.0_V}, 1.0_s)
                      .Value(),
                  -0.5F);

  field_weakening.reset();
  EXPECT_FLOAT_EQ(field_weakening.i_d_fw.Value(), 0.0F);
}

TEST(CurrentControllerTest, UnsaturatedPiStepUpdatesIntegrator) {
  unimoc::control::CurrentController<float> controller;
  controller.kp_d = 2.0_V_per_A;
  controller.ki_d = 10.0_V_per_A_s;
  controller.v_max = 1.0_ratio;

  const VoltageRotor output =
      controller.update(CurrentRotor{1.0F, 0.0F}, CurrentRotor{0.0F, 0.0F}, 0.0_rad_per_s, 0.1_s, 10.0_V);

  EXPECT_FLOAT_EQ(output.d.Value(), 2.0F);
  EXPECT_FLOAT_EQ(output.q.Value(), 0.0F);
  EXPECT_FLOAT_EQ(controller.integrator_d.Value(), 1.0F);
}

TEST(CurrentControllerTest, CircularLimitScalesBothAxes) {
  unimoc::control::CurrentController<float> controller;
  controller.kp_d = 2.0_V_per_A;
  controller.kp_q = 2.0_V_per_A;
  controller.ki_d = 0.0_V_per_A_s;
  controller.ki_q = 0.0_V_per_A_s;
  controller.kb_d = 0.0_per_s;
  controller.kb_q = 0.0_per_s;
  controller.v_max = 0.1_ratio;

  const VoltageRotor output =
      controller.update(CurrentRotor{1.0F, 1.0F}, CurrentRotor{0.0F, 0.0F}, 0.0_rad_per_s, 0.1_s, 10.0_V);
  const float magnitude =
      std::sqrt(output.d.Value() * output.d.Value() + output.q.Value() * output.q.Value());

  EXPECT_NEAR(magnitude, 1.0F, 1.0e-6F);
  EXPECT_NEAR(output.d.Value(), output.q.Value(), 1.0e-6F);
}