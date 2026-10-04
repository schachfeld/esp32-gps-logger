#include <unity.h>

#include "navigation_fix.h"

using gps_logger::NavigationFix;

void test_formats_coordinate_with_leading_zeroes() {
  TEST_ASSERT_EQUAL_STRING("13.0000123", gps_logger::formatCoordinateE7(130000123).c_str());
}
void test_formats_negative_coordinate() {
  TEST_ASSERT_EQUAL_STRING("-122.0845678",
                           gps_logger::formatCoordinateE7(-1220845678).c_str());
}

void test_formats_negative_coordinate_below_one_degree() {
  TEST_ASSERT_EQUAL_STRING("-0.1234567", gps_logger::formatCoordinateE7(-1234567).c_str());
}

void test_rejects_dead_reckoning_and_time_only_fixes() {
  NavigationFix fix;
  fix.gnss_fix_ok = true;
  fix.invalid_llh = false;

  fix.fix_type = 1;
  TEST_ASSERT_FALSE(gps_logger::hasUsablePosition(fix));
  fix.fix_type = 5;
  TEST_ASSERT_FALSE(gps_logger::hasUsablePosition(fix));
}

void test_accepts_valid_2d_3d_and_combined_fixes() {
  NavigationFix fix;
  fix.gnss_fix_ok = true;
  fix.invalid_llh = false;

  for (const auto type : {2, 3, 4}) {
    fix.fix_type = type;
    TEST_ASSERT_TRUE(gps_logger::hasUsablePosition(fix));
  }
}

void test_requires_resolved_utc() {
  NavigationFix fix;
  fix.utc = {2026, 9, 17, 12, 34, 56, 789, true, true, false};
  TEST_ASSERT_FALSE(gps_logger::hasValidUtc(fix));
  fix.utc.fully_resolved = true;
  TEST_ASSERT_TRUE(gps_logger::hasValidUtc(fix));
  TEST_ASSERT_EQUAL_STRING("2026-09-17T12:34:56.789Z", gps_logger::formatUtc(fix.utc).c_str());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_formats_coordinate_with_leading_zeroes);
  RUN_TEST(test_formats_negative_coordinate);
  RUN_TEST(test_formats_negative_coordinate_below_one_degree);
  RUN_TEST(test_rejects_dead_reckoning_and_time_only_fixes);
  RUN_TEST(test_accepts_valid_2d_3d_and_combined_fixes);
  RUN_TEST(test_requires_resolved_utc);
  return UNITY_END();
}
