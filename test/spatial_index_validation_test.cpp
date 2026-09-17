#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include <m-detector/spatial_index_validation.h>

namespace spatial_index = m_detector::spatial_index;

namespace
{
constexpr int kHorizontalSize = 1257;
constexpr int kVerticalSize = 449;
constexpr int kPositionSize = 564393;

TEST(FixedStorageIndex, RejectsVoxelAndDepthMapBoundaries)
{
    constexpr std::size_t size = 10;
    EXPECT_FALSE(spatial_index::IsIndexInRange(-1, size));
    EXPECT_TRUE(spatial_index::IsIndexInRange(0, size));
    EXPECT_TRUE(spatial_index::IsIndexInRange(size - 1, size));
    EXPECT_FALSE(spatial_index::IsIndexInRange(size, size));
    EXPECT_FALSE(spatial_index::IsIndexInRange(size + 1, size));
    EXPECT_FALSE(spatial_index::IsIndexInRange(std::numeric_limits<std::int64_t>::max(), size));

    EXPECT_FALSE(spatial_index::IsIndexInRange(-1, kPositionSize));
    EXPECT_TRUE(spatial_index::IsIndexInRange(0, kPositionSize));
    EXPECT_TRUE(spatial_index::IsIndexInRange(kPositionSize - 1, kPositionSize));
    EXPECT_FALSE(spatial_index::IsIndexInRange(kPositionSize, kPositionSize));
    EXPECT_FALSE(spatial_index::IsIndexInRange(std::numeric_limits<int>::max(), kPositionSize));
}

TEST(Projection, PreservesRepresentativeFiniteProjection)
{
    spatial_index::Projection projection;
    ASSERT_TRUE(spatial_index::ProjectCartesian(
        1.0, 2.0, 3.0, 0.005f, 0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));

    const float expected_horizontal = std::atan2(2.0f, 1.0f);
    const float expected_vertical = std::atan2(
        3.0f, static_cast<float>(std::sqrt(std::pow(1.0f, 2) + std::pow(2.0f, 2))));
    const float expected_depth = static_cast<float>(std::sqrt(1.0 + 4.0 + 9.0));
    EXPECT_FLOAT_EQ(projection.horizontal_angle, expected_horizontal);
    EXPECT_FLOAT_EQ(projection.vertical_angle, expected_vertical);
    EXPECT_FLOAT_EQ(projection.depth, expected_depth);
    EXPECT_EQ(projection.horizontal_index,
              static_cast<int>(std::floor((expected_horizontal + 3.141593f) / 0.005f)));
    EXPECT_EQ(projection.vertical_index,
              static_cast<int>(std::floor((expected_vertical + 0.5f * 3.141593f) / 0.01f)));
    EXPECT_EQ(projection.position,
              projection.horizontal_index * kVerticalSize + projection.vertical_index);
}

TEST(Projection, PreservesUpstreamFloatPixelArithmeticAtBoundary)
{
    constexpr double x = 321.0797258836692;
    constexpr double y = 900.82722719292997;
    constexpr double z = -197.77856227419556;
    spatial_index::Projection projection;
    ASSERT_TRUE(spatial_index::ProjectCartesian(
        x, y, z, 0.005f, 0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));

    const float horizontal_angle = std::atan2(static_cast<float>(y), static_cast<float>(x));
    const float planar_depth = static_cast<float>(std::sqrt(
        std::pow(static_cast<float>(x), 2) + std::pow(static_cast<float>(y), 2)));
    const float vertical_angle = std::atan2(static_cast<float>(z), planar_depth);
    const int upstream_horizontal_index = static_cast<int>(
        std::floor((horizontal_angle + 3.141593f) / 0.005f));
    const int upstream_vertical_index = static_cast<int>(
        std::floor((vertical_angle + 0.5f * 3.141593f) / 0.01f));

    EXPECT_EQ(projection.horizontal_index, upstream_horizontal_index);
    EXPECT_EQ(projection.vertical_index, upstream_vertical_index);
    EXPECT_EQ(projection.horizontal_index, 874);
}

TEST(Projection, HandlesZeroRangeWithoutInvalidConversion)
{
    spatial_index::Projection projection;
    ASSERT_TRUE(spatial_index::ProjectCartesian(
        0.0, 0.0, 0.0, 0.005f, 0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
    EXPECT_FLOAT_EQ(projection.depth, 0.0f);
    EXPECT_TRUE(spatial_index::IsProjectionValid(
        projection, kHorizontalSize, kVerticalSize, kPositionSize));
}

TEST(Projection, AcceptsFirstAndLastPixelsAndRejectsOnePastGrid)
{
    EXPECT_TRUE(spatial_index::IsProjectionIndexValid(
        0, 0, 0, kHorizontalSize, kVerticalSize, kPositionSize));
    EXPECT_TRUE(spatial_index::IsProjectionIndexValid(
        kHorizontalSize - 1, kVerticalSize - 1, kPositionSize - 1,
        kHorizontalSize, kVerticalSize, kPositionSize));
    EXPECT_FALSE(spatial_index::IsProjectionIndexValid(
        kHorizontalSize, 0, kPositionSize,
        kHorizontalSize, kVerticalSize, kPositionSize));
    EXPECT_FALSE(spatial_index::IsProjectionIndexValid(
        0, kVerticalSize, kVerticalSize,
        kHorizontalSize, kVerticalSize, kPositionSize));
    EXPECT_FALSE(spatial_index::IsProjectionIndexValid(
        0, 0, kPositionSize,
        kHorizontalSize, kVerticalSize, kPositionSize));
}

TEST(Projection, RejectsComputedOutOfGridValues)
{
    spatial_index::Projection projection;
    EXPECT_FALSE(spatial_index::ProjectCartesian(
        1.0, 0.0, 0.0, 0.001f, 0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
    EXPECT_FALSE(spatial_index::ProjectCartesian(
        1.0, 0.0, 0.0, 0.005f, 0.001f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
}

TEST(Projection, RejectsNonFiniteCoordinatesBeforeConversion)
{
    spatial_index::Projection projection;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double positive_infinity = std::numeric_limits<double>::infinity();
    const double negative_infinity = -positive_infinity;

    EXPECT_FALSE(spatial_index::ProjectCartesian(
        nan, 0.0, 0.0, 0.005f, 0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
    EXPECT_FALSE(spatial_index::ProjectCartesian(
        0.0, nan, 0.0, 0.005f, 0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
    EXPECT_FALSE(spatial_index::ProjectCartesian(
        0.0, 0.0, nan, 0.005f, 0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
    EXPECT_FALSE(spatial_index::ProjectCartesian(
        positive_infinity, 0.0, 0.0, 0.005f, 0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
    EXPECT_FALSE(spatial_index::ProjectCartesian(
        negative_infinity, 0.0, 0.0, 0.005f, 0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
}

TEST(Projection, RejectsInvalidResolutionBeforeConversion)
{
    spatial_index::Projection projection;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();

    EXPECT_FALSE(spatial_index::ProjectCartesian(
        1.0, 2.0, 3.0, 0.0f, 0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
    EXPECT_FALSE(spatial_index::ProjectCartesian(
        1.0, 2.0, 3.0, -0.005f, 0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
    EXPECT_FALSE(spatial_index::ProjectCartesian(
        1.0, 2.0, 3.0, nan, 0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
    EXPECT_FALSE(spatial_index::ProjectCartesian(
        1.0, 2.0, 3.0, infinity, 0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
    EXPECT_FALSE(spatial_index::ProjectCartesian(
        1.0, 2.0, 3.0, 0.005f, 0.0f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
    EXPECT_FALSE(spatial_index::ProjectCartesian(
        1.0, 2.0, 3.0, 0.005f, -0.01f,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
    EXPECT_FALSE(spatial_index::ProjectCartesian(
        1.0, 2.0, 3.0, 0.005f, nan,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
    EXPECT_FALSE(spatial_index::ProjectCartesian(
        1.0, 2.0, 3.0, 0.005f, infinity,
        kHorizontalSize, kVerticalSize, kPositionSize, &projection));
}

TEST(Projection, RejectsCachedInvalidProjection)
{
    spatial_index::Projection cached;
    cached.horizontal_angle = 0.0f;
    cached.vertical_angle = 0.0f;
    cached.depth = 1.0f;
    cached.horizontal_index = 0;
    cached.vertical_index = 0;
    cached.position = kPositionSize;
    EXPECT_FALSE(spatial_index::IsProjectionValid(
        cached, kHorizontalSize, kVerticalSize, kPositionSize));

    cached.position = 1;
    EXPECT_FALSE(spatial_index::IsProjectionValid(
        cached, kHorizontalSize, kVerticalSize, kPositionSize));

    cached.horizontal_index = 0;
    cached.vertical_index = 0;
    cached.position = 0;
    cached.depth = -1.0f;
    EXPECT_FALSE(spatial_index::IsProjectionValid(
        cached, kHorizontalSize, kVerticalSize, kPositionSize));
}

TEST(HistoryLink, ValidatesEveryTupleLevel)
{
    constexpr int first_map = 20;
    constexpr std::size_t map_count = 3;
    constexpr std::size_t pixel_size = 2;

    EXPECT_TRUE(spatial_index::IsHistoryLinkValid(
        21, first_map, map_count, 0, 1, pixel_size, kPositionSize));
    EXPECT_FALSE(spatial_index::IsHistoryLinkValid(
        -1, first_map, map_count, 0, 1, pixel_size, kPositionSize));
    EXPECT_FALSE(spatial_index::IsHistoryLinkValid(
        23, first_map, map_count, 0, 1, pixel_size, kPositionSize));
    EXPECT_FALSE(spatial_index::IsHistoryLinkValid(
        19, first_map, map_count, 0, 1, pixel_size, kPositionSize));
    EXPECT_FALSE(spatial_index::IsHistoryLinkValid(
        21, first_map, map_count, -1, 1, pixel_size, kPositionSize));
    EXPECT_FALSE(spatial_index::IsHistoryLinkValid(
        21, first_map, map_count, kPositionSize, 1, pixel_size, kPositionSize));
    EXPECT_FALSE(spatial_index::IsHistoryLinkValid(
        21, first_map, map_count, 0, -1, pixel_size, kPositionSize));
    EXPECT_FALSE(spatial_index::IsHistoryLinkValid(
        21, first_map, map_count, 0, 2, pixel_size, kPositionSize));
    EXPECT_FALSE(spatial_index::IsHistoryLinkValid(
        21, first_map, map_count, 0, 0, 0, kPositionSize));
}

TEST(CacheIndex, RejectsNegativeAndStaleReferences)
{
    std::size_t index = 99;
    EXPECT_FALSE(spatial_index::TryCacheSlot(-1, 19, &index));
    EXPECT_TRUE(spatial_index::TryCacheSlot(19, 19, &index));
    EXPECT_EQ(index, 0u);

    EXPECT_TRUE(spatial_index::TryRelativeIndex(32, 32, 17, &index));
    EXPECT_EQ(index, 0u);
    EXPECT_TRUE(spatial_index::TryRelativeIndex(48, 32, 17, &index));
    EXPECT_EQ(index, 16u);
    EXPECT_FALSE(spatial_index::TryRelativeIndex(31, 32, 17, &index));
    EXPECT_FALSE(spatial_index::TryRelativeIndex(49, 32, 17, &index));
}

TEST(HistoryCapacity, EnforcesFixedStorageLimit)
{
    EXPECT_FALSE(spatial_index::IsHistoryCapacityValid(0, 17, 19));
    EXPECT_TRUE(spatial_index::IsHistoryCapacityValid(1, 17, 19));
    EXPECT_TRUE(spatial_index::IsHistoryCapacityValid(17, 17, 19));
    EXPECT_FALSE(spatial_index::IsHistoryCapacityValid(18, 17, 19));
    EXPECT_FALSE(spatial_index::IsHistoryCapacityValid(25, 17, 19));
}

}  // namespace

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
