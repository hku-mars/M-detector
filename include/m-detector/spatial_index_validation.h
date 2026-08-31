#ifndef M_DETECTOR_SPATIAL_INDEX_VALIDATION_H
#define M_DETECTOR_SPATIAL_INDEX_VALIDATION_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace m_detector
{
namespace spatial_index
{

struct Projection
{
    float horizontal_angle = 0.0f;
    float vertical_angle = 0.0f;
    float depth = 0.0f;
    int horizontal_index = -1;
    int vertical_index = -1;
    int position = -1;
};

inline bool IsIndexInRange(std::int64_t index, std::size_t size)
{
    return index >= 0 && static_cast<std::uint64_t>(index) < size;
}

inline bool IsProjectionIndexValid(int horizontal_index, int vertical_index, int position,
                                   int horizontal_size, int vertical_size, int position_size)
{
    if (horizontal_size <= 0 || vertical_size <= 0 || position_size <= 0 ||
        !IsIndexInRange(horizontal_index, static_cast<std::size_t>(horizontal_size)) ||
        !IsIndexInRange(vertical_index, static_cast<std::size_t>(vertical_size)) ||
        !IsIndexInRange(position, static_cast<std::size_t>(position_size)))
    {
        return false;
    }

    const std::int64_t expected_position =
        static_cast<std::int64_t>(horizontal_index) * vertical_size + vertical_index;
    return expected_position == position;
}

inline bool IsProjectionValid(const Projection &projection,
                              int horizontal_size, int vertical_size, int position_size)
{
    return std::isfinite(projection.horizontal_angle) &&
           std::isfinite(projection.vertical_angle) &&
           std::isfinite(projection.depth) && projection.depth >= 0.0f &&
           IsProjectionIndexValid(projection.horizontal_index, projection.vertical_index,
                                  projection.position, horizontal_size, vertical_size, position_size);
}

inline bool ProjectCartesian(double x, double y, double z,
                             float horizontal_resolution, float vertical_resolution,
                             int horizontal_size, int vertical_size, int position_size,
                             Projection *projection)
{
    if (projection == nullptr || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(horizontal_resolution) || horizontal_resolution <= 0.0f ||
        !std::isfinite(vertical_resolution) || vertical_resolution <= 0.0f ||
        horizontal_size <= 0 || vertical_size <= 0 || position_size <= 0)
    {
        return false;
    }

    const float x_float = static_cast<float>(x);
    const float y_float = static_cast<float>(y);
    const float z_float = static_cast<float>(z);
    if (!std::isfinite(x_float) || !std::isfinite(y_float) || !std::isfinite(z_float))
    {
        return false;
    }

    constexpr float kPi = 3.141593f;
    Projection candidate;
    const double depth = std::sqrt(x * x + y * y + z * z);
    const float planar_depth = static_cast<float>(
        std::sqrt(std::pow(x_float, 2) + std::pow(y_float, 2)));
    candidate.depth = static_cast<float>(depth);
    candidate.horizontal_angle = ::atan2f(y_float, x_float);
    candidate.vertical_angle = ::atan2f(z_float, planar_depth);
    if (!std::isfinite(depth) || !std::isfinite(planar_depth) ||
        !std::isfinite(candidate.depth) || !std::isfinite(candidate.horizontal_angle) ||
        !std::isfinite(candidate.vertical_angle))
    {
        return false;
    }

    // Keep these operations in float to preserve the upstream projection's
    // pixel assignment exactly for valid inputs near bin boundaries.
    const float horizontal =
        (candidate.horizontal_angle + kPi) / horizontal_resolution;
    const float vertical =
        (candidate.vertical_angle + 0.5f * kPi) / vertical_resolution;
    if (!std::isfinite(horizontal) || !std::isfinite(vertical) ||
        horizontal < 0.0f || horizontal >= static_cast<float>(horizontal_size) ||
        vertical < 0.0f || vertical >= static_cast<float>(vertical_size))
    {
        return false;
    }

    candidate.horizontal_index = static_cast<int>(std::floor(horizontal));
    candidate.vertical_index = static_cast<int>(std::floor(vertical));
    const std::int64_t position =
        static_cast<std::int64_t>(candidate.horizontal_index) * vertical_size +
        candidate.vertical_index;
    if (!IsIndexInRange(position, static_cast<std::size_t>(position_size)))
    {
        return false;
    }
    candidate.position = static_cast<int>(position);

    if (!IsProjectionValid(candidate, horizontal_size, vertical_size, position_size))
    {
        return false;
    }
    *projection = candidate;
    return true;
}

inline bool TryCacheSlot(int cache_key, std::size_t capacity, std::size_t *slot)
{
    if (slot == nullptr || cache_key < 0 || capacity == 0)
    {
        return false;
    }
    *slot = static_cast<std::size_t>(cache_key) % capacity;
    return true;
}

inline bool TryRelativeIndex(int index, int first_index, std::size_t capacity, std::size_t *relative_index)
{
    if (relative_index == nullptr)
    {
        return false;
    }
    const std::int64_t relative = static_cast<std::int64_t>(index) - first_index;
    if (!IsIndexInRange(relative, capacity))
    {
        return false;
    }
    *relative_index = static_cast<std::size_t>(relative);
    return true;
}

inline bool IsHistoryLinkValid(int map_index, int first_map_index, std::size_t map_count,
                               int pixel_index, int element_index, std::size_t pixel_size,
                               int position_size)
{
    std::size_t relative_index = 0;
    return TryRelativeIndex(map_index, first_map_index, map_count, &relative_index) &&
           IsIndexInRange(pixel_index, static_cast<std::size_t>(position_size)) &&
           IsIndexInRange(element_index, pixel_size);
}

inline bool IsHistoryCapacityValid(int capacity, int map_capacity, int hash_capacity)
{
    const int supported_capacity = map_capacity < hash_capacity ? map_capacity : hash_capacity;
    return capacity >= 1 && capacity <= supported_capacity;
}

inline bool TryFloorIndex(double numerator, double resolution, int size, int *index)
{
    if (index == nullptr || !std::isfinite(numerator) || !std::isfinite(resolution) ||
        resolution <= 0.0 || size <= 0)
    {
        return false;
    }
    const double raw_index = numerator / resolution;
    if (!std::isfinite(raw_index) || raw_index < 0.0 || raw_index >= size)
    {
        return false;
    }
    *index = static_cast<int>(std::floor(raw_index));
    return true;
}

}  // namespace spatial_index
}  // namespace m_detector

#endif
