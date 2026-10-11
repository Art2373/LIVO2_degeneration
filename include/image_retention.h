#pragma once
#include <cstddef>
#include <stdexcept>

namespace frontend_image_queue
{
// Input order remains chronological. Preserve the oldest still-pending frame
// while synchronization waits for LiDAR/IMU/reordering support; retain the newest
// capacity-1 frames behind it. This is admission/retention only: the caller must
// still reject late input and apply the existing time/coverage/consumption rules.
// Return size when no overflow exists. A capacity of one retains only the head.
inline std::size_t overflow_drop_index(std::size_t size, std::size_t capacity)
{
  if (capacity == 0) throw std::invalid_argument("image capacity must be positive");
  return size > capacity ? 1 : size;
}

// Keep image payload and sensor identity paired; notify before removal so the
// caller can record the exact discarded identity without changing its timestamp.
template <class Images, class Stamps, class OnDrop>
std::size_t trim_keep_head(Images &images, Stamps &stamps, std::size_t capacity,
                          OnDrop on_drop)
{
  if (images.size() != stamps.size())
    throw std::invalid_argument("image/stamp queue size mismatch");
  overflow_drop_index(images.size(), capacity);
  std::size_t removed = 0;
  while (images.size() > capacity)
  {
    const std::size_t index = overflow_drop_index(images.size(), capacity);
    on_drop(index, stamps[index]);
    images.erase(images.begin() + index);
    stamps.erase(stamps.begin() + index);
    ++removed;
  }
  return removed;
}
}
