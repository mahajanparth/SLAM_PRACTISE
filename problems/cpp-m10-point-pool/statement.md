# Point Pool Allocator with Stable Addresses

A mapping backend creates and deletes millions of map points, and other structures (keyframe observations, a voxel hash) keep **raw pointers** to them. `std::vector<Point3>` won't work, because growth relocates every point and invalidates those pointers, and per-point `new` is slow and fragments the heap. Build a block-based pool. `struct Point3 { double x, y, z; };` is defined.

| Function / member | Behaviour |
|---|---|
| `std::uintptr_t alignUp(std::uintptr_t addr, std::size_t align)` | Smallest multiple of `align` that is `≥ addr`. `align` is a power of two. |
| `explicit PointPool(std::size_t block_size)` | Each block holds `block_size` points (`0` is treated as `1`). No memory is allocated until the first `allocate`. |
| `~PointPool()` | Frees **all** blocks. Copying is disabled. |
| `Point3* allocate(double x, double y, double z)` | Returns a slot initialised to `{x,y,z}`. Reuse the **most recently freed** slot if there is one (LIFO). Otherwise take the next unused slot, adding a new block only when every block is full. **Previously returned pointers never move.** |
| `void deallocate(Point3* p)` | Returns `p` to the pool. `nullptr` is a no-op. (`p` is otherwise always a live pointer from this pool.) |
| `std::size_t size() const` | Number of live (allocated, not freed) points. |
| `std::size_t capacity() const` | Total slots across all blocks. |
| `std::size_t blockCount() const` | Number of blocks allocated. |

### Constraints
- `allocate` and `deallocate` are amortised **O(1)**.
- Up to 2·10⁵ points per test. No leaks (ASan leak detection is on).
- Returned pointers must be suitably aligned for `Point3`.

### Think about
- Why does `std::deque` also give stable addresses on `push_back`, and what does it not give you that the pool does?
- How could the free list live **inside** the freed slots themselves (intrusive free list) with zero extra memory?
- What would you need to make this an STL-compatible allocator (`std::allocator_traits`)?
- How do you guarantee 32-byte alignment for AVX if `Point3` were `alignas(32)`? (`alignUp`, aligned `operator new`.)
- Is this thread-safe? What's the cheapest way to make it so (per-thread pools)?
