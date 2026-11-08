# Deep-Copyable Keyframe

A loop-closure module **copies** keyframes into a candidate database, while the main map **moves** them between containers. Each keyframe owns a lidar cloud and a **polymorphic** place-recognition descriptor (given):
```cpp
struct Descriptor {                         // abstract; live/clones are counters for the tests
  virtual std::unique_ptr<Descriptor> clone() const = 0;    // ++clones
  virtual std::string type() const = 0;
  virtual double score(const Descriptor& other) const = 0;
  virtual ~Descriptor();                                     // --live
};
struct ScanContext : Descriptor { std::vector<float> rings; ... };
struct M2DP        : Descriptor { std::array<float, 192> sig; ... };
```
Implement `class Keyframe`:

| Member | Semantics |
|---|---|
| `Keyframe(int id, std::vector<Point3> cloud, std::unique_ptr<Descriptor> desc)` | Takes ownership. `desc` may be null. |
| `Keyframe(const Keyframe&)`, `operator=(const Keyframe&)` | **Deep** copy: the copy gets its own descriptor via `clone()` (exactly one `clone()` per non-null descriptor), with the **same dynamic type**. Self-assignment is safe. |
| move ctor / move `=` | **No** cloning and **no** cloud copy: the cloud's buffer and the descriptor are transferred. Must be `noexcept`. |
| `int id() const`, `const std::vector<Point3>& cloud() const`, `const Descriptor* descriptor() const` | Accessors. |
| `void setDescriptor(std::unique_ptr<Descriptor> d)` | Replaces the descriptor (the old one is destroyed). |

And a free function:

| Function | Semantics |
|---|---|
| `void swap(Keyframe& a, Keyframe& b) noexcept` | Swap everything, with no clone and no allocation. |

### Constraints
- `std::vector<Keyframe>` growth must **not** clone or copy clouds (the tests count `Descriptor::clones`).
- No leaks: tests check `Descriptor::live`.

### Think about
- Which special members does the compiler generate for a class holding a `unique_ptr`? Which ones did you have to write, and why does writing one make others disappear?
- Why is `clone()` virtual instead of copying via `*desc_` (slicing)?
- When would you choose `std::shared_ptr<const Descriptor>` instead, and get the **rule of zero** back? What changes semantically?
- What exception guarantee does your copy assignment give?
