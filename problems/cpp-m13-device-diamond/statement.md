# Device Diamond

Your hardware abstraction layer has a `Device` base (id + boot counter), and two capability classes: `Powered` (power rail) and `Networked` (Ethernet). A `SmartLidar` is both:
```
          Device
         /      \
    Powered    Networked
         \      /
        SmartLidar
```
A physical lidar is **one** device: it must have exactly **one** `Device` subobject, so it's constructed once, has one id, and `powerOn()` and `connect()` both increment the **same** boot counter. The starter declares the classes, but the inheritance isn't right yet. Fix the hierarchy and implement:

| Class | Member | Behaviour |
|---|---|---|
| `Device` | `explicit Device(int id)` / `virtual ~Device()` | Log `"Device(<id>)"` / `"~Device(<id>)"`. |
| | `id()`, `bootCount()` | Accessors. |
| | `void boot()` (**protected**) | `++bootCount`. |
| `Powered` | `Powered(int id, double watts)` / dtor | Log `"Powered(<id>)"` / `"~Powered(<id>)"`. |
| | `void powerOn()` | Sets `isOn()`, calls `boot()`. Calling it again while already on does nothing. |
| | `watts()`, `isOn()` | Accessors. |
| `Networked` | `Networked(int id, std::string ip)` / dtor | Log `"Networked(<id>)"` / `"~Networked(<id>)"`. |
| | `void connect()` | Calls `boot()` every time. |
| | `ip()` | Accessor. |
| `SmartLidar : Powered, Networked` | `SmartLidar(int id, double watts, std::string ip)` / dtor | Log `"SmartLidar(<id>)"` / `"~SmartLidar(<id>)"`. |
| | `int deviceId() const`, `int totalBoots() const` | The id and boot count of its (single) `Device`. |

Expected log for `{ SmartLidar s(7, 12.0, "10.0.0.7"); }`:
```
Device(7) Powered(7) Networked(7) SmartLidar(7) ~SmartLidar(7) ~Networked(7) ~Powered(7) ~Device(7)
```
A standalone `Powered p(3, 5.0)` must still work: `Device(3) Powered(3) ~Powered(3) ~Device(3)`.

### Constraints
- Keep the starter's public interface and the base-class order `Powered, Networked`.
- Deleting a `SmartLidar` through a `Powered*` or `Device*` must run every destructor.

### Think about
- Who constructs a virtual base, and what happens to the `Device(id)` calls written in `Powered`'s and `Networked`'s initializer lists?
- What does virtual inheritance cost (object layout, vbase offset, casts)? Why can't you `static_cast` from `Device*` down to `SmartLidar*`?
- Without `virtual`, why is `s.id()` a compile error, and how would you disambiguate it?
- Could you avoid the diamond with composition, e.g. `SmartLidar` *has* a `PowerRail` and a `NetLink`? Which design would you choose in production?
