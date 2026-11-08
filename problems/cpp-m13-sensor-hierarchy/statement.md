# Sensor Hierarchy

Your robot's driver layer has a common `Sensor` base with `Lidar` and `Camera` derived classes. Every constructor and destructor appends to a global `g_log` so that bring-up and shutdown order can be audited. A helper `Tracer` member logs `"+name"` on construction and `"-name"` on destruction (given).

| Class | Member | Behaviour |
|---|---|---|
| `Sensor` | `Sensor(std::string name, int rate_hz)` (**protected**) | Logs `"Sensor(<name>)"`. Only derived classes may construct a `Sensor`. |
| | `virtual ~Sensor()` | Logs `"~Sensor(<name>)"`. |
| | `name()`, `rateHz()`, `frames()` | Accessors (`frames()` starts at 0). |
| | `std::string describe() const` | `"<name>@<rate>Hz"`, e.g. `"vlp16@10Hz"`. |
| | `void tick()` (**protected**) | `++frames`. Derived classes call it once per produced frame. |
| `Lidar : public Sensor` | `Lidar(std::string name, int rings, int rate_hz = 10)` | Throws `std::invalid_argument` if `rings < 1` (**before** logging). Otherwise logs `"Lidar(<name>)"`. Has a member `Tracer motor_{"motor"}`. |
| | `~Lidar()` | Logs `"~Lidar(<name>)"`. |
| | `void spin(int revolutions)` | One frame per revolution (ignores negative values). |
| | `std::string describe() const` | The base description + `" rings=<rings>"`, e.g. `"vlp16@10Hz rings=16"`. |
| `Camera : public Sensor` | `Camera(std::string name, int width, int height, int rate_hz = 30)` | Logs `"Camera(<name>)"`. Members `Tracer lens_{"lens"}` and `Tracer shutter_{"shutter"}`, declared in that order. |
| | `~Camera()` | Logs `"~Camera(<name>)"`. |
| | `void capture()` | One frame. |
| | `std::string describe() const` | The base description + `" <w>x<h>"`, e.g. `"cam0@30Hz 640x480"`. |

For example, `{ Lidar l("vlp16", 16); }` must log exactly `Sensor(vlp16) +motor Lidar(vlp16) ~Lidar(vlp16) -motor ~Sensor(vlp16)`.

### Constraints
- Don't change the declared member order or access specifiers in the starter.
- `describe()` is deliberately **not** virtual (polymorphism is M14). Think about what `const Sensor& s = lidar; s.describe()` returns.

### Think about
- Why is `Sensor`'s constructor protected rather than public? How else could you prevent instantiating the base?
- If `Lidar`'s constructor throws, which destructors run? Which don't, and why?
- Why does `Sensor` need a **virtual** destructor if anyone ever owns a `Lidar` through `std::unique_ptr<Sensor>`?
- Inheritance vs composition: should `Lidar` *be* a `Sensor` or *have* a motor? (It does both here.)
