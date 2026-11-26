// Egyszeru Follow the Gap (javitott)
// /scan (lidar) -> /cmd_vel (sebesseg)

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

constexpr double kPi = 3.14159265358979323846;
constexpr double kFov = 1.9;              // ~+/- 109 fok (hogy a kanyar nyilasat lassuk)
constexpr double kMaxRange = 3.0;         // ennel messzebbit levagunk [m]
constexpr double kBubbleFov = 1.05;       // buborekhoz csak az elso +/-60 fokot nezzuk [rad]
constexpr double kMaxBubbleAngle = 0.6;   // buborek felszoge max ~35 fok [rad]

class Megoldas : public rclcpp::Node
{
public:
  Megoldas() : Node("megoldas")
  {
    declare_parameter("max_speed", 0.3);       // [m/s]
    declare_parameter("min_speed", 0.12);      // [m/s]
    declare_parameter("steering_gain", 0.8);   // angular.z alaperosites
    declare_parameter("bubble_radius", 0.30);  // robot fele szelessege + raghagyas [m]
    declare_parameter("free_threshold", 0.6);  // ennel kozelebbi sugar nem szabad [m]
    declare_parameter("stop_distance", 0.25);  // ennel kozelebb elore megall [m]

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&Megoldas::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Az algoritmus elindult");
  }

  ~Megoldas() override
  {
    // leallitaskor nulla sebesseg, hogy a robot ne menjen tovabb
    if (cmd_pub_) {
      drive(0.0, 0.0);
    }
  }

private:
  double param(const char * name) { return get_parameter(name).as_double(); }

  void drive(double v, double w)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const double inc = std::abs(msg->angle_increment);
    if (msg->ranges.empty() || inc <= 0.0) {
      return;
    }

    // 1. Sugarak (szog, tavolsag), csak az elso +/-kFov, szog szerint rendezve
    std::vector<std::pair<double, double>> pts;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double a =
        std::remainder(msg->angle_min + i * msg->angle_increment, 2.0 * kPi);
      if (std::abs(a) > kFov) {
        continue;
      }
      double r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
        r = kMaxRange;  // ervenytelen / nincs visszaverodes = szabad
      }
      pts.emplace_back(a, std::min(r, kMaxRange));
    }
    if (pts.size() < 3) {
      drive(0.0, 0.0);
      return;
    }
    std::sort(pts.begin(), pts.end());
    const size_t n = pts.size();

    // 2. Biztonsag: tavolsag elore (+/-15 fok)
    double front = kMaxRange;
    for (const auto & p : pts) {
      if (std::abs(p.first) < deg15) {
        front = std::min(front, p.second);
      }
    }
    if (front < param("stop_distance")) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "STOP: akadaly elore");
      drive(0.0, 0.0);
      return;
    }

    // 3. Buborek: csak az elso +/-60 fok legkozelebbi pontja korul blokkolunk
    std::vector<double> r(n);
    for (size_t k = 0; k < n; ++k) {
      r[k] = pts[k].second;
    }
    size_t ci = n / 2;
    double closest = kMaxRange;
    for (size_t k = 0; k < n; ++k) {
      if (std::abs(pts[k].first) < kBubbleFov && pts[k].second < closest) {
        closest = pts[k].second;
        ci = k;
      }
    }
    closest = std::max(closest, 0.05);
    if (closest < kMaxRange) {
      const double half_angle = std::min(
        kMaxBubbleAngle, std::asin(std::min(1.0, param("bubble_radius") / closest)));
      const long half_rays = static_cast<long>(std::ceil(half_angle / inc));
      const long lo = std::max<long>(0, static_cast<long>(ci) - half_rays);
      const long hi = std::min<long>(static_cast<long>(n) - 1, static_cast<long>(ci) + half_rays);
      for (long k = lo; k <= hi; ++k) {
        r[k] = 0.0;
      }
    }

    // 4. Rés keresés: a LEGMELYEBB rés nyer (nem a legszélesebb).
    //    Igy a kanyar nyilasa (r ~ 3 m) mar messzirol nyer a fal (r ~ 1.5 m) ellen.
    const double free_th = param("free_threshold");
    const double min_width = 0.15;  // [rad] ennel keskenyebb rest nem veszunk figyelembe
    double best_score = -1e9;
    double angle = 0.0;
    bool found = false;
    for (size_t k = 0; k < n;) {
      if (r[k] <= free_th) {
        ++k;
        continue;
      }
      const size_t s = k;
      double max_r = 0.0;
      while (k < n && r[k] > free_th) {
        max_r = std::max(max_r, r[k]);
        ++k;
      }
      const size_t e = k - 1;
      if (pts[e].first - pts[s].first < min_width) {
        continue;
      }
      // a rés legmélyebb (max 90%-a feletti) sugarainak közepe
      size_t lo2 = s, hi2 = e;
      while (r[lo2] < 0.9 * max_r) {++lo2;}
      while (r[hi2] < 0.9 * max_r) {--hi2;}
      const double ang = 0.5 * (pts[lo2].first + pts[hi2].first);
      // melyseg - kis büntetes a nagy szogert (egyenes folyosón az egyenes nyer)
      const double score = max_r - 0.4 * std::abs(ang);
      if (score > best_score) {
        best_score = score;
        angle = ang;
        found = true;
      }
    }
    if (!found) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "STOP: nincs szabad rés");
      drive(0.0, 0.0);
      return;
    }

    // 5. Celszog = a kivalasztott rés legmelyebb reszenek kozepe (pozitiv = balra)
    // szures: csokkenti az egyenes menti cikazast
    angle_f_ = 0.5 * angle_f_ + 0.5 * angle;
    angle = angle_f_;

    // holtsav egyenesben
    if (std::abs(angle) < 0.05) {
      angle = 0.0;
    }

    // 6. Kormanyzas (nemlinearis: kis szognel puha, nagynal eros) + sebesseg
    const double w = std::clamp(
      param("steering_gain") * angle * (1.0 + 1.5 * std::abs(angle)), -1.5, 1.5);
    const double turn_factor = 1.0 - std::min(1.0, std::abs(angle) / kFov);
    const double dist_factor = std::clamp(front / 1.5, 0.0, 1.0);  // 1.5 m-nel kezd lassulni
    const double v = param("min_speed") +
      (param("max_speed") - param("min_speed")) * std::min(turn_factor, dist_factor);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "angle=%.2f front=%.2f score=%.2f v=%.2f w=%.2f",
      angle, front, best_score, v, w);

    drive(v, w);
  }

  static constexpr double deg15 = 15.0 * kPi / 180.0;

  double angle_f_ = 0.0;  // szurt celszog

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Megoldas>());
  rclcpp::shutdown();
  return 0;
}