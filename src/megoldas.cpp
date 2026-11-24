// Egyszeru Follow the Gap
// /scan (lidar) -> /cmd_vel (sebesseg)

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

constexpr double kPi = 3.14159265358979323846;
constexpr double kFov = kPi / 2.0;   // +/- 90 fok
constexpr double kMaxRange = 3.0;    // ennel messzebbit levagunk [m]

class Megoldas : public rclcpp::Node
{
public:
  Megoldas() : Node("megoldas")
  {
    declare_parameter("max_speed", 0.4);       // [m/s]
    declare_parameter("min_speed", 0.15);      // [m/s]
    declare_parameter("steering_gain", 1.5);   // angular.z = gain * szog
    declare_parameter("bubble_radius", 0.35);  // robot fele szelessege + ráhagyas [m]
    declare_parameter("free_threshold", 0.8);  // ennel kozelebbi sugar nem szabad [m]
    declare_parameter("stop_distance", 0.25);  // ennel kozelebb elore megall [m]

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&Megoldas::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Az algoritmus elindult");
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

    // 1. Sugarak (szog, tavolsag), csak az elso +/-90 fok, szog szerint rendezve
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

    // 3. Buborek: a legkozelebbi pont koruli sugarakat blokkoltnak vesszuk
    std::vector<double> r(n);
    for (size_t k = 0; k < n; ++k) {
      r[k] = pts[k].second;
    }
    const size_t ci = std::min_element(
      pts.begin(), pts.end(),
      [](const auto & a, const auto & b) {return a.second < b.second;}) - pts.begin();
    const double closest = std::max(pts[ci].second, 0.05);
    if (closest < kMaxRange) {
      const double half_angle = std::asin(std::min(1.0, param("bubble_radius") / closest));
      const long half_rays = static_cast<long>(std::ceil(half_angle / inc));
      const long lo = std::max<long>(0, static_cast<long>(ci) - half_rays);
      const long hi = std::min<long>(static_cast<long>(n) - 1, static_cast<long>(ci) + half_rays);
      for (long k = lo; k <= hi; ++k) {
        r[k] = 0.0;
      }
    }

    // 4. Legjobb rés: az a szabad szakasz, ahol a tavolsagok osszege a legnagyobb
    //    (szeles es melyseg is szamit)
    const double free_th = param("free_threshold");
    double best_sum = 0.0;
    size_t best_s = 0, best_e = 0;
    for (size_t k = 0; k < n;) {
      if (r[k] <= free_th) {
        ++k;
        continue;
      }
      const size_t s = k;
      double sum = 0.0;
      while (k < n && r[k] > free_th) {
        sum += r[k];
        ++k;
      }
      if (sum > best_sum) {
        best_sum = sum;
        best_s = s;
        best_e = k - 1;
      }
    }
    if (best_sum == 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "STOP: nincs szabad rés");
      drive(0.0, 0.0);
      return;
    }

    // 5. Celszog = a rés kozepe (pozitiv = balra)
    const double angle = 0.5 * (pts[best_s].first + pts[best_e].first);

    // 6. Kormanyzas + sebesseg (kanyarban es akadaly kozeleben lassabb)
    const double w = std::clamp(param("steering_gain") * angle, -1.5, 1.5);
    const double turn_factor = 1.0 - std::min(1.0, std::abs(angle) / kFov);
    const double dist_factor = std::clamp(front / 1.0, 0.0, 1.0);  // 1 m-nel kezd lassulni
    const double v = param("min_speed") +
      (param("max_speed") - param("min_speed")) * std::min(turn_factor, dist_factor);

    drive(v, w);
  }

  static constexpr double deg15 = 15.0 * kPi / 180.0;

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