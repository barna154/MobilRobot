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
    declare_parameter("max_speed", 0.4);       // [m/s]
    declare_parameter("min_speed", 0.1);       // [m/s]
    declare_parameter("steering_gain", 1.5);   // a rés iranyaba kormanyzas erositese
    declare_parameter("bubble_radius", 0.4);   // robot fele szelessege + raghagyas [m]
    declare_parameter("free_threshold", 0.6);  // ennel kozelebbi sugar nem szabad [m]
    declare_parameter("stop_distance", 0.2);   // ennel kozelebb elore megall/hatrál [m]
    declare_parameter("center_gain", 0.8);     // folyoson kozepre huzas erositese (0 = ki)
    declare_parameter("reverse_speed", -0.12); // hatrálas sebessege elakadaskor (0 = ki) [m/s]

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

  // Elakadas: ha van hely hatul, hatral ~1.2 s-ig a nyitottabb oldal fele fordulva.
  // Ha nem latunk hatrafele (rear_n == 0) vagy kicsi a hely, csak megall.
  void tryRecover(
    const std::vector<std::pair<double, double>> & pts, double rear, size_t rear_n)
  {
    if (param("reverse_speed") >= 0.0 || rear_n == 0 || rear < 0.3) {
      drive(0.0, 0.0);
      return;
    }
    double sum_l = 0.0, sum_r = 0.0;
    for (const auto & p : pts) {
      (p.first > 0.0 ? sum_l : sum_r) += p.second;
    }
    recover_dir_ = (sum_l >= sum_r) ? 1.0 : -1.0;  // az orr a nyitottabb oldal fele forduljon
    recovering_ = true;
    recover_until_ = now() + rclcpp::Duration::from_seconds(1.2);
    drive(param("reverse_speed"), recover_dir_);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const double inc = std::abs(msg->angle_increment);
    if (msg->ranges.empty() || inc <= 0.0) {
      return;
    }

    // Hatso szabad hely a nyers merésbol (|szog| > ~143 fok)
    double rear = kMaxRange;
    size_t rear_n = 0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double a =
        std::remainder(msg->angle_min + i * msg->angle_increment, 2.0 * kPi);
      if (std::abs(a) > 2.5) {
        ++rear_n;
        const double rr = msg->ranges[i];
        if (std::isfinite(rr) && rr >= msg->range_min) {
          rear = std::min(rear, rr);
        }
      }
    }

    // Folyamatban levo hatralas
    if (recovering_) {
      if (now() < recover_until_ && rear > 0.3 && rear_n > 0 && param("reverse_speed") < 0.0) {
        drive(param("reverse_speed"), recover_dir_);
        return;
      }
      recovering_ = false;
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
      if (!std::isfinite(r) || r > msg->range_max) {
        r = kMaxRange;         // nincs visszaverodes (inf) = szabad
      } else if (r < msg->range_min) {
        r = msg->range_min;    // tul kozeli / ervenytelen = FOGLALT (nem szabad!)
      }
      pts.emplace_back(a, std::min(r, kMaxRange));
    }
    if (pts.size() < 3) {
      drive(0.0, 0.0);
      return;
    }
    std::sort(pts.begin(), pts.end());
    const size_t n = pts.size();

    // Oldalso tavolsag (kb. 70-100 fok): a falak merolegesen mert tavolsaga, max 1 m
    double lat_l = 1.0, lat_r = 1.0;
    for (const auto & p : pts) {
      const double aa = std::abs(p.first);
      if (aa >= 1.2 && aa <= 1.8) {
        const double d = p.second * std::sin(aa);
        (p.first > 0.0 ? lat_l : lat_r) = std::min(p.first > 0.0 ? lat_l : lat_r, d);
      }
    }

    // 2. Biztonsag: tavolsag elore (+/-15 fok)
    double front = kMaxRange;
    for (const auto & p : pts) {
      if (std::abs(p.first) < deg15) {
        front = std::min(front, p.second);
      }
    }
    if (front < param("stop_distance")) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "STOP: akadaly elore");
      tryRecover(pts, rear, rear_n);
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

    // 3b. Min-szuro (+/-1 sugar): kiszuri az egyedi tavoli sugarakat (zaj, repedes a falon),
    //     igy egy hamis "melyseg" nem csalja be a robotot
    std::vector<double> rf(n);
    for (size_t k = 0; k < n; ++k) {
      const size_t a = k >= 1 ? k - 1 : 0;
      const size_t b = std::min(n - 1, k + 1);
      double m = r[k];
      for (size_t j = a; j <= b; ++j) {
        m = std::min(m, r[j]);
      }
      rf[k] = m;
    }

    // 4. Rés keresés: a legmélyebb rés nyer, de csak ha a robot belefér.
    //    score = melyseg * (belefer-e) - kis büntetes a nagy szogert
    const double free_th = param("free_threshold");
    const double bubble = param("bubble_radius");
    double best_score = -1e9;
    double angle = 0.0;
    bool found = false;
    for (size_t k = 0; k < n;) {
      if (rf[k] <= free_th) {
        ++k;
        continue;
      }
      const size_t s = k;
      size_t pk = k;
      while (k < n && rf[k] > free_th) {
        if (rf[k] > rf[pk]) {
          pk = k;
        }
        ++k;
      }
      const size_t e = k - 1;
      const double max_r = rf[pk];
      // a csucs koruli, ~max melysegu (>=90%) osszefuggo szakasz
      size_t lo2 = pk, hi2 = pk;
      while (lo2 > s && rf[lo2 - 1] >= 0.9 * max_r) {--lo2;}
      while (hi2 < e && rf[hi2 + 1] >= 0.9 * max_r) {++hi2;}
      const double ang = 0.5 * (pts[lo2].first + pts[hi2].first);
      const double width = pts[hi2].first - pts[lo2].first;
      const double need = 2.0 * std::asin(std::min(1.0, 0.5 * bubble / max_r));  // ennyi szog kell
      const double fit = std::min(1.0, width / need);
      const double score = max_r * fit - 0.4 * std::abs(ang);
      if (score > best_score) {
        best_score = score;
        angle = ang;
        found = true;
      }
    }
    if (!found) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "STOP: nincs szabad rés");
      tryRecover(pts, rear, rear_n);
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
    const double w_gap = param("steering_gain") * angle * (1.0 + 1.5 * std::abs(angle));
    // kozepre huzas: ha az egyik fal kozelebb van, forduljon el tole (kanyarban kikapcsol)
    const double center_w =
      param("center_gain") * (lat_l - lat_r) * (1.0 - std::min(1.0, std::abs(angle) / 0.5));
    const double w = std::clamp(w_gap + center_w, -1.5, 1.5);
    const double turn_factor = 1.0 - std::min(1.0, std::abs(angle) / kFov);
    const double dist_factor = std::clamp(front / 1.5, 0.0, 1.0);  // 1.5 m-nel kezd lassulni
    const double v = param("min_speed") +
      (param("max_speed") - param("min_speed")) * std::min(turn_factor, dist_factor);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "angle=%.2f front=%.2f score=%.2f latL=%.2f latR=%.2f v=%.2f w=%.2f",
      angle, front, best_score, lat_l, lat_r, v, w);

    drive(v, w);
  }

  static constexpr double deg15 = 15.0 * kPi / 180.0;

  double angle_f_ = 0.0;  // szurt celszog
  bool recovering_ = false;
  double recover_dir_ = 1.0;
  rclcpp::Time recover_until_{0, 0, RCL_ROS_TIME};

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
