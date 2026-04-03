// =============================================================================
// igvc_teleop  —  AVL IGVC 2026
// =============================================================================
//
// Keyboard controls:
//   W          hold to throttle forward   (releases = coast / 0 throttle)
//   S          hold to throttle backward
//   A / D      left / right turn (differential)
//   1 2 3 4    speed mode  (25 / 50 / 75 / 100 % max velocity)
//   Q          DRIVE mode  (commands flow to motors)
//   E          NEUTRAL     (zero output, motors coast)
//   X          E-STOP toggle  (latches until pressed again)
//
// Published topics:
//   /diff_drive_controller/cmd_vel_unstamped   geometry_msgs/Twist
//   /igvc/status                               std_msgs/String
//
// Designed to work with igvc_control (ros2_control + diff_drive_controller).
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

// POSIX terminal (raw-mode keyboard input)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <csignal>

// terminal raw-mode helpers (uses /dev/tty so it works via ros2 launch)
static struct termios g_orig_termios;
static bool           g_raw_mode = false;
static int            g_tty_fd   = -1;

static void enableRawMode()
{
    g_tty_fd = open("/dev/tty", O_RDWR);
    if (g_tty_fd < 0) {
        g_tty_fd = STDIN_FILENO;
    }
    tcgetattr(g_tty_fd, &g_orig_termios);
    struct termios raw = g_orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(g_tty_fd, TCSAFLUSH, &raw);
    g_raw_mode = true;
}

static void disableRawMode()
{
    if (g_raw_mode) {
        tcsetattr(g_tty_fd, TCSAFLUSH, &g_orig_termios);
        g_raw_mode = false;
    }
    if (g_tty_fd >= 0 && g_tty_fd != STDIN_FILENO) {
        ::close(g_tty_fd);
        g_tty_fd = -1;
    }
}

// ── Constants ────────────────────────────────────────────────────────────────
static constexpr int CMD_HZ_DEFAULT = 50;

// ── Modes ────────────────────────────────────────────────────────────────────
enum class DriveMode { NEUTRAL, DRIVE };
enum class EStop     { OFF, ON };

// ── Node ─────────────────────────────────────────────────────────────────────
class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode()
    : rclcpp::Node("igvc_teleop_node")
    {
        // Parameters
        this->declare_parameter<int>("cmd_hz", CMD_HZ_DEFAULT);
        this->declare_parameter<double>("max_linear_vel",  0.5);
        this->declare_parameter<double>("max_angular_vel", 2.0);
        this->declare_parameter<std::vector<double>>("speed_levels",
            std::vector<double>{0.25, 0.50, 0.75, 1.00});
        this->declare_parameter<int>("default_speed_mode", 0);

        int cmd_hz       = this->get_parameter("cmd_hz").as_int();
        max_linear_vel_  = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        speed_levels_    = this->get_parameter("speed_levels").as_double_array();
        speed_idx_       = this->get_parameter("default_speed_mode").as_int();

        // Publishers
        pub_cmd_    = create_publisher<geometry_msgs::msg::Twist>(
            "/diff_drive_controller/cmd_vel_unstamped", 10);
        pub_status_ = create_publisher<std_msgs::msg::String>("/igvc/status", 10);

        // Enable raw keyboard input
        enableRawMode();

        // Main loop timer at cmd_hz
        timer_ = create_wall_timer(
            std::chrono::milliseconds(1000 / cmd_hz),
            std::bind(&TeleopNode::timerCb, this));

        printBanner();
    }

    ~TeleopNode()
    {
        disableRawMode();
    }

private:
    // ── State ────────────────────────────────────────────────────────────
    DriveMode mode_       = DriveMode::NEUTRAL;
    EStop     estop_      = EStop::OFF;
    int       speed_idx_  = 0;
    bool      key_w_      = false;
    bool      key_s_      = false;
    bool      key_a_      = false;
    bool      key_d_      = false;

    // Config params
    std::vector<double> speed_levels_;
    double              max_linear_vel_  = 0.5;
    double              max_angular_vel_ = 2.0;

    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     pub_status_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ── Read keyboard (non-blocking) ──────────────────────────────────────
    void pollKeyboard()
    {
        key_w_ = key_s_ = key_a_ = key_d_ = false;

        char c;
        while (::read(g_tty_fd, &c, 1) == 1)
        {
            switch (c)
            {
                case 'w': case 'W': key_w_ = true;  break;
                case 's': case 'S': key_s_ = true;  break;
                case 'a': case 'A': key_a_ = true;  break;
                case 'd': case 'D': key_d_ = true;  break;

                case '1': setSpeedMode(0); break;
                case '2': setSpeedMode(1); break;
                case '3': setSpeedMode(2); break;
                case '4': setSpeedMode(3); break;

                case 'q': case 'Q': setMode(DriveMode::DRIVE);   break;
                case 'e': case 'E': setMode(DriveMode::NEUTRAL); break;

                case 'x': case 'X':
                    toggleEstop();
                    break;

                case 3:  // Ctrl+C
                    disableRawMode();
                    rclcpp::shutdown();
                    return;

                default: break;
            }
        }
    }

    // ── Mode setters ──────────────────────────────────────────────────────
    void setMode(DriveMode m)
    {
        if (mode_ == m) return;
        mode_ = m;
        publishStatus();
        printState();
    }

    void setSpeedMode(int idx)
    {
        if (speed_idx_ == idx) return;
        speed_idx_ = idx;
        publishStatus();
        printState();
    }

    void toggleEstop()
    {
        estop_ = (estop_ == EStop::OFF) ? EStop::ON : EStop::OFF;
        publishStatus();
        printState();
    }

    // ── Compute and publish Twist ─────────────────────────────────────────
    void timerCb()
    {
        pollKeyboard();

        geometry_msgs::msg::Twist twist;

        if (estop_ == EStop::ON || mode_ == DriveMode::NEUTRAL) {
            // Zero twist — diff_drive_controller will stop the motors
            pub_cmd_->publish(twist);
            return;
        }

        // DRIVE mode: compute linear + angular from WASD
        double scale = speed_levels_[speed_idx_];

        // Forward / reverse
        if (key_w_) twist.linear.x =  scale * max_linear_vel_;
        if (key_s_) twist.linear.x = -scale * max_linear_vel_;

        // Turn left / right
        if (key_a_) twist.angular.z =  scale * max_angular_vel_;
        if (key_d_) twist.angular.z = -scale * max_angular_vel_;

        pub_cmd_->publish(twist);
    }

    // ── Helpers ───────────────────────────────────────────────────────────
    void publishStatus()
    {
        std_msgs::msg::String msg;
        msg.data = buildStatusString();
        pub_status_->publish(msg);
    }

    std::string buildStatusString()
    {
        char buf[128];
        snprintf(buf, sizeof(buf),
            "MODE=%s  SPEED=%d(%d%%)  ESTOP=%s",
            mode_ == DriveMode::DRIVE ? "DRIVE" : "NEUTRAL",
            speed_idx_ + 1,
            static_cast<int>(speed_levels_[speed_idx_] * 100),
            estop_ == EStop::ON ? "ON" : "OFF");
        return std::string(buf);
    }

    void printState()
    {
        printf("\r\033[K[%s]\n", buildStatusString().c_str());
        fflush(stdout);
    }

    void printBanner()
    {
        printf("\n");
        printf("╔══════════════════════════════════════╗\n");
        printf("║      AVL IGVC 2026  —  Teleop        ║\n");
        printf("╠══════════════════════════════════════╣\n");
        printf("║  W/S       Forward / Reverse         ║\n");
        printf("║  A/D       Turn left / right         ║\n");
        printf("║  1 2 3 4   Speed 25/50/75/100%%       ║\n");
        printf("║  Q         DRIVE mode                ║\n");
        printf("║  E         NEUTRAL mode              ║\n");
        printf("║  X         E-STOP toggle             ║\n");
        printf("║  Ctrl+C    Quit                      ║\n");
        printf("╚══════════════════════════════════════╝\n");
        printf("\n");
        printState();
    }
};

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    signal(SIGINT,  [](int){ disableRawMode(); rclcpp::shutdown(); });
    signal(SIGTERM, [](int){ disableRawMode(); rclcpp::shutdown(); });

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
