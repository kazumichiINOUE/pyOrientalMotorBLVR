#include <iostream>
#include <vector>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <iomanip>  // std::setprecision
#include <tuple>
#include <random>
#include <lua.hpp>
#include <unistd.h>
#include <iostream>
#include <ncurses.h>
#include <thread>
#include <SDL2/SDL.h>

namespace fs = std::filesystem;

struct Point {
    double x;
    double y;

    Point() {};
    Point(double x, double y) {
        this->x = x;
        this->y = y;
    }
};

struct Pose {
    long long ts;
    double x, y, a;
    Pose() {};
    Pose(long long ts, double x, double y, double a) {
        this->ts = ts;
        this->x = x;
        this->y = y;
        this->a = a;
    }
};

struct Node {
    Pose p;
    std::vector<Point> pt;

    Node() {};
    Node(Pose p, std::vector<Point> pt) {
        this->p = p;
        this->pt = pt;
    }
};

inline std::tuple<int, int> xy2index(double xd, double yd, double csize, int ox, int oy) {
    int ix = static_cast<int>( xd / csize) + ox;
    int iy = static_cast<int>(-yd / csize) + oy;
    return std::make_tuple(ix, iy);
}
std::vector<std::vector<int>> update_map_all_node(std::vector<Node> &node, 
                            std::vector<std::vector<int>> &gmap, int width, int height, int originX, int originY, double csize) {
    for (auto nd: node) { 
        double cs = cos(nd.p.a);
        double sn = sin(nd.p.a);
        for (size_t i = 0; i < nd.pt.size(); i++) {
            double xd = nd.pt[i].x * cs - nd.pt[i].y * sn + nd.p.x;
            double yd = nd.pt[i].x * sn + nd.pt[i].y * cs + nd.p.y;
            int ix = static_cast<int>( xd / csize) + originX;
            int iy = static_cast<int>(-yd / csize) + originY;
            if (0 <= ix && ix < width && 0 <= iy && iy < height) {
                gmap[iy][ix] = 1;
            }
        }
    }
    return gmap;
}

std::vector<std::vector<int>> update_map(std::vector<std::vector<int>> &gmap, 
                                         std::vector<Point> &pt, 
                                         double current_x, double current_y, double current_a, 
                                         int width, int height, int ox, int oy, double csize) {
    double cs = cos(current_a);
    double sn = sin(current_a);
    for (size_t i = 0; i < pt.size(); i++) {
        double xd = pt[i].x * cs - pt[i].y * sn + current_x;
        double yd = pt[i].x * sn + pt[i].y * cs + current_y;
        //auto [ix, iy] = xy2index(xd, yd, csize, ox, oy);
        int ix = static_cast<int>( xd / csize) + ox;
        int iy = static_cast<int>(-yd / csize) + oy;
        if (0 <= ix && ix < width && 0 <= iy && iy < height) {
            if (gmap[iy][ix] != 1) {
                gmap[iy][ix] = 1;
            }
        }
    }

    return gmap;
}

std::vector<std::vector<int>> update_trajectory(std::vector<std::vector<int>> &gmap, 
                                                std::vector<Pose> &poses, 
                                                int width, int height, int ox, int oy, double csize) {
    int ix, iy;
    for (Pose p : poses) {
        auto [ix, iy] = xy2index(p.x, p.y, csize, ox, oy);
        if (0 <= ix && ix < width && 0 <= iy && iy < height) {
            gmap[iy][ix] = 1;
        }
    }

    return gmap;
}

cv::Mat gmap_show(std::vector<std::vector<int>> &gmap, double width, double height) {
    cv::Mat img = cv::Mat(cv::Size(width, height), CV_8UC3, cv::Scalar(0xed, 0xe7, 0xe6));
    for (int j = 0; j < gmap.size(); j++) {
        for (int i = 0; i < gmap[0].size(); i++) {
            if (gmap[j][i] == 1) {
                img.at<cv::Vec3b>(j, i)[0] = 0;
                img.at<cv::Vec3b>(j, i)[1] = 0;
                img.at<cv::Vec3b>(j, i)[2] = 0;
            }
        }
    }
    //cv::imshow("slam", img);
    //int key = cv::waitKey(10);
    return img;
}

double normalize_th(double ra) {
    while(1) {
        if (ra > M_PI) {
            ra -= 2*M_PI;
        } else if (ra < -M_PI) {
            ra += 2*M_PI;
        } else {
            break;
        }
    }
    return ra;
}

int match_count(std::vector<std::vector<int>> &gmap,
                std::vector<Point> &pt,
                double cx, double cy, double ca,
                double csize, int originX, int originY, int width, int height) {
    int eval = 0;
    double cs = cos(ca);
    double sn = sin(ca);
    for (int ind = 0; ind < pt.size(); ind += 1) {
        double xd = pt[ind].x * cs - pt[ind].y * sn + cx;
        double yd = pt[ind].x * sn + pt[ind].y * cs + cy;
        auto [ix, iy] = xy2index(xd, yd, csize, originX, originY);
        if (0 <= ix && ix < width && 0 <= iy && iy < height) {
            if (gmap[iy][ix] == 1) {
                eval += 1;
            }
        }
    }
    return eval;
}

// 乱数生成エンジンの準備
std::random_device rd; // ハードウェア乱数生成器
std::mt19937 gen(rd()); // メルセンヌ・ツイスタ法の乱数生成器
// -1から1の間の一様分布を定義
std::uniform_real_distribution<> dis(-1.0, 1.0);

std::tuple<double, double, double, double> optimize_de(
    std::vector<std::vector<int>> &gmap, std::vector<Point> &pt,
    double current_x, double current_y, double current_a,
    int originX, int originY, 
    double csize, double dth,
    int width, int height, 
    double Wxy, double Wa, 
    int population_size, int generations, double F, double CR) {

    // Initialize population
    std::vector<std::tuple<double, double, double>> population(population_size);
    for (int i = 0; i < population_size; i++) {
        double x = current_x + dis(gen) * Wxy;
        double y = current_y + dis(gen) * Wxy;
        double a = current_a + dis(gen) * Wa;
        population[i] = std::make_tuple(x, y, a);
    }

    double best_eval = -std::numeric_limits<double>::infinity();
    double best_x = current_x, best_y = current_y, best_a = current_a;

    std::uniform_int_distribution<> dist(0, population_size - 1);
    for (int generation = 0; generation < generations; generation++) {
        for (int i = 0; i < population_size; i++) {
            // Mutation: select three distinct individuals (r1, r2, r3)
            int r1, r2, r3;
            do r1 = dist(gen);
            while (r1 == i);
            do r2 = dist(gen);
            while (r2 == i || r2 == r1);
            do r3 = dist(gen);
            while (r3 == i || r3 == r1 || r3 == r2);

            auto [x1, y1, a1] = population[r1];
            auto [x2, y2, a2] = population[r2];
            auto [x3, y3, a3] = population[r3];

            // Mutation: v = x_r1 + F * (x_r2 - x_r3)
            double vx = x1 + F * (x2 - x3);
            double vy = y1 + F * (y2 - y3);
            double va = a1 + F * (a2 - a3);

            // Crossover: generate trial vector u by mixing the mutant vector with the target vector
            double trial_x = vx;
            double trial_y = vy;
            double trial_a = va;
            for (int j = 0; j < 3; j++) {
                if (dis(gen) > CR) {
                    if (j == 0) trial_x = std::get<0>(population[i]);
                    if (j == 1) trial_y = std::get<1>(population[i]);
                    if (j == 2) trial_a = std::get<2>(population[i]);
                }
            }

            // Evaluate the trial vector
            int eval = match_count(gmap, pt, trial_x, trial_y, trial_a, csize, originX, originY, width, height);

            // Selection: if trial is better, replace the target individual
            if (eval > best_eval) {
                best_eval = eval;
                best_x = trial_x;
                best_y = trial_y;
                best_a = trial_a;
            }

            if (eval > match_count(gmap, pt, std::get<0>(population[i]), std::get<1>(population[i]), std::get<2>(population[i]), csize, originX, originY, width, height)) {
                population[i] = std::make_tuple(trial_x, trial_y, trial_a);
            }
        }
    }

    return std::make_tuple(best_x, best_y, best_a, best_eval);
}

// 円を描く関数
void draw_circle(uint32_t* pixels, int width, int height, int centerX, int centerY, int radius, uint32_t color, int pitch) {
    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
            if (x * x + y * y <= radius * radius) {  // 円の方程式
                int px = centerX + x;
                int py = centerY + y;
                if (px >= 0 && px < width && py >= 0 && py < height) { // 範囲内チェック
                    pixels[py * (pitch / 4) + px] = color;
                }
            }
        }
    }
}

int main (int argc, char *argv[]) {

    int DATA_SKIP = 1;

    const std::string STORE_ROOT_DIR_NAME = "slam_result_250406-1";
    if (!fs::exists(STORE_ROOT_DIR_NAME)) {
        std::cout << "指定したディレクトリがありません: " << STORE_ROOT_DIR_NAME << "\n";
        return 1;
    }
    // mapInfo.lua から設定値を読み込む
    lua_State* L = luaL_newstate();   // Lua仮想マシン作成
    luaL_openlibs(L);                 // 標準ライブラリ読み込み

    std::string PATH_TO_MAPINFO = STORE_ROOT_DIR_NAME + "/mapInfo.lua";
    if (luaL_dofile(L, PATH_TO_MAPINFO.c_str()) != LUA_OK) {
        std::cerr << "Luaファイル読み込み失敗: " << lua_tostring(L, -1) << std::endl;
        lua_close(L);
        return 1;
    }

    if (!lua_istable(L, -1)) {
        std::cerr << "返り値がテーブルではありません\n";
        lua_close(L);
        return 1;
    }

    // テーブルの値を取得
    lua_getfield(L, -1, "originX");
    int originX = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "originY");
    int originY = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "csize");
    double csize = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "minX");
    double minX = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "maxX");
    double maxX = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "minY");
    double minY = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "maxY");
    double maxY = lua_tonumber(L, -1);
    lua_pop(L, 1);
    lua_close(L);

    int width = int((maxX - minX)/csize);
    int height = int((maxY - minY)/csize);

    double Wxy = 0.5;      // 探索範囲[m]
    double Wa  = M_PI/5;    // 角度[rad]

    const std::string PATH_TO_URGLOG = "../data250405/urglog";

    std::vector<std::vector<int>> gmap(height, std::vector<int>(width, 0));

    std::vector<Point> pt1;
    std::vector<Pose> robot_poses;

    double current_x = 0;
    double current_y = 0;
    double current_a = 0;

    double p_odo_x = 0;
    double p_odo_y = 0;
    double p_odo_a = 0;

    std::ifstream urglog_File;
    std::ifstream robot_poses_File;

    //urglog_File.open(PATH_TO_URGLOG);
    robot_poses_File.open(STORE_ROOT_DIR_NAME + "/robot_poses.txt");

    double robot_ts, robot_x, robot_y, robot_a;
    std::vector<Pose> robot_poses_list;
    while (1) {
        robot_poses_File >> robot_ts >> robot_x >> robot_y >> robot_a;
        if(robot_poses_File.eof()) break;
        //std::cout << robot_x << " " << robot_y << " " << robot_a << "\n";
        robot_poses_list.emplace_back(Pose(robot_ts, robot_x, robot_y, robot_a));
    }

    // robot_poses_list に合わせて対応するtsのurglogをnodeに登録する
    urglog_File.open(PATH_TO_URGLOG);
    std::vector<Node> node;
    for (auto p : robot_poses_list) {
        long long ts = p.ts;
        //std::cout << ts << std::endl;
        std::string type;
        std::string dummy;
        long long timestamp;

        while(1) { 
            urglog_File >> type;
            if(urglog_File.eof()) {
                urglog_File.close();
                break;
            }

            if (type == "LASERSCANRT") {
                urglog_File >> timestamp;
                //std::cout << timestamp << std::endl;
                if (timestamp != ts) {
                    std::getline(urglog_File, dummy);
                } else {
                    //std::cout << "hit!" << std::endl;

                    int count;
                    double START_ANGLE, END_ANGLE, deltaTH;
                    int max_echo_size;
                    std::vector<Point> pt;
                    long intensity;
                    long long timestamp_end;

                    urglog_File >> count >> START_ANGLE >> END_ANGLE >> deltaTH >> max_echo_size;

                    long max_r = 0;
                    long r;
                    double th = START_ANGLE * M_PI/180.0;
                    for (int i = 0; i < count/(max_echo_size+1); i+=DATA_SKIP) {
                        urglog_File >> r;
                        if(max_r < r) {
                            max_r = r;
                        }
                        if (r > 200) {
                            double x = (double)r * cos(th) / 1000.0;
                            double y = (double)r * sin(th) / 1000.0;
                            pt.emplace_back(x, y);
                        }
                        th += deltaTH * M_PI/180.0;
                        // マルチエコーと反射強度を読み飛ばす
                        urglog_File >> r >> r >> intensity;
                    }
                    double odo_x, odo_y, odo_a;
                    urglog_File >> odo_x >> odo_y >> odo_a >> timestamp_end;
                    node.emplace_back(Node(p, pt));
                    break;
                }
            }
        }
    }
    urglog_File.close();

    gmap = update_trajectory(gmap, robot_poses_list, width, height, originX, originY, csize);
    int count = 0;
    auto start = std::chrono::high_resolution_clock::now();
    gmap = update_map_all_node(node, gmap, width, height, originX, originY, csize);
    //for (auto nd: node) { 
    //    gmap = update_map(gmap, nd.pt, nd.p.x, nd.p.y, nd.p.a, width, height, originX, originY, csize);
    //    if (count % 100 == 0) {
    //        auto end = std::chrono::high_resolution_clock::now();
    //        std::chrono::duration<double> elapsed = end - start;
    //        std::cout << count << "/" << node.size() << " " << std::fixed << std::setprecision(1) 
    //            << double(count)/node.size()*100 << "% " << elapsed << std::endl;
    //    }
    //    count++;
    //}
    cv::Mat retimg = gmap_show(gmap, width, height);

    //cv::imshow("map", retimg);
    //int ret = cv::waitKey(10);
    // ncursesの初期化
    initscr();
    raw();            // 入力をそのまま受け取る
    keypad(stdscr, TRUE); // 特殊キーを有効にする
    noecho();         // 入力を画面に表示しない
    curs_set(0);      // カーソルを非表示にする

    printw("[n]ext or [p]revious index or [q]uit:\n");
    refresh();

    int current_node_index = 0;

    while (true) {
        int ch = getch(); // ユーザーの入力を受け取る
        if (ch == 'q') {  // 'q'が押されたら終了
            break;
        }

        bool doRefresh = false;
        bool isSelect = false;
        // n または p の入力があった場合に表示
        if (ch == 'n') {
            current_node_index += 1;
            if (current_node_index >= node.size()) {
                current_node_index = 0;
            }
            doRefresh = true;
            clear(); // 画面をクリア
            printw("current_node_index: %d\n", current_node_index); // 入力された文字を表示
        } 
        else if (ch == 'N') {
            current_node_index += 10;
            if (current_node_index >= node.size()) {
                current_node_index = 0;
            }
            doRefresh = true;
            clear(); // 画面をクリア
            printw("current_node_index: %d\n", current_node_index); // 入力された文字を表示
        } 
        else if (ch == 'h') {
            current_node_index += 100;
            if (current_node_index >= node.size()) {
                current_node_index = 0;
            }
            doRefresh = true;
            clear(); // 画面をクリア
            printw("current_node_index: %d\n", current_node_index); // 入力された文字を表示
        } 
        else if (ch == 'p') {
            current_node_index -= 1;
            if (current_node_index < 0) {
                current_node_index = node.size()-1;
            }
            doRefresh = true;
            clear(); // 画面をクリア
            printw("current_node_index: %d\n", current_node_index); // 入力された文字を表示
        } 
        else if (ch == 'P') {
            current_node_index -= 10;
            if (current_node_index < 0) {
                current_node_index = node.size()-1;
            }
            doRefresh = true;
            clear(); // 画面をクリア
            printw("current_node_index: %d\n", current_node_index); // 入力された文字を表示
        }
        else if (ch == 'l') {
            current_node_index -= 100;
            if (current_node_index < 0) {
                current_node_index = node.size()-1;
            }
            doRefresh = true;
            clear(); // 画面をクリア
            printw("current_node_index: %d\n", current_node_index); // 入力された文字を表示
        }
        else if (ch == 's') {
            isSelect = true;
            doRefresh = true;
            clear(); // 画面をクリア
            printw("current_node_index: %d\n", current_node_index); // 入力された文字を表示
        }

        if (doRefresh) {
            refresh();  // 画面を更新
            auto [ix, iy] = xy2index(node[current_node_index].p.x, node[current_node_index].p.y, csize, originX, originY);
            cv::Mat disp_img = retimg.clone();
            cv::circle(disp_img, cv::Point(ix, iy), 50, cv::Scalar(0, 0, 0), -1, 8);
            cv::imshow("map_adjust", disp_img);
            doRefresh = false;
        }

        if (isSelect) {
            // 調整
            // current_nodeから先の相対姿勢を記録する
            Node prev_node = node[current_node_index];
            std::vector<Pose> relative_pose_list;
            for (int ind = current_node_index+1; ind < node.size(); ind++) {
                Node curr_node = node[ind];
                double delta_x = curr_node.p.x - prev_node.p.x;
                double delta_y = curr_node.p.y - prev_node.p.y;
                double delta_a = curr_node.p.a - prev_node.p.a;
                double u = cos(prev_node.p.a) * delta_x + sin(prev_node.p.a) * delta_y;
                double v =-sin(prev_node.p.a) * delta_x + cos(prev_node.p.a) * delta_y;
                double a = delta_a;
                a = normalize_th(a);
                relative_pose_list.emplace_back(0, u, v, a);
                prev_node = curr_node;
            }
            while (1) {
                int ch = getch(); // ユーザーの入力を受け取る
                if (ch == 'q') {  // 'q'が押されたら終了
                    break;
                }
                if (ch == 'u') {
                    node[current_node_index].p.a += 0.5*M_PI/180;
                } else if (ch == 'i') {
                    node[current_node_index].p.a -= 0.5*M_PI/180;
                } else if (ch == 'h') {
                    node[current_node_index].p.x -= 0.025;
                } else if (ch == 'j') {
                    node[current_node_index].p.y -= 0.025;
                } else if (ch == 'k') {
                    node[current_node_index].p.y += 0.025;
                } else if (ch == 'l') {
                    node[current_node_index].p.x += 0.025;
                }
                prev_node = node[current_node_index];
                // current_node_index から先の姿勢を相対姿勢で修正する
                for (int ind = 0; ind < relative_pose_list.size(); ind++) {
                    double u = relative_pose_list[ind].x;
                    double v = relative_pose_list[ind].y;
                    double a = relative_pose_list[ind].a;
                    double cs = cos(prev_node.p.a);
                    double sn = sin(prev_node.p.a);
                    node[current_node_index+ind+1].p.x = prev_node.p.x + u*cs - v*sn;
                    node[current_node_index+ind+1].p.y = prev_node.p.y + u*sn + v*cs;
                    node[current_node_index+ind+1].p.a = prev_node.p.a + a;
                    prev_node = node[current_node_index+ind+1];
                }
                std::vector<std::vector<int>> gmap(height, std::vector<int>(width, 0));
                //gmap = update_trajectory(gmap, robot_poses_list, width, height, originX, originY, csize);
                gmap = update_map_all_node(node, gmap, width, height, originX, originY, csize);
                cv::Mat img = gmap_show(gmap, width, height);
                retimg = img.clone();
                cv::Mat disp_img = retimg.clone();
                cv::imshow("map_adjust", disp_img);
                cv::waitKey(10);
            }
            isSelect = false;
        }
        cv::waitKey(10);
    }

    // ncursesを終了
    endwin();
    cv::imwrite("rebuild_opt.png", retimg);
    std::ofstream result_file("rebuild_robot_pose.txt");
    for (Node n: node) {
        result_file << n.p.ts << " " << n.p.x << " " << n.p.y << " " << n.p.a << "\n";
    }
    result_file.close();
    current_node_index = 0;
    // スレッドの終了を待機
    return 0;
}
