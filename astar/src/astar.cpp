#include<algorithm>
#include<cmath>
#include<fstream>
#include<iostream>
#include<limits>
#include<queue>
#include<vector>

struct Node
{
    int row;
    int col;

    int g;
    int h;
    int f;
};

struct CompareNode
{
    bool operator()(const Node& a, const Node& b)const
    {
        if(a.f !=b.f)
        {
            return a.f > b.f;
        }
        return a.h > b.h;
    }
};

struct Parent
{
    int row = -1;
    int col = -1;
};

int main(int argc,char* argv[])
{
    //读取起点和终点
    if (argc != 5)
    {
        std::cerr
            << "Usage: ./astar start_row start_col goal_row goal_col"
            << std::ends;
        return 1;
    }

    int start_row = std::stoi(argv[1]);
    int start_col = std::stoi(argv[2]);

    int goal_row = std::stoi(argv[3]);
    int goal_col = std::stoi(argv[4]);

    //读取地图

    std::ifstream map_file("map.txt");

    if(!map_file.is_open())
    {
        std::cerr << "Cannot open map.txt" << std::ends;
        return 1;
    }

    int rows;
    int cols;

    map_file >> rows >> cols;

    std::vector<std::vector<int>> grid(
        rows,
        std::vector<int>(cols)
    );

    for (int row = 0;row<rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            map_file >> grid[row][col];
        }
    }

    map_file.close();

    //检查起点终点是否有效
    if (start_row < 0 || start_row >= rows ||
        start_col < 0 || start_col >= cols ||
        goal_row < 0 || goal_row >= rows ||
        goal_col < 0 || goal_col >= cols)
    {
        std::cerr << "Start or goal is outside the map."
                  << std::endl;

        return 1;
    }

    if (grid[start_row][start_col] == 1)
    {
        std::cerr << "Start is an obstacle." << std::endl;
        return 1;
    }

    if (grid[goal_row][goal_col] == 1)
    {
        std::cerr << "Goal is an obstacle." << std::endl;
        return 1;
    }

    //需要的数据
    const int INF = std::numeric_limits<int>::max();

    std::vector<std::vector<int>> g_score(
        rows,
        std::vector<int>(cols, INF)
    );


    std::vector<std::vector<bool>> closed(
        rows,
        std::vector<bool>(cols, false)
    );


    std::vector<std::vector<Parent>> parent(
        rows,
        std::vector<Parent>(cols)
    );


    std::priority_queue<
        Node,
        std::vector<Node>,
        CompareNode
    > open_list;

    //保存搜索过的节点
    std::vector<std::pair<int, int>> explored;

    //曼哈顿距离h(n)
    auto heuristic = [&](int row, int col)
    {
        return std::abs(row - goal_row)
             + std::abs(col - goal_col);
    };

    //初始化起点
    g_score[start_row][start_col] = 0;

    int h = heuristic(start_row, start_col);

    open_list.push({
        start_row,
        start_col,
        0,
        h,
        h
    });

    // 上、下、左、右
    const int d_row[] = {-1, 1, 0, 0};
    const int d_col[] = {0, 0, -1, 1};


    bool found = false;

    //A*
    while (!open_list.empty())
    {
        Node current = open_list.top();
        open_list.pop();

        int row = current.row;
        int col = current.col;


        // 已经处理过，跳过
        if (closed[row][col])
        {
            continue;
        }


        // 标记为已经处理
        closed[row][col] = true;

        explored.push_back({row, col});


        // 到达终点
        if (row == goal_row &&
            col == goal_col)
        {
            found = true;
            break;
        }


        //检查四个方向

        for (int direction = 0;
             direction < 4;
             ++direction)
        {
            int next_row =
                row + d_row[direction];

            int next_col =
                col + d_col[direction];


            // 越界
            if (next_row < 0 ||
                next_row >= rows ||
                next_col < 0 ||
                next_col >= cols)
            {
                continue;
            }


            // 障碍物
            if (grid[next_row][next_col] == 1)
            {
                continue;
            }


            // 已经处理
            if (closed[next_row][next_col])
            {
                continue;
            }


            // 从当前节点走一步
            int new_g =
                g_score[row][col] + 1;


            // 找到了更短的路线
            if (new_g < g_score[next_row][next_col])
            {
                g_score[next_row][next_col] = new_g;


                parent[next_row][next_col] = {
                    row,
                    col
                };


                int next_h =
                    heuristic(next_row, next_col);

                int next_f =
                    new_g + next_h;


                open_list.push({
                    next_row,
                    next_col,
                    new_g,
                    next_h,
                    next_f
                });
            }
        }
    }

    //保存搜过的节点
    std::ofstream explored_file(
        "explored.txt"
    );

    for (const auto& point : explored)
    {
        explored_file
            << point.first << " "
            << point.second << "\n";
    }

    explored_file.close();

    //没找到路径
    if (!found)
    {
        std::ofstream path_file("path.txt");
        path_file.close();

        std::cout << "NO_PATH" << std::endl;

        return 0;
    }

    //从终点反向恢复路径
    std::vector<std::pair<int, int>> path;


    int row = goal_row;
    int col = goal_col;


    while (!(row == start_row &&
             col == start_col))
    {
        path.push_back({row, col});


        Parent p =
            parent[row][col];


        row = p.row;
        col = p.col;
    }

    // 把起点也加入路径
    path.push_back({
        start_row,
        start_col
    });


    // 反转
    std::reverse(
        path.begin(),
        path.end()
    );

    //保存最终路径
    std::ofstream path_file("path.txt");


    for (const auto& point : path)
    {
        path_file
            << point.first << " "
            << point.second << "\n";
    }


    path_file.close();

    //输出
    std::cout
        << "FOUND" << std::endl;

    std::cout
        << "Path length: "
        << path.size() - 1
        << std::endl;

    std::cout
        << "Explored nodes: "
        << explored.size()
        << std::endl;


    return 0;

}