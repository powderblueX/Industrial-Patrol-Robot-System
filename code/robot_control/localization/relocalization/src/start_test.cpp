#include <iostream>
#include <unistd.h>    // fork, exec
#include <sys/types.h> // pid_t
#include <sys/wait.h>  // waitpid
#include <signal.h>    // kill
#include <cstdlib>
#include <chrono>
#include <thread>

int main()
{
    pid_t pid = fork();

    if (pid < 0)
    {
        std::cerr << "Failed to fork" << std::endl;
        return 1;
    }

    if (pid == 0)
    {
        // 子进程：执行 ROS2 launch 命令，自动 source 环境
        execlp("bash", "bash", "-c",
               "source /opt/ros/humble/setup.bash && "
               "ros2 launch point_lio mapping_mid360.launch.py",
               nullptr);
        std::cerr << "Failed to exec ros2 launch" << std::endl;
        exit(1); // exec 失败必须退出子进程
    }

    // 父进程
    std::cout << "ROS2 launch started in child process, PID = " << pid << std::endl;
    std::cout << "Main program is running..." << std::endl;

    int status = 0;
    // 示例：等待用户输入后终止子进程
    std::cout << "Press ENTER to stop ROS2 process..." << std::endl;
    std::cin.get();
    kill(pid, SIGINT);
    while (true)
    {
        // 非阻塞检测子进程状态
        pid_t result = waitpid(pid, &status, WNOHANG);
        if (result == 0)
        {
            // 子进程还在运行，继续等
            std::cout << "[INFO] ROS2 process still running..." << std::endl;
        }
        else if (result == pid)
        {
            // 子进程已退出
            if (WIFEXITED(status))
            {
                std::cout << "[INFO] ROS2 process exited normally with code "
                          << WEXITSTATUS(status) << std::endl;
            }
            else if (WIFSIGNALED(status))
            {
                std::cout << "[INFO] ROS2 process terminated by signal "
                          << WTERMSIG(status) << std::endl;
            }
            else
            {
                std::cout << "[INFO] ROS2 process exited abnormally." << std::endl;
            }
            break;
        }
        else
        {
            std::cerr << "[ERROR] waitpid() failed" << std::endl;
            break;
        }

        // 检查间隔
        std::this_thread::sleep_for(std::chrono::seconds(1));

    }
    // 父进程
    // std::cout << "ROS2 launch started in child process, PID = " << pid << std::endl;

    // // 主程序继续运行（可以加逻辑）
    // std::cout << "Main program is running..." << std::endl;

    // // 示例：等待用户输入后终止子进程
    // std::cout << "Press ENTER to stop ROS2 process..." << std::endl;
    // std::cin.get();

    // // 杀掉子进程（向其发送SIGINT或SIGTERM）
    // kill(pid, SIGINT);

    // // 等待子进程退出，避免僵尸进程
    // int status;
    // waitpid(pid, &status, 0);
    return 0;
}
