/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

- Added ROS2 support.
- Added mutex and lock guards to make the save_file_callback thread-safe.
    (C) Siemens AG, 2024, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)

- Added path traversal and extension checks to prevent unauthorized file access.
- Added parameters to enable/disable saving and overwriting files (disabled by default).
- Removed auto package generation for security.
    (C) Siemens AG, 2026, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)
*/


#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "file_server2/srv/get_binary_file.hpp"
#include "file_server2/srv/save_binary_file.hpp"
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <mutex>
#include <filesystem>
#include <unordered_set>
#include <algorithm>

using namespace std::placeholders;

class FileServerNode : public rclcpp::Node
{
public:
    FileServerNode() : Node("file_server")
    {
        this->declare_parameter("allow_save",      false);
        this->declare_parameter("allow_overwrite", false);

        get_file_service_ = this->create_service<file_server2::srv::GetBinaryFile>(
            "/file_server/get_file",
            std::bind(&FileServerNode::get_file_callback, this, _1, _2));

        save_file_service_ = this->create_service<file_server2::srv::SaveBinaryFile>(
            "/file_server/save_file",
            std::bind(&FileServerNode::save_file_callback, this, _1, _2));

        if (this->get_parameter("allow_save").as_bool())
        {
            RCLCPP_INFO(this->get_logger(), "Save enabled. Overwrite: %s",
                this->get_parameter("allow_overwrite").as_bool() ? "yes" : "no");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Save disabled.");
        }

        RCLCPP_INFO(this->get_logger(), "ROS2 File Server initialized.");
    }

private:
    static const std::unordered_set<std::string>& allowed_extensions()
    {
        static const std::unordered_set<std::string> exts =
        {
            ".urdf", ".xacro", ".stl", ".dae", ".obj", ".mtl",
            ".png",  ".jpg",   ".jpeg",".bmp", ".tga", ".yaml",
            ".tif",  ".tiff",  ".gif", ".material"
        };
        return exts;
    }

    bool is_extension_allowed(const std::string& path)
    {
        auto ext = std::filesystem::path(path).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        return allowed_extensions().count(ext) > 0;
    }

    bool has_traversal(const std::string& path)
    {
        for (const auto& part : std::filesystem::path(path))
            if (part == ".." || part == ".")
                return true;
        return false;
    }

    bool is_path_safe(const std::string& base_dir, const std::string& full_path)
    {
        try
        {
            auto base   = std::filesystem::canonical(base_dir);
            auto target = std::filesystem::weakly_canonical(full_path);
            auto [end, _] = std::mismatch(base.begin(), base.end(), target.begin());
            return end == base.end();
        } catch (...)
        {
            return false;
        }
    }

    bool validate_path(const std::string& filepath, const std::string& base_dir, const std::string& full_path)
    {
        if (has_traversal(filepath))
        {
            RCLCPP_WARN(this->get_logger(), "Path traversal attempt blocked: %s", full_path.c_str());
            return false;
        }
        if (!is_extension_allowed(filepath))
        {
            RCLCPP_WARN(this->get_logger(), "Extension not allowed: %s", full_path.c_str());
            return false;
        }
        if (!is_path_safe(base_dir, full_path))
        {
            RCLCPP_WARN(this->get_logger(), "Path escapes package directory: %s", full_path.c_str());
            return false;
        }
        return true;
    }


    bool parse_package_url(const std::string& name, std::string& base_dir, std::string& filepath, std::string& full_path)
    {
        if (name.compare(0, 10, "package://") != 0)
        {
            RCLCPP_WARN(this->get_logger(), "Only \"package://\" addresses allowed.");
            return false;
        }

        std::string address = name.substr(10);
        std::string package = address.substr(0, address.find("/"));
        filepath = address.substr(package.length());

        // TODO: need better exception handling
        try
        {
            base_dir = ament_index_cpp::get_package_share_directory(package);
        } catch (...)
        {
            base_dir = "";
        }
        full_path = base_dir + filepath;
        return true;
    }

    // callbacks

    void get_file_callback(
        const std::shared_ptr<file_server2::srv::GetBinaryFile::Request> request,
        std::shared_ptr<file_server2::srv::GetBinaryFile::Response> response)
    {
        std::string base_dir, filepath, full_path;
        if (!parse_package_url(request->name, base_dir, filepath, full_path)) return;
        if (base_dir.empty()) { RCLCPP_WARN(this->get_logger(), "Package not found."); return; }
        if (!validate_path(filepath, base_dir, full_path)) return;

        std::ifstream file(full_path, std::ios::binary);
        if (!file.is_open())
        {
            RCLCPP_WARN(this->get_logger(), "File not found: %s", full_path.c_str());
            return;
        }

        response->value.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
        RCLCPP_INFO(this->get_logger(), "get_file: %s", request->name.c_str());
    }

    void save_file_callback(
        const std::shared_ptr<file_server2::srv::SaveBinaryFile::Request> request,
        std::shared_ptr<file_server2::srv::SaveBinaryFile::Response> response)
    {
        std::lock_guard<std::mutex> guard(save_file_mutex_);
        if (!this->get_parameter("allow_save").as_bool())
        {
            RCLCPP_WARN(this->get_logger(), "Save service is disabled. Enable with parameter \"allow_save\".");
            return;
        }

        // TODO: need better exception handling
        std::string base_dir, filepath, full_path;
        if (!parse_package_url(request->name, base_dir, filepath, full_path)) return;
        if (!validate_path(filepath, base_dir, full_path)) return;

        if (!this->get_parameter("allow_overwrite").as_bool() && std::filesystem::exists(full_path))
        {
            RCLCPP_WARN(this->get_logger(), "Overwrite blocked: %s", full_path.c_str());
            return;
        }

        std::filesystem::create_directories(std::filesystem::path(full_path).parent_path());

        std::ofstream file(full_path, std::ios::binary);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open for write: %s", full_path.c_str());
            return;
        }

        file.write(reinterpret_cast<const char*>(request->value.data()), request->value.size());
        response->name = request->name;
        RCLCPP_INFO(this->get_logger(), "save_file: %s", request->name.c_str());
    }

    rclcpp::Service<file_server2::srv::GetBinaryFile>::SharedPtr get_file_service_;
    rclcpp::Service<file_server2::srv::SaveBinaryFile>::SharedPtr save_file_service_;
    std::mutex save_file_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FileServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
