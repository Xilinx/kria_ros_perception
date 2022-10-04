/*
      ____  ____
     /   /\/   /
    /___/  \  /   Copyright (c) 2021, Xilinx®.
    \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
     \   \
     /   /
    /___/   /\
    \   \  /  \
     \___\/\___\

*/

#include "common/utilities.hpp"
#include <dirent.h>
#include <cstring>
// #include <sys/sysinfo.h>

/**
 * @brief Get the xilinx devices object
 *
 * @return std::vector<cl::Device>
 */
std::vector<cl::Device> get_xilinx_devices() {
    size_t i;
    cl_int err;
    std::vector<cl::Platform> platforms;
    err = cl::Platform::get(&platforms);
    cl::Platform platform;
    for (i  = 0 ; i < platforms.size(); i++) {
        platform = platforms[i];
        std::string platformName = platform.getInfo<CL_PLATFORM_NAME>(&err);
        if (platformName == "Xilinx") {
            std::cout << "INFO: Found Xilinx Platform" << std::endl;
            break;
        }
    }
    if (i == platforms.size()) {
        std::cout << "ERROR: Failed to find Xilinx platform" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Getting ACCELERATOR Devices and selecting 1st such device
    std::vector<cl::Device> devices;
    err = platform.getDevices(CL_DEVICE_TYPE_ACCELERATOR, &devices);
    return devices;
}

/**
 * @brief function to process the accelerated kernel and prepare it for use
 *
 * @param xclbin_file_name the accelerated kernel binary
 * @param nb number of bytes
 * @return char* pointer to the loaded kernel
 */
char* read_binary_file(const std::string &xclbin_file_name, unsigned &nb) {  // NOLINT
    if (access(xclbin_file_name.c_str(), R_OK) != 0) {
        printf("ERROR: %s xclbin not available please build\n",
          xclbin_file_name.c_str());
        exit(EXIT_FAILURE);
    }
    // Loading XCL Bin into char buffer
    std::cout << "INFO: Loading '" << xclbin_file_name << "'\n";
    std::ifstream bin_file(xclbin_file_name.c_str(), std::ifstream::binary);
    bin_file.seekg(0, bin_file.end);
    nb = bin_file.tellg();
    bin_file.seekg(0, bin_file.beg);
    char *buf = new char[nb];
    bin_file.read(buf, nb);
    return buf;
}


/**
 * @brief Returns the number of hwmon devices registered under /sys/class/hwmon
 *
 * @return       num_hwmon_devices: Number of registered hwmon devices
 *
 */
int count_hwmon_reg_devices() {
    // find number of hwmon devices listed under
    int num_hwmon_devices;
    DIR *d;
    struct dirent *dir;

    num_hwmon_devices = 0;
    d = opendir("/sys/class/hwmon");

    if (!d) {
        printf("Unable to open /sys/class/hwmon path\n");
        return(errno);
    }

    while ((dir = readdir(d)) != NULL) {
        if (strstr(dir->d_name, "hwmon")) {
            num_hwmon_devices++;
        }
    }
    closedir(d);
    return(num_hwmon_devices);
}

/**
 * @brief Reads the sysfs enteries for a given sysfs file
 *
 * @param	filename: sysfs path
 * @param	value: value read from sysfs entry
 * @return       None
 *
 */
int read_sysfs_entry(char* filename, char* value)
{
    FILE *fp;
    fp = fopen(filename, "r");

    if (fp == NULL) {
        printf("Unable to open %s\n", filename);
        return(errno);
    }

    fscanf(fp, "%s", value);
    fclose(fp);
    return(0);
}

/**
 * @brief Returns hwmon_id of the specified device:
 *
 * @param        name: device name for which hwmon_id needs to be identified
 * @return       hwmon_id int value
 *
 */
int get_device_hwmon_id(int verbose_flag, const char* name)
{
    // find number of hwmon devices listed under
    int num_hwmon_devices, hwmon_id;
    char hwmon_id_str[50];
    char *device_name;
    char *filename;

    filename = reinterpret_cast<char*>(malloc(255));
    device_name = reinterpret_cast<char*>(malloc(255));
    hwmon_id = -1;
    num_hwmon_devices = count_hwmon_reg_devices();

    for (hwmon_id = 0; hwmon_id < num_hwmon_devices; hwmon_id++) {
        snprintf(hwmon_id_str, sizeof(hwmon_id_str), "%d", hwmon_id);
        snprintf(filename, 255 - strlen(filename), "/sys/class/hwmon/hwmon");
        snprintf(filename + strlen(filename),
            255 - strlen(filename), "%s", hwmon_id_str);
        snprintf(filename + strlen(filename),
            255 - strlen(filename), "%s", "/name");
        read_sysfs_entry(filename, device_name);

        if (!strcmp(name, device_name)) {
            return(hwmon_id);
        }

        if (verbose_flag) {
            printf("filename %s\n", filename);
            printf("device_name = %s\n", device_name);
        }
    }
    free(filename);
    free(device_name);
    return(-1);
}
