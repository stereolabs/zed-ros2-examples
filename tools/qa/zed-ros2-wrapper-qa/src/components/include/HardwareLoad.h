#ifndef HardwareLoad_h
#define HardwareLoad_h

#include <thread>
#include <fstream>
#include <unistd.h>

#ifdef _WIN32
#include "TCHAR.h"
#include "pdh.h"
#include "windows.h"
#include "psapi.h"
#include <io.h>

#else
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "sys/times.h"
// #include "sys/vtimes.h"
#include "sys/types.h"
#include "sys/sysinfo.h"
#include <sys/resource.h>
#endif


#include <cuda.h>
#include <cuda_runtime.h>
// magic numbers, do not change them
#define NVAPI_MAX_PHYSICAL_GPUS   64
#define NVAPI_MAX_USAGES_PER_GPU  34

// function pointer types
typedef int *(*NvAPI_QueryInterface_t)(unsigned int offset);
typedef int(*NvAPI_Initialize_t)();
typedef int(*NvAPI_EnumPhysicalGPUs_t)(int **handles, int *count);
typedef int(*NvAPI_GPU_GetUsages_t)(int *handle, unsigned int *usages);

class HardwareLoad
{
public:
    HardwareLoad();
    ~HardwareLoad();

    void launch();
    void join();

    void compute();

    float getCPUload() { return cpuload; }
    float getGPUload() { return gpuload; }
    float getRAMload() { return ramload; }
    float getGPUMemload() { return gpumemload; }

    std::vector<float> getCPUloadMeasures() {return cpu_load_measures;}
    std::vector<float> getGPUloadMeasures() {return gpu_load_measures;}

    void clear() {
        cpu_load_measures.clear();
        gpu_load_measures.clear();
    }

    void activate() {
        is_activated = true;
    }

    void deactivate() {
        is_activated = false;
    }
    
    protected:
    void run();
    
    private:
    std::thread m_thread;
    bool running;
    float cpuload;
    float gpuload;
    float gpumemload;
    float ramload;
    
    bool is_activated;
    
    std::vector<float> cpu_load_measures;
    std::vector<float> gpu_load_measures;
    
    bool can_read_gpuload;
    
    void createCPUusage();
    float CalculateCPUusage();
    
    void createGPUusage();
    float CalculateGPUusage();
    
    void createGPUMemoryUsage();
    float CalculateGPUMemoryUsage();
    
    void createRAMusage();
    float CalculateRAMusage();

    size_t gpu_freem, gpu_totalMem;
#ifdef _WIN32
    NvAPI_QueryInterface_t      NvAPI_QueryInterface = NULL;
    NvAPI_Initialize_t          NvAPI_Initialize = NULL;
    NvAPI_EnumPhysicalGPUs_t    NvAPI_EnumPhysicalGPUs = NULL;
    NvAPI_GPU_GetUsages_t       NvAPI_GPU_GetUsages = NULL;
    int         *gpuHandles[NVAPI_MAX_PHYSICAL_GPUS];
    unsigned int gpuUsages[NVAPI_MAX_USAGES_PER_GPU];
    ULARGE_INTEGER lastCPU, lastSysCPU, lastUserCPU;
    int numProcessors;
    HANDLE self;
    DWORDLONG totalVirtualMem;
    HMODULE hmod;
#else
    unsigned long long lastProcessTime, lastTotalTime;
    int numProcessors;
    long long totalVirtualMem;
    int gpuMethod;
#endif
};

#ifndef _WIN32

inline int ram_parseLine(char* line) {
    // This assumes that a digit will be found and the line ends in " Kb".
    int i = strlen(line);
    const char* p = line;
    while (*p < '0' || *p > '9') p++;
    line[i - 3] = '\0';
    i = atoi(p);
    return i;
}
#endif

inline HardwareLoad::HardwareLoad() {
    cpuload = -1;
    gpuload = -1;
    ramload = -1;
    gpumemload = -1;
    #ifndef _WIN32
        gpuMethod = -1;
    #endif
    can_read_gpuload = false;
    running = false;
    is_activated = false;
}

inline void HardwareLoad::launch() {
    if (running || is_activated) {
        return;
    }
    createCPUusage();
    createGPUusage();
    createGPUMemoryUsage();
    createRAMusage();
    running = true;
    is_activated = true;
    m_thread = std::thread([this] {run();});
}

inline void HardwareLoad::join() {
    running = false;
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

inline HardwareLoad::~HardwareLoad() {
    if (running) {
        join();
    }
}

inline void HardwareLoad::createCPUusage() {
#ifdef _WIN32
    SYSTEM_INFO sysInfo;
    FILETIME ftime, fsys, fuser;
    GetSystemInfo(&sysInfo);
    numProcessors = sysInfo.dwNumberOfProcessors;
    GetSystemTimeAsFileTime(&ftime);
    memcpy(&lastCPU, &ftime, sizeof(FILETIME));
    self = GetCurrentProcess();
    GetProcessTimes(self, &ftime, &ftime, &fsys, &fuser);
    memcpy(&lastSysCPU, &fsys, sizeof(FILETIME));
    memcpy(&lastUserCPU, &fuser, sizeof(FILETIME));
#else
    FILE* file;
    char line[128];
    file = fopen("/proc/cpuinfo", "r");
    numProcessors = 0;
    while (fgets(line, 128, file) != NULL) {
        if (strncmp(line, "processor", 9) == 0) numProcessors++;
    }
    fclose(file);
#endif
}

inline void HardwareLoad::createGPUMemoryUsage() {
    size_t freemem, totalmem;
    cudaMemGetInfo(&freemem, &totalmem);
    gpu_totalMem = totalmem;
}

inline void HardwareLoad::createGPUusage() {
#ifdef _WIN32
    hmod = LoadLibraryA("nvapi64.dll");
    if (hmod == NULL) {
        //Couldn't find nvapi64.dll
        return;
    }
    // nvapi_QueryInterface is a function used to retrieve other internal functions in nvapi.dll
    NvAPI_QueryInterface = (NvAPI_QueryInterface_t)GetProcAddress(hmod, "nvapi_QueryInterface");
    // some useful internal functions that aren't exported by nvapi.dll
    NvAPI_Initialize = (NvAPI_Initialize_t)(*NvAPI_QueryInterface)(0x0150E828);
    NvAPI_EnumPhysicalGPUs = (NvAPI_EnumPhysicalGPUs_t)(*NvAPI_QueryInterface)(0xE5AC921F);
    NvAPI_GPU_GetUsages = (NvAPI_GPU_GetUsages_t)(*NvAPI_QueryInterface)(0x189A1FDF);
    if (NvAPI_Initialize == NULL || NvAPI_EnumPhysicalGPUs == NULL ||
        NvAPI_EnumPhysicalGPUs == NULL || NvAPI_GPU_GetUsages == NULL) {
        //Couldn't get functions in nvapi.dll
        return;
    }
    // initialize NvAPI library, call it once before calling any other NvAPI functions
    (*NvAPI_Initialize)();
    int gpuCount = 0;
    // gpuUsages[0] must be this value, otherwise NvAPI_GPU_GetUsages won't work
    gpuUsages[0] = (NVAPI_MAX_USAGES_PER_GPU * 4) | 0x10000;
    (*NvAPI_EnumPhysicalGPUs)(gpuHandles, &gpuCount);
    if (gpuCount > 0)
        can_read_gpuload = true;
#else
    can_read_gpuload = true;
#endif
}

inline void HardwareLoad::createRAMusage() {
#ifdef _WIN32
    MEMORYSTATUSEX memInfo;
    memInfo.dwLength = sizeof(MEMORYSTATUSEX);
    GlobalMemoryStatusEx(&memInfo);
    totalVirtualMem = memInfo.ullTotalPageFile;
#else
    struct sysinfo memInfo;
    sysinfo(&memInfo);
    totalVirtualMem = memInfo.totalram;
    //Add other values in next statement to avoid int overflow on right hand side...
    //totalVirtualMem += memInfo.totalswap;
    //totalVirtualMem *= memInfo.mem_unit;
#endif
}

inline void HardwareLoad::run() {
    while (running) {
        if (is_activated) compute();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

inline unsigned long long getTotalSystemTime() {
    std::ifstream statFile("/proc/stat");
    std::string line;
    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;
    
    if (std::getline(statFile, line)) {
        std::istringstream iss(line);
        std::string cpu;
        iss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
        
        return user + nice + system + idle + iowait + irq + softirq + steal;
    }
    
    return 0;
}

inline unsigned long long getProcessTime() {
    std::ifstream procStatFile("/proc/self/stat");
    std::string line;
    
    if (std::getline(procStatFile, line)) {
        std::istringstream iss(line);
        std::vector<std::string> tokens;
        std::string token;
        
        while (iss >> token) {
            tokens.push_back(token);
        }
        
        // The 14th, 15th, 16th, and 17th fields represent CPU time
        // (utime, stime, cutime, cstime) https://man7.org/linux/man-pages/man5/proc_pid_stat.5.html
        if (tokens.size() >= 17) {
            unsigned long long utime = std::stoull(tokens[13]);
            unsigned long long stime = std::stoull(tokens[14]);
            unsigned long long cutime = std::stoull(tokens[15]);
            unsigned long long cstime = std::stoull(tokens[16]);
            
            return utime + stime + cutime + cstime;
        }
    }
    
    return 0;
}

inline void HardwareLoad::compute() {
    cpuload = CalculateCPUusage();
    gpumemload = CalculateGPUMemoryUsage();
    ramload = CalculateRAMusage();
    if (can_read_gpuload) {
        gpuload = CalculateGPUusage();
        gpu_load_measures.push_back(gpuload);
    }
    cpu_load_measures.push_back(cpuload);
}

inline float HardwareLoad::CalculateCPUusage() {
#ifdef _WIN32
    FILETIME ftime, fsys, fuser;
    ULARGE_INTEGER now, sys, user;
    double percent;
    GetSystemTimeAsFileTime(&ftime);
    memcpy(&now, &ftime, sizeof(FILETIME));
    GetProcessTimes(self, &ftime, &ftime, &fsys, &fuser);
    memcpy(&sys, &fsys, sizeof(FILETIME));
    memcpy(&user, &fuser, sizeof(FILETIME));
    percent = (sys.QuadPart - lastSysCPU.QuadPart) +
        (user.QuadPart - lastUserCPU.QuadPart);
    percent /= (now.QuadPart - lastCPU.QuadPart);
    percent /= numProcessors;
    lastCPU = now;
    lastUserCPU = user;
    lastSysCPU = sys;
    if (percent < 0) percent = 0;
    if (percent > 1) percent = 1;
    return percent * 100.0;
#else
    unsigned long long currentTotalTime = getTotalSystemTime();
    unsigned long long currentProcessTime = getProcessTime();
    
    unsigned long long totalTimeDelta = currentTotalTime - lastTotalTime;
    unsigned long long processTimeDelta = currentProcessTime - lastProcessTime;
    
    lastTotalTime = currentTotalTime;
    lastProcessTime = currentProcessTime;
    
    if (totalTimeDelta == 0) {
        return 0.0;
    }
    double cpuLoad = (100.0 * processTimeDelta) / totalTimeDelta;
    
    return cpuLoad;
#endif
}

inline std::string ssystem(const char *command) {
    #ifdef _WIN32
    char tmpname[MAX_PATH];
    GetTempPathA(MAX_PATH, tmpname); // Get temp directory path
    strcat(tmpname, "ssystemXXXXXX.tmp");
    int fd = _mktemp_s(tmpname, sizeof(tmpname));
#else
    char tmpname[] = "/tmp/ssystemXXXXXX";
    int fd = mkstemp(tmpname);
#endif
    if (fd == -1) {
        perror("mkstemp");
        return "";
    }
    close(fd);

    std::string scommand = command;
    std::string cmd = scommand + " >> " + tmpname;
    std::system(cmd.c_str());

    std::ifstream file(tmpname);
    std::stringstream result;
    if (file) {
        result << file.rdbuf();
        file.close();
    }

    std::remove(tmpname);
    return result.str();
}

inline float HardwareLoad::CalculateGPUusage() {
#ifdef _WIN32
    (*NvAPI_GPU_GetUsages)(gpuHandles[0], gpuUsages);
    int usage = gpuUsages[3];
    return usage;
#else
    if (gpuMethod == -1 || gpuMethod == 1) {
        //TX1
        std::ifstream inFile;
        inFile.open("/sys/devices/platform/host1x/57000000.gpu/load");
        if (inFile) {
            int load = 0;
            inFile >> load;
            load /= 10;
            inFile.close();
            can_read_gpuload = true;
            gpuMethod = 1;
            return load;
        }
    }
    if (gpuMethod == -1 || gpuMethod == 2) {
        //TX2
        std::ifstream inFile;
        inFile.open("/sys/devices/platform/host1x/17000000.gp10b/load");
        if (inFile) {
            int load = 0;
            inFile >> load;
            load /= 10;
            inFile.close();
            can_read_gpuload = true;
            gpuMethod = 2;
            return load;
        }
    }
    if (gpuMethod == -1 || gpuMethod == 4) {
        //XAVIER
        std::ifstream inFile;
        inFile.open("/sys/devices/platform/host1x/17000000.gv11b/load");
        if (inFile) {
            int load = 0;
            inFile >> load;
            load /= 10;
            inFile.close();
            can_read_gpuload = true;
            gpuMethod = 4;
            return load;
        }
    }
    if (gpuMethod == -1 || gpuMethod == 5) {
      //ORIN
      std::ifstream inFile;
      inFile.open("/sys/devices/platform/host1x/17000000.ga10b/load");
      if (!inFile.is_open()) {
          // this is the actual file on newer JP (36.4 for instance)
        inFile.open("/sys/class/devfreq/17000000.gpu/device/load");
      }
      if (!inFile.is_open()) {
          // this is the actual file on newer JP with super mode
        inFile.open("/sys/class/devfreq/17000000.gpu/load");
      }
      if (inFile) {
        int load = 0;
        inFile >> load;
        load /= 10;
        inFile.close();
        can_read_gpuload = true;
        gpuMethod = 5;
        return load;
      }
    }
    if (gpuMethod == -1 || gpuMethod == 3) {
        //use nvidia-smi
        std::string result = ssystem("nvidia-smi --query-gpu=utilization.gpu --format=csv");
        if (result.find('\n') != std::string::npos) {
            if (result.find('%') != std::string::npos) {
                int load = 0;
                sscanf(result.substr(result.find('\n'), result.find('%')).c_str(), "%i", &load);
                can_read_gpuload = true;
                gpuMethod = 3;
                return load;
            }
        }
    }
    return -1;
#endif
}

inline float HardwareLoad::CalculateGPUMemoryUsage() {
    size_t freemem, totalmem;
    cudaMemGetInfo(&freemem, &totalmem);
    gpu_freem = freemem;
    size_t used_mem = gpu_totalMem - gpu_freem;
    //return (100.0 * used_mem) / gpu_totalMem;
    return used_mem / 1048576.f; // to MB
}

inline float HardwareLoad::CalculateRAMusage() {
#ifdef _WIN32
    HANDLE hProc = GetCurrentProcess();
    PROCESS_MEMORY_COUNTERS_EX info;
    info.cb = sizeof(info);
    GetProcessMemoryInfo(hProc, (PROCESS_MEMORY_COUNTERS*)& info, info.cb);
    SIZE_T virtualMemUsedByMe = info.PrivateUsage;
    //return (100.0 * virtualMemUsedByMe) / totalVirtualMem;
    return virtualMemUsedByMe / 1048576.f; // to MB
#else
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    //return (100.0 * r_usage.ru_maxrss * 1024) / totalVirtualMem;
    return (r_usage.ru_maxrss * 1024) / 1048576.f; // to MB
#endif
}
#endif // HardwareLoad_h